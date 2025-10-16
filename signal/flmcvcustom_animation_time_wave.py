import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

# ============ 信号生成函数 ============
def generate_sonar_signal(sigParams, t, custom_wave=None):
    fre = sigParams.get("fre", 150e3)
    Bw = sigParams.get("Bw", 40e3)
    amp = sigParams.get("amp", 5000)
    fd = sigParams.get("fd", 0)
    signaltype = sigParams.get("signaltype", "lfm").lower()
    pw = sigParams.get("pw", 1e-3)

    if signaltype == "lfm":
        f0 = fre - Bw / 2
        K = Bw / pw
        s = amp * np.sin(2 * np.pi * (f0 * t + 0.5 * K * t**2 + fd * t))

    elif signaltype == "cw":
        s = amp * np.sin(2 * np.pi * (fre + fd) * t)

    elif signaltype == "custom":
        if custom_wave is None:
            raise ValueError("自定义波形需提供 custom_wave 数组")
        # 循环取索引，保证长度匹配
        idx = np.arange(len(t)) % len(custom_wave)
        s = amp * custom_wave[idx]

    else:
        raise ValueError("未知信号类型，应为 'lfm', 'cw' 或 'custom'")

    return s

# ============ 参数设置 ============
sigParams = {
    "fs": 2e6,
    "fre": 150e3,
    "Bw": 40e3,
    "pw": 1e-3,
    "amp": 5000,
    "fd": 0,
    "signaltype": "lfm",  # 可切换 'lfm', 'cw', 'custom'
}

fs = sigParams["fs"]
pw = sigParams["pw"]
N = int(fs * pw)
t_total = np.arange(0, pw, 1/fs)

# 自定义波形示例（可选）
custom_wave = np.sin(2*np.pi*100e3*t_total) + 0.5*np.sin(2*np.pi*180e3*t_total)

# ============ 动画参数 ============
window_size = 4000  # 滚动窗口长度
step = 200          # 每帧滑动步长
current_idx = [0]   # 当前窗口起点（可变）

# ============ Matplotlib 图像窗口 ============
fig, (ax_time, ax_freq) = plt.subplots(2, 1, figsize=(12,6))

line_time, = ax_time.plot([], [], lw=1.2)
ax_time.set_ylim(-sigParams["amp"], sigParams["amp"])
ax_time.set_xlabel("时间 (ms)")
ax_time.set_ylabel("幅度 (mV)")
ax_time.set_title("时域信号（滚动显示）")
ax_time.grid(True)

line_freq, = ax_freq.plot([], [], lw=1.2)
ax_freq.set_xlim(0, fs / 2000)  # kHz
ax_freq.set_ylim(0, 1)
ax_freq.set_xlabel("频率 (kHz)")
ax_freq.set_ylabel("归一化幅度")
ax_freq.set_title("频域信号（实时FFT）")
ax_freq.grid(True)

# ============ 更新函数 ============
def update(_):
    # 使用模运算循环索引，防止越界
    idx_window = np.arange(window_size) + current_idx[0]

    if sigParams["signaltype"] == "lfm":
        t_win = t_total[idx_window % N]
        s_win = generate_sonar_signal(sigParams, t_win)
    elif sigParams["signaltype"] == "cw":
        t_win = np.arange(window_size) / fs
        s_win = sigParams["amp"] * np.sin(2*np.pi*(sigParams["fre"] + sigParams["fd"]) * (t_win + current_idx[0]/fs))
    elif sigParams["signaltype"] == "custom":
        idx = idx_window % len(custom_wave)
        s_win = sigParams["amp"] * custom_wave[idx]

    # --- 更新时域 ---
    line_time.set_data(np.arange(window_size)*1e3/fs, s_win)

    # --- 更新频域 ---
    fft_size = 4096
    spec = np.fft.fft(s_win, n=fft_size)
    freqs = np.fft.fftfreq(len(spec), 1/fs)
    pos_mask = freqs >= 0
    freqs = freqs[pos_mask]
    amp_spectrum = np.abs(spec[pos_mask])
    amp_spectrum /= np.max(amp_spectrum)
    line_freq.set_data(freqs/1e3, amp_spectrum)

    # 滚动索引
    current_idx[0] = (current_idx[0] + step) % N

    return line_time, line_freq
    
# ============ 更新函数（显示完整脉冲） ============
def update_v2(_):
    # 完整时域信号
    t_win = t_total  # 从0到pw完整时间
    if sigParams["signaltype"] == "lfm":
        s_win = generate_sonar_signal(sigParams, t_win)
    elif sigParams["signaltype"] == "cw":
        s_win = sigParams["amp"] * np.sin(2*np.pi*(sigParams["fre"] + sigParams["fd"]) * t_win)
    elif sigParams["signaltype"] == "custom":
        idx = np.arange(len(t_win)) % len(custom_wave)
        s_win = sigParams["amp"] * custom_wave[idx]

    # --- 更新时域 ---
    line_time.set_data(t_win*1e3, s_win)  # 转为毫秒显示
    ax_time.set_xlim(0, pw*1e3)           # 固定显示整个脉冲

    # --- 更新频域 ---
    fft_size = 4096
    spec = np.fft.fft(s_win, n=fft_size)
    freqs = np.fft.fftfreq(len(spec), 1/fs)
    pos_mask = freqs >= 0
    freqs = freqs[pos_mask]
    amp_spectrum = np.abs(spec[pos_mask])
    amp_spectrum /= np.max(amp_spectrum)
    line_freq.set_data(freqs/1e3, amp_spectrum)
    ax_freq.set_xlim(0, fs/2000)  # kHz

    return line_time, line_freq
def update_v3(_):
    # --- 滚动窗口索引 ---
    idx_window = np.arange(window_size) + current_idx[0]

    # --- 时域信号 ---
    t_win = (idx_window % N) / fs  # 转为真实时间，单位秒
    if sigParams["signaltype"] == "lfm":
        s_win = generate_sonar_signal(sigParams, t_win)
    elif sigParams["signaltype"] == "cw":
        s_win = sigParams["amp"] * np.sin(2*np.pi*(sigParams["fre"] + sigParams["fd"]) * t_win)
    elif sigParams["signaltype"] == "custom":
        idx = idx_window % len(custom_wave)
        s_win = sigParams["amp"] * custom_wave[idx]

    # --- 更新时域 ---
    line_time.set_data(t_win*1e3, s_win)  # 单位 ms
    ax_time.set_xlim(t_win[0]*1e3, t_win[-1]*1e3)  # X轴从当前窗口开始

    # --- 更新频域 ---
    fft_size = 4096
    spec = np.fft.fft(s_win, n=fft_size)
    freqs = np.fft.fftfreq(len(spec), 1/fs)
    pos_mask = freqs >= 0
    freqs = freqs[pos_mask]
    amp_spectrum = np.abs(spec[pos_mask])
    amp_spectrum /= np.max(amp_spectrum)
    line_freq.set_data(freqs/1e3, amp_spectrum)
    ax_freq.set_xlim(0, fs/2000)  # kHz

    # --- 滚动索引更新 ---
    current_idx[0] = (current_idx[0] + step) % N

    return line_time, line_freq

# ============ 无限滚动动画 ============
ani = FuncAnimation(fig, update, interval=30, blit=False)

plt.tight_layout()
plt.show()

