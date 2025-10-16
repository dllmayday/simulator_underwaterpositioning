import numpy as np
import matplotlib.pyplot as plt

# ============ 信号生成函数 ============

def generate_sonar_signal(sigParams, custom_wave=None):
    """
    根据参数生成声纳信号
    支持: LFM, CW, Custom
    """
    fs = sigParams.get("fs", 2e6)
    fre = sigParams.get("fre", 150e3)
    Bw = sigParams.get("Bw", 40e3)
    pw = sigParams.get("pw", 1e-3)
    amp = sigParams.get("amp", 5000)
    fd = sigParams.get("fd", 0)
    signaltype = sigParams.get("signaltype", "lfm").lower()

    t = np.arange(0, pw, 1/fs)

    if signaltype == "lfm":
        f0 = fre - Bw / 2
        K = Bw / pw
        s = amp * np.sin(2 * np.pi * (f0 * t + 0.5 * K * t**2 + fd * t))

    elif signaltype == "cw":
        s = amp * np.sin(2 * np.pi * (fre + fd) * t)

    elif signaltype == "custom":
        if custom_wave is None:
            raise ValueError("自定义波形需提供 custom_wave 数组")
        if len(custom_wave) != len(t):
            raise ValueError("custom_wave 长度必须与采样点数相同")
        s = amp * custom_wave

    else:
        raise ValueError("未知信号类型，应为 'lfm', 'cw' 或 'custom'")

    return t, s


# ============ 波形绘制函数 ============

def plot_time_freq(t, s, fs, title="声纳信号"):
    """绘制时域与频域波形"""
    N = len(s)
    freqs = np.fft.fftfreq(N, 1/fs)
    spectrum = np.fft.fft(s)
    amplitude = np.abs(spectrum) / N

    # 只取正频部分
    pos_mask = freqs >= 0
    freqs = freqs[pos_mask]
    amplitude = amplitude[pos_mask]

    plt.figure(figsize=(12,6))

    plt.subplot(2,1,1)
    plt.plot(t * 1e3, s)
    plt.title(f"{title} - 时域波形")
    plt.xlabel("时间 (ms)")
    plt.ylabel("幅度 (mV)")
    plt.grid(True)

    plt.subplot(2,1,2)
    plt.plot(freqs / 1e3, amplitude)
    plt.title(f"{title} - 频域波形")
    plt.xlabel("频率 (kHz)")
    plt.ylabel("归一化幅度")
    plt.grid(True)

    plt.tight_layout()
    plt.show()


# ============ 示例测试 ============

sigParams = {
    "fs": 2e6,
    "fre": 150e3,
    "Bw": 40e3,
    "pw": 1e-3,
    "amp": 5000,
    "fd": 0,
}

# 示例 1：LFM 信号
sigParams["signaltype"] = "lfm"
t, s = generate_sonar_signal(sigParams)
plot_time_freq(t, s, sigParams["fs"], title="LFM 线性调频信号")

# 示例 2：CW 连续波信号
sigParams["signaltype"] = "cw"
t, s = generate_sonar_signal(sigParams)
plot_time_freq(t, s, sigParams["fs"], title="CW 连续波信号")

# 示例 3：Custom 自定义波形（例如双频混合信号）
sigParams["signaltype"] = "custom"
t = np.arange(0, sigParams["pw"], 1/sigParams["fs"])
custom_wave = np.sin(2*np.pi*100e3*t) + 0.5*np.sin(2*np.pi*180e3*t)
t, s = generate_sonar_signal(sigParams, custom_wave=custom_wave)
plot_time_freq(t, s, sigParams["fs"], title="自定义双频信号")

