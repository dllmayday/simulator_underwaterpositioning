import numpy as np
import matplotlib.pyplot as plt

def generate_signal(amp=5000, fs=2e6, fre=150e3, Bw=40e3, pw=1e-3,
                    fd=0, signaltype='lfm'):
    """生成声纳发射信号"""
    t = np.arange(0, pw, 1/fs)
    if signaltype.lower() == 'lfm':
        f0 = fre - Bw/2
        K = Bw / pw
        s = amp * np.sin(2*np.pi*(f0*t + 0.5*K*t**2 + fd*t))
    elif signaltype.lower() == 'cw':
        s = amp * np.sin(2*np.pi*(fre + fd)*t)
    else:
        raise ValueError("signaltype 必须是 'lfm' 或 'cw'")
    return t, s

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
    plt.plot(t*1e3, s)
    plt.title(f"{title} - 时域波形")
    plt.xlabel("时间 (ms)")
    plt.ylabel("幅度 (mV)")
    plt.grid(True)

    plt.subplot(2,1,2)
    plt.plot(freqs/1e3, amplitude)
    plt.title(f"{title} - 频域波形")
    plt.xlabel("频率 (kHz)")
    plt.ylabel("归一化幅度")
    plt.grid(True)

    plt.tight_layout()
    plt.show()

# ========================
# 示例：LFM 声纳信号
# ========================
fs = 2e6          # 采样率
fre = 150e3       # 中心频率
Bw = 40e3         # 调频带宽
pw = 1e-3         # 脉宽
amp = 5000        # 幅度

t, s = generate_signal(amp=amp, fs=fs, fre=fre, Bw=Bw, pw=pw, signaltype='lfm')
plot_time_freq(t, s, fs, title="线性调频 (LFM) 声纳信号")

