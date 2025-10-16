import numpy as np
import matplotlib.pyplot as plt

# ========================
# 生成发射信号函数
# ========================
def generate_signal(amp=5000, fs=2e6, fre=150e3, Bw=40e3, pw=1e-3,
                    fd=0, signaltype='lfm'):
    t = np.arange(0, pw, 1/fs)
    if signaltype.lower() == 'lfm':
        f0 = fre - Bw/2
        K = Bw / pw
        s = amp * np.sin(2*np.pi*(f0*t + 0.5*K*t**2 + fd*t))
    elif signaltype.lower() == 'cw':
        s = amp * np.sin(2*np.pi*(fre + fd)*t)
    else:
        s = np.zeros_like(t)
    return t, s

# ========================
# 回波生成函数
# ========================
def simulate_echo(tx_signal, fs, R=40.0, c=1500.0, alpha=0.5, noise_level=0.02):
    tau = 2 * R / c              # 双程传播时延
    delay_samples = int(tau * fs)
    rx = np.zeros(delay_samples + len(tx_signal))
    rx[delay_samples:delay_samples+len(tx_signal)] = alpha * tx_signal
    # 添加噪声
    noise = noise_level * np.max(tx_signal) * np.random.randn(len(rx))
    rx += noise
    t = np.arange(0, len(rx)) / fs
    return t, rx, tau

# ========================
# 仿真主流程
# ========================
def sonar_simulation():
    fs = 2e6
    fre = 150e3
    Bw = 40e3
    pw = 1e-3
    amp = 5000
    R = 40.0
    c = 1500.0

    # 1. 发射信号
    t_tx, s_tx = generate_signal(amp, fs, fre, Bw, pw, signaltype='lfm')

    # 2. 回波信号（含时延+噪声）
    t_rx, s_rx, tau = simulate_echo(s_tx, fs, R, c, alpha=0.6, noise_level=0.05)

    # 3. 绘制结果
    plt.figure(figsize=(12,6))
    plt.subplot(2,1,1)
    plt.plot(t_tx*1e3, s_tx)
    plt.title("发射信号 (Tx) - LFM声纳脉冲")
    plt.xlabel("时间 (ms)")
    plt.ylabel("幅度 (mV)")
    plt.grid(True)

    plt.subplot(2,1,2)
    plt.plot(t_rx*1e3, s_rx)
    plt.title(f"接收信号 (Rx) - 含传播时延 τ = {tau*1e3:.2f} ms 与噪声")
    plt.xlabel("时间 (ms)")
    plt.ylabel("幅度 (mV)")
    plt.grid(True)

    plt.tight_layout()
    plt.show()

# ========================
# 执行仿真
# ========================
sonar_simulation()

