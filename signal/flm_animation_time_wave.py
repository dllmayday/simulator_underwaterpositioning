import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

# 参数
fs = 2e6
fre = 150e3
Bw = 40e3
pw = 1e-3
amp = 5000
K = Bw / pw

t = np.arange(0, pw, 1/fs)
f0 = fre - Bw/2

# 初始化绘图
fig, ax = plt.subplots()
line, = ax.plot([], [], lw=1.5)
ax.set_xlim(0, pw*1e3)
ax.set_ylim(-amp, amp)
ax.set_xlabel("时间 (ms)")
ax.set_ylabel("幅度 (mV)")
ax.set_title("LFM 信号动态显示")

def init():
    line.set_data([], [])
    return line,

def update(frame):
    # 模拟信号“逐渐生成”
    idx = int(frame / len(t) * len(t))
    current_t = t[:idx]
    current_s = amp * np.sin(2*np.pi*(f0*current_t + 0.5*K*current_t**2))
    line.set_data(current_t * 1e3, current_s)
    return line,

ani = FuncAnimation(fig, update, frames=len(t), init_func=init,
                    blit=True, interval=1)
plt.show()

