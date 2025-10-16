import numpy as np
import matplotlib.pyplot as plt
from math import atan2

np.random.seed(1)

# =====================
# 参数设置
# =====================
dt = 0.5
T = 240
t = np.arange(T) * dt

R0 = 60.0         # 半径
omega = 0.04      # 角速度
vz_true = -0.03   # Z方向速度

# 目标真实轨迹
px = R0 * np.cos(omega * t)
py = R0 * np.sin(omega * t)
pz = -0.4 * t + 10.0
vx = -R0 * omega * np.sin(omega * t)
vy = R0 * omega * np.cos(omega * t)
vz = vz_true * np.ones_like(t)
Xtrue = np.vstack([px, py, pz, vx, vy, vz])

# 观测噪声
sigma_azi = np.deg2rad(0.8)
sigma_ele = np.deg2rad(0.8)
sigma_r = 0.4
sigma_v = 0.04
R = np.diag([sigma_azi**2, sigma_ele**2, sigma_r**2,
             sigma_v**2, sigma_v**2, sigma_v**2])

# 过程噪声
q_pos, q_vel = 1e-4, 5e-4
Q = np.block([
    [q_pos * np.eye(3), np.zeros((3,3))],
    [np.zeros((3,3)), q_vel * np.eye(3)]
])

# 初始条件
x_est = np.array([px[0]+3.0, py[0]-3.0, pz[0]+1.5, 0.0, 0.0, 0.0])
P = np.diag([10.0,10.0,5.0,1.0,1.0,1.0])
F = np.block([[np.eye(3), dt * np.eye(3)],
              [np.zeros((3,3)), np.eye(3)]])

Xest = np.zeros((6, T))

def wrapToPi(a):
    return (a + np.pi) % (2*np.pi) - np.pi

# =====================
# EKF 迭代
# =====================
for k in range(T):
    xt = Xtrue[:, k]
    rel = xt[0:3]  # USBL 位于原点
    r = np.linalg.norm(rel)
    az = atan2(rel[1], rel[0])
    el = atan2(rel[2], np.hypot(rel[0], rel[1]))

    z = np.array([
        az + sigma_azi*np.random.randn(),
        el + sigma_ele*np.random.randn(),
        r  + sigma_r*np.random.randn(),
        xt[3] + sigma_v*np.random.randn(),
        xt[4] + sigma_v*np.random.randn(),
        xt[5] + sigma_v*np.random.randn()
    ])

    # 预测
    x_pred = F.dot(x_est)
    P_pred = F.dot(P).dot(F.T) + Q

    px_e, py_e, pz_e, vx_e, vy_e, vz_e = x_pred
    rho_xy = np.hypot(px_e, py_e)
    rho = np.linalg.norm(x_pred[0:3])

    hx = np.array([
        atan2(py_e, px_e),
        atan2(pz_e, rho_xy),
        rho,
        vx_e, vy_e, vz_e
    ])

    # 观测矩阵 H
    H = np.zeros((6,6))
    denom = px_e**2 + py_e**2 + 1e-12
    H[0,0] = -py_e / denom
    H[0,1] =  px_e / denom
    if rho_xy < 1e-6:
        H[1,0:3] = 0.0
    else:
        H[1,0] = -px_e * pz_e / (rho**2 * rho_xy)
        H[1,1] = -py_e * pz_e / (rho**2 * rho_xy)
        H[1,2] = rho_xy / (rho**2)
    H[2,0] = px_e / rho
    H[2,1] = py_e / rho
    H[2,2] = pz_e / rho
    H[3,3] = H[4,4] = H[5,5] = 1.0

    # EKF 更新
    S = H.dot(P_pred).dot(H.T) + R
    K = P_pred.dot(H.T).dot(np.linalg.inv(S))
    y = z - hx
    y[0] = wrapToPi(y[0])
    y[1] = wrapToPi(y[1])
    x_est = x_pred + K.dot(y)
    P = (np.eye(6) - K.dot(H)).dot(P_pred)
    Xest[:, k] = x_est

# =====================
# 误差计算
# =====================
pos_err = np.sqrt(np.sum((Xtrue[0:3,:] - Xest[0:3,:])**2, axis=0))
print("平均位置误差:", pos_err.mean())
print("RMSE:", np.sqrt(np.mean(pos_err**2)))

# =====================
# 绘图
# =====================
fig = plt.figure(figsize=(10,8))

# 3D轨迹
ax = fig.add_subplot(221, projection='3d')
ax.plot(Xtrue[0,:], Xtrue[1,:], Xtrue[2,:], label="True", linewidth=1.4)
ax.plot(Xest[0,:], Xest[1,:], Xest[2,:], '--', label="EKF", linewidth=1.2)
ax.set_xlabel("X (m)"); ax.set_ylabel("Y (m)"); ax.set_zlabel("Z (m)")
ax.set_title("3D Trajectory")
ax.legend()

# 误差曲线
ax2 = fig.add_subplot(222)
ax2.plot(t, pos_err, linewidth=1.2)
ax2.set_xlabel("Time (s)"); ax2.set_ylabel("Error (m)")
ax2.set_title("Position Error")
ax2.grid(True)

# X误差
ax3 = fig.add_subplot(223)
ax3.plot(t, Xtrue[0,:] - Xest[0,:])
ax3.set_title("Error in X")
ax3.set_xlabel("Time (s)"); ax3.set_ylabel("Error (m)")
ax3.grid(True)

# Z误差
ax4 = fig.add_subplot(224)
ax4.plot(t, Xtrue[2,:] - Xest[2,:])
ax4.set_title("Error in Z")
ax4.set_xlabel("Time (s)"); ax4.set_ylabel("Error (m)")
ax4.grid(True)

plt.tight_layout()
plt.show()

