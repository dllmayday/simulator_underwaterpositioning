import numpy as np
import matplotlib.pyplot as plt
import argparse
import time
from math import atan2

np.random.seed(1)

# =====================
# 生成目标运动轨迹
# =====================
def generate_trajectory(motion_type, t):
    if motion_type == "circle":
        R0, omega, vz_true = 60.0, 0.04, -0.03
        px = R0 * np.cos(omega * t)
        py = R0 * np.sin(omega * t)
        pz = -0.4 * t + 10.0
        vx = -R0 * omega * np.sin(omega * t)
        vy =  R0 * omega * np.cos(omega * t)
        vz = vz_true * np.ones_like(t)
    elif motion_type == "line":
        vx0, vy0, vz0 = 0.3, 0.1, -0.02
        px = vx0 * t
        py = vy0 * t
        pz = vz0 * t
        vx = vx0 * np.ones_like(t)
        vy = vy0 * np.ones_like(t)
        vz = vz0 * np.ones_like(t)
    elif motion_type == "spiral":
        R0, omega, vz_true = 30.0, 0.1, -0.05
        px = R0 * np.cos(omega * t)
        py = R0 * np.sin(omega * t)
        pz = vz_true * t
        vx = -R0 * omega * np.sin(omega * t)
        vy =  R0 * omega * np.cos(omega * t)
        vz = vz_true * np.ones_like(t)
    elif motion_type == "random_walk":
        steps = np.random.randn(len(t), 3) * 0.2
        pos = np.cumsum(steps, axis=0)
        px, py, pz = pos[:,0], pos[:,1], pos[:,2]
        vx, vy, vz = np.gradient(px,t), np.gradient(py,t), np.gradient(pz,t)
    else:
        raise ValueError("未知运动模式: " + motion_type)
    return np.vstack([px, py, pz, vx, vy, vz])

# =====================
# EKF 仿真
# =====================
def run_ekf(Xtrue, t):
    dt = t[1] - t[0]
    T = len(t)
    # 噪声
    sigma_azi = np.deg2rad(0.8)
    sigma_ele = np.deg2rad(0.8)
    sigma_r = 0.4
    sigma_v = 0.04
    R = np.diag([sigma_azi**2, sigma_ele**2, sigma_r**2,
                 sigma_v**2, sigma_v**2, sigma_v**2])
    q_pos, q_vel = 1e-4, 5e-4
    Q = np.block([
        [q_pos * np.eye(3), np.zeros((3,3))],
        [np.zeros((3,3)), q_vel * np.eye(3)]
    ])
    # 初始估计
    x_est = Xtrue[:,0] + np.array([3,-3,1.5,0,0,0])
    P = np.diag([10.0,10.0,5.0,1.0,1.0,1.0])
    F = np.block([[np.eye(3), dt * np.eye(3)],
                  [np.zeros((3,3)), np.eye(3)]])
    Xest = np.zeros((6, T))

    def wrapToPi(a): return (a + np.pi) % (2*np.pi) - np.pi

    for k in range(T):
        xt = Xtrue[:,k]
        rel = xt[0:3]
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
        # H矩阵
        H = np.zeros((6,6))
        denom = px_e**2 + py_e**2 + 1e-12
        H[0,0], H[0,1] = -py_e/denom, px_e/denom
        if rho_xy > 1e-6:
            H[1,0] = -px_e*pz_e/(rho**2 * rho_xy)
            H[1,1] = -py_e*pz_e/(rho**2 * rho_xy)
            H[1,2] = rho_xy/(rho**2)
        H[2,0:3] = [px_e/rho, py_e/rho, pz_e/rho]
        H[3,3] = H[4,4] = H[5,5] = 1.0
        # 更新
        S = H @ P_pred @ H.T + R
        K = P_pred @ H.T @ np.linalg.inv(S)
        y = z - hx
        y[0], y[1] = wrapToPi(y[0]), wrapToPi(y[1])
        x_est = x_pred + K @ y
        P = (np.eye(6) - K @ H) @ P_pred
        Xest[:,k] = x_est
    return Xest

# =====================
# 主程序
# =====================
def simulate(motion_type="circle", sim_time=120.0, dt=0.5):
    T = int(sim_time / dt)
    t = np.arange(T) * dt
    Xtrue = generate_trajectory(motion_type, t)
    Xest  = run_ekf(Xtrue, t)
    pos_err = np.linalg.norm(Xtrue[0:3,:] - Xest[0:3,:], axis=0)
    print(f"[{motion_type}] 仿真时长={sim_time:.1f}s, dt={dt}, 平均误差={pos_err.mean():.2f}, RMSE={np.sqrt(np.mean(pos_err**2)):.2f}")
    # 绘图
    fig = plt.figure(figsize=(10,8))
    ax = fig.add_subplot(221, projection='3d')
    ax.plot(Xtrue[0],Xtrue[1],Xtrue[2],label="True")
    ax.plot(Xest[0],Xest[1],Xest[2],'--',label="EKF")
    ax.set_title(f"3D Motion - {motion_type}")
    ax.set_xlabel("X")   # X轴
    ax.set_ylabel("Y")   # Y轴
    ax.set_zlabel("Z")   # Z轴
    ax.legend()
    ax2 = fig.add_subplot(222)
    ax2.plot(t,pos_err); ax2.set_title("Error in Position"); ax2.grid(True)
    ax3 = fig.add_subplot(223)
    ax3.plot(t,Xtrue[0]-Xest[0]); ax3.set_title("Error in X"); ax3.grid(True)
    ax4 = fig.add_subplot(224)
    ax4.plot(t,Xtrue[2]-Xest[2]); ax4.set_title("Error in Z"); ax4.grid(True)
    plt.tight_layout(); plt.show()

def update_3d_limits(ax, Xtrue, Xest, k):
    # 获取当前显示范围
    xs = np.concatenate([Xtrue[0,:k+1], Xest[0,:k+1]])
    ys = np.concatenate([Xtrue[1,:k+1], Xest[1,:k+1]])
    zs = np.concatenate([Xtrue[2,:k+1], Xest[2,:k+1]])
    
    ax.set_xlim(xs.min()-1, xs.max()+1)
    ax.set_ylim(ys.min()-1, ys.max()+1)
    ax.set_zlim(zs.min()-1, zs.max()+1)
#version with dynamic plotting 
def simulate_dynamic(motion_type="circle", sim_time=120.0, dt=0.5, time_scale=10.0):
    T = int(sim_time / dt)
    t = np.arange(T) * dt
    Xtrue = generate_trajectory(motion_type, t)
    Xest  = run_ekf(Xtrue, t)

    pos_err = np.linalg.norm(Xtrue[0:3,:] - Xest[0:3,:], axis=0)
    print(f"[{motion_type}] 仿真时长={sim_time:.1f}s, dt={dt}, 平均误差={pos_err.mean():.2f}, RMSE={np.sqrt(np.mean(pos_err**2)):.2f}")

    # 设置动态刷新图表
    plt.ion()
    fig = plt.figure(figsize=(10,8))
    ax = fig.add_subplot(221, projection='3d')
    ax.set_title(f"3D Motion - {motion_type}")
    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_zlabel("Z")
    line_true, = ax.plot([], [], [], label="True")
    line_est,  = ax.plot([], [], [], '--', label="EKF")
    ax.legend()

    ax2 = fig.add_subplot(222)
    ax2.set_title("Error in Position")
    ax2.grid(True)
    line_err, = ax2.plot([], [], 'r-')

    ax3 = fig.add_subplot(223)
    ax3.set_title("Error in X")
    ax3.grid(True)
    line_xerr, = ax3.plot([], [], 'g-')

    ax4 = fig.add_subplot(224)
    ax4.set_title("Error in Z")
    ax4.grid(True)
    line_zerr, = ax4.plot([], [], 'b-')

    plt.tight_layout()
    plt.show()

    for k in range(T):
        # 更新3D轨迹
        line_true.set_data(Xtrue[0,:k+1], Xtrue[1,:k+1])
        line_true.set_3d_properties(Xtrue[2,:k+1])
        line_est.set_data(Xest[0,:k+1], Xest[1,:k+1])
        line_est.set_3d_properties(Xest[2,:k+1])
        update_3d_limits(ax, Xtrue, Xest, k)

        # 更新误差曲线
        line_err.set_data(t[:k+1], pos_err[:k+1])
        line_xerr.set_data(t[:k+1], Xtrue[0,:k+1]-Xest[0,:k+1])
        line_zerr.set_data(t[:k+1], Xtrue[2,:k+1]-Xest[2,:k+1])

        # 设置坐标轴范围自动调整
        ax2.set_xlim(0, t[k])
        ax2.set_ylim(0, max(pos_err[:k+1])*1.1)
        ax3.set_xlim(0, t[k])
        ax3.set_ylim(min(Xtrue[0,:k+1]-Xest[0,:k+1])*1.1, max(Xtrue[0,:k+1]-Xest[0,:k+1])*1.1)
        ax4.set_xlim(0, t[k])
        ax4.set_ylim(min(Xtrue[2,:k+1]-Xest[2,:k+1])*1.1, max(Xtrue[2,:k+1]-Xest[2,:k+1])*1.1)

        plt.pause(dt / time_scale)  # 根据时间系数加速或减慢仿真

    plt.ioff()
    plt.show()
# =====================
# CLI 接口
# =====================
if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="USBL + DVL EKF 仿真")
    parser.add_argument("--motion", type=str, default="circle",
                        choices=["circle","line","spiral","random_walk"],
                        help="选择目标运动模式")
    parser.add_argument("--time", type=float, default=120.0,
                        help="仿真时长 (秒)")
    parser.add_argument("--dt", type=float, default=0.5,
                        help="时间间隔 dt (秒)")
    args = parser.parse_args()
    simulate(args.motion, args.time, args.dt)

