import sys
import numpy as np
import matplotlib
matplotlib.use("Qt5Agg")
import matplotlib.pyplot as plt
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure
from PyQt5.QtWidgets import (QApplication, QWidget, QVBoxLayout, QHBoxLayout,
                             QLabel, QComboBox, QLineEdit, QCheckBox, QPushButton)
from math import atan2
import time

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
        x_pred = F.dot(x_est)
        P_pred = F.dot(P).dot(F.T) + Q
        px_e, py_e, pz_e, vx_e, vy_e, vz_e = x_pred
        rho_xy = np.hypot(px_e, py_e)
        rho = np.linalg.norm(x_pred[0:3])
        hx = np.array([atan2(py_e, px_e), atan2(pz_e, rho_xy), rho, vx_e, vy_e, vz_e])
        H = np.zeros((6,6))
        denom = px_e**2 + py_e**2 + 1e-12
        H[0,0], H[0,1] = -py_e/denom, px_e/denom
        if rho_xy > 1e-6:
            H[1,0] = -px_e*pz_e/(rho**2 * rho_xy)
            H[1,1] = -py_e*pz_e/(rho**2 * rho_xy)
            H[1,2] = rho_xy/(rho**2)
        H[2,0:3] = [px_e/rho, py_e/rho, pz_e/rho]
        H[3,3] = H[4,4] = H[5,5] = 1.0
        S = H @ P_pred @ H.T + R
        K = P_pred @ H.T @ np.linalg.inv(S)
        y = z - hx
        y[0], y[1] = wrapToPi(y[0]), wrapToPi(y[1])
        x_est = x_pred + K @ y
        P = (np.eye(6) - K @ H) @ P_pred
        Xest[:,k] = x_est
    return Xest

# =====================
# Qt 主窗口
# =====================
class EKFSimulator(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("超短基线定位仿真")
        self.setGeometry(100,100,1000,700)
        self.initUI()
    
    def initUI(self):
        main_layout = QVBoxLayout(self)

        # 参数面板
        param_layout = QHBoxLayout()
        self.motion_label = QLabel("运动模式:")
        self.motion_combo = QComboBox()
        self.motion_combo.addItems(["circle","line","spiral","random_walk"])
        self.time_label = QLabel("仿真时长(s):")
        self.time_edit = QLineEdit("500.0")
        self.dt_label = QLabel("时间间隔(s):")
        self.dt_edit = QLineEdit("0.5")
        self.dynamic_checkbox = QCheckBox("动态显示")
        self.scale_label = QLabel("时间系数:")
        self.scale_edit = QLineEdit("10.0")
        self.start_btn = QPushButton("开始仿真")
        param_layout.addWidget(self.motion_label)
        param_layout.addWidget(self.motion_combo)
        param_layout.addWidget(self.time_label)
        param_layout.addWidget(self.time_edit)
        param_layout.addWidget(self.dt_label)
        param_layout.addWidget(self.dt_edit)
        param_layout.addWidget(self.dynamic_checkbox)
        param_layout.addWidget(self.scale_label)
        param_layout.addWidget(self.scale_edit)
        param_layout.addWidget(self.start_btn)
        main_layout.addLayout(param_layout)

        # Matplotlib Figure
        self.fig = Figure(figsize=(10,6))
        self.canvas = FigureCanvas(self.fig)
        main_layout.addWidget(self.canvas)
        self.start_btn.clicked.connect(self.start_simulation)

    def start_simulation(self):
        motion_type = self.motion_combo.currentText()
        sim_time = float(self.time_edit.text())
        dt = float(self.dt_edit.text())
        dynamic = self.dynamic_checkbox.isChecked()
        time_scale = float(self.scale_edit.text())

        T = int(sim_time / dt)
        t = np.arange(T) * dt
        Xtrue = generate_trajectory(motion_type, t)
        Xest  = run_ekf(Xtrue, t)
        pos_err = np.linalg.norm(Xtrue[0:3,:]-Xest[0:3,:],axis=0)
        print(f"[{motion_type}] 仿真时长={sim_time}s, dt={dt}, 平均误差={pos_err.mean():.2f}, RMSE={np.sqrt(np.mean(pos_err**2)):.2f}")

        self.fig.clf()
        ax = self.fig.add_subplot(221, projection='3d')
        ax2 = self.fig.add_subplot(222)
        ax3 = self.fig.add_subplot(223)
        ax4 = self.fig.add_subplot(224)

        if dynamic:
            plt.ion()
            line_true, = ax.plot([],[],[],label="True")
            line_est,  = ax.plot([],[],[],'--',label="Simulator")
            ax.set_xlabel("X"); ax.set_ylabel("Y"); ax.set_zlabel("Z")
            ax.set_title("3D Motion - "+motion_type)
            ax.legend()
            ax2.set_title("Error in Position"); ax2.grid(True)
            ax3.set_title("Error in X"); ax3.grid(True)
            ax4.set_title("Error in Z"); ax4.grid(True)

            for k in range(T):
                line_true.set_data(Xtrue[0,:k+1], Xtrue[1,:k+1])
                line_true.set_3d_properties(Xtrue[2,:k+1])
                line_est.set_data(Xest[0,:k+1], Xest[1,:k+1])
                line_est.set_3d_properties(Xest[2,:k+1])

                xs = np.concatenate([Xtrue[0,:k+1], Xest[0,:k+1]])
                ys = np.concatenate([Xtrue[1,:k+1], Xest[1,:k+1]])
                zs = np.concatenate([Xtrue[2,:k+1], Xest[2,:k+1]])
                ax.set_xlim(xs.min()-1,xs.max()+1)
                ax.set_ylim(ys.min()-1,ys.max()+1)
                ax.set_zlim(zs.min()-1,zs.max()+1)

                ax2.plot(t[:k+1], pos_err[:k+1], 'r-')
                ax3.plot(t[:k+1], Xtrue[0,:k+1]-Xest[0,:k+1], 'g-')
                ax4.plot(t[:k+1], Xtrue[2,:k+1]-Xest[2,:k+1], 'b-')
                ax2.set_xlim(0,t[k]); ax2.set_ylim(0,max(pos_err[:k+1])*1.1)
                ax3.set_xlim(0,t[k]); ax3.set_ylim(min(Xtrue[0,:k+1]-Xest[0,:k+1])*1.1,max(Xtrue[0,:k+1]-Xest[0,:k+1])*1.1)
                ax4.set_xlim(0,t[k]); ax4.set_ylim(min(Xtrue[2,:k+1]-Xest[2,:k+1])*1.1,max(Xtrue[2,:k+1]-Xest[2,:k+1])*1.1)

                self.canvas.draw()
                QApplication.processEvents()
                time.sleep(dt/time_scale)
            plt.ioff()
        else:
            ax.plot(Xtrue[0],Xtrue[1],Xtrue[2],label="True")
            ax.plot(Xest[0],Xest[1],Xest[2],'--',label="Simulator")
            ax.set_xlabel("X"); ax.set_ylabel("Y"); ax.set_zlabel("Z")
            ax.set_title("3D Motion - "+motion_type)
            ax.legend()
            ax2.plot(t,pos_err); ax2.set_title("Error in Position"); ax2.grid(True)
            ax3.plot(t,Xtrue[0]-Xest[0]); ax3.set_title("Error in X"); ax3.grid(True)
            ax4.plot(t,Xtrue[2]-Xest[2]); ax4.set_title("Error in Z"); ax4.grid(True)
            self.canvas.draw()

# =====================
# 运行
# =====================
if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = EKFSimulator()
    window.show()
    sys.exit(app.exec_())
