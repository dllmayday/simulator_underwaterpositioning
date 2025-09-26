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
# 载荷真实运动轨迹
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
# 载荷运动轨迹显示仿真
# =====================
def run_ekf(Xtrue, t, mode="融合模式"):
    dt = t[1] - t[0]
    T = len(t)
    sigma_azi = np.deg2rad(0.8)
    sigma_ele = np.deg2rad(0.8)
    sigma_r = 0.4
    sigma_v = 0.04
    
    # 根据组合方式修改观测噪声矩阵
    if mode == "SINS+DVL":
        R = np.diag([sigma_v**2, sigma_v**2, sigma_v**2])
    elif mode == "USBL":
        R = np.diag([sigma_azi**2, sigma_ele**2, sigma_r**2])
    else:  # 融合模式
        R = np.diag([sigma_azi**2, sigma_ele**2, sigma_r**2,
                    sigma_v**2, sigma_v**2, sigma_v**2])

    q_pos, q_vel = 1e-4, 5e-4
    Q = np.block([
        [q_pos * np.eye(3), np.zeros((3,3))],
        [np.zeros((3,3)), q_vel * np.eye(3)]
    ])
    
    # 更合理的初始状态估计
    x_est = Xtrue[:,0] + np.array([1.0, -1.0, 0.5, 0.1, -0.1, 0.05])
    P = np.diag([5.0, 5.0, 2.0, 0.5, 0.5, 0.5])
    
    F = np.block([[np.eye(3), dt * np.eye(3)],
                 [np.zeros((3,3)), np.eye(3)]])
    
    Xest = np.zeros((6, T))
    
    def wrapToPi(a): 
        return (a + np.pi) % (2*np.pi) - np.pi

    for k in range(T):
        xt = Xtrue[:,k]
        
        # 生成观测值，添加数值稳定性检查
        rho = max(np.linalg.norm(xt[0:3]), 1e-6)
        rho_xy = max(np.hypot(xt[0], xt[1]), 1e-6)
        
        z_all = np.array([
            atan2(xt[1], xt[0]) + sigma_azi * np.random.randn(),
            atan2(xt[2], rho_xy) + sigma_ele * np.random.randn(),
            rho + sigma_r * np.random.randn(),
            xt[3] + sigma_v * np.random.randn(),
            xt[4] + sigma_v * np.random.randn(),
            xt[5] + sigma_v * np.random.randn()
        ])

        # 根据模式选择观测向量
        if mode == "SINS+DVL":
            z = z_all[3:6]
        elif mode == "USBL":
            z = z_all[0:3]
        else:  # 融合模式
            z = z_all

        # 预测步骤
        x_pred = F @ x_est
        P_pred = F @ P @ F.T + Q

        # 构建观测模型 H
        px_e, py_e, pz_e, vx_e, vy_e, vz_e = x_pred
        rho_xy_pred = max(np.hypot(px_e, py_e), 1e-6)
        rho_pred = max(np.linalg.norm(x_pred[0:3]), 1e-6)
        
        hx_all = np.array([
            atan2(py_e, px_e),
            atan2(pz_e, rho_xy_pred),
            rho_pred,
            vx_e,
            vy_e,
            vz_e
        ])

        if mode == "SINS+DVL":
            hx = hx_all[3:6]
            H = np.zeros((3,6))
            H[0,3] = H[1,4] = H[2,5] = 1.0
        elif mode == "USBL":
            hx = hx_all[0:3]
            H = np.zeros((3,6))
            
            # 方位角观测对状态的偏导
            denom = px_e**2 + py_e**2 + 1e-12
            H[0,0] = -py_e / denom
            H[0,1] = px_e / denom
            
            # 俯仰角观测对状态的偏导
            if rho_xy_pred > 1e-6:
                H[1,0] = -px_e * pz_e / (rho_pred**2 * rho_xy_pred)
                H[1,1] = -py_e * pz_e / (rho_pred**2 * rho_xy_pred)
                H[1,2] = rho_xy_pred / (rho_pred**2)
            
            # 距离观测对状态的偏导
            H[2,0] = px_e / rho_pred
            H[2,1] = py_e / rho_pred
            H[2,2] = pz_e / rho_pred
        else:  # 融合模式
            hx = hx_all
            H = np.zeros((6,6))
            
            # 方位角观测对状态的偏导
            denom = px_e**2 + py_e**2 + 1e-12
            H[0,0] = -py_e / denom
            H[0,1] = px_e / denom
            
            # 俯仰角观测对状态的偏导
            if rho_xy_pred > 1e-6:
                H[1,0] = -px_e * pz_e / (rho_pred**2 * rho_xy_pred)
                H[1,1] = -py_e * pz_e / (rho_pred**2 * rho_xy_pred)
                H[1,2] = rho_xy_pred / (rho_pred**2)
            
            # 距离观测对状态的偏导
            H[2,0] = px_e / rho_pred
            H[2,1] = py_e / rho_pred
            H[2,2] = pz_e / rho_pred
            
            # 速度观测对状态的偏导
            H[3,3] = H[4,4] = H[5,5] = 1.0

        # 更新步骤
        y = z - hx
        if len(y) >= 2:
            y[0] = wrapToPi(y[0])  # 方位角误差
            y[1] = wrapToPi(y[1])  # 俯仰角误差
            
        S = H @ P_pred @ H.T + R
        try:
            K = P_pred @ H.T @ np.linalg.inv(S)
        except np.linalg.LinAlgError:
            K = np.zeros((6, len(z)))
            
        x_est = x_pred + K @ y
        P = (np.eye(6) - K @ H) @ P_pred
        
        # 确保协方差矩阵保持对称正定
        P = (P + P.T) / 2
        
        Xest[:,k] = x_est

    return Xest

# =====================
# Qt 主窗口
# =====================
class EKFSimulator(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("水下组合定位仿真")
        self.setGeometry(100,100,1000,700)
        self.initUI()
    
    def initUI(self):
        main_layout = QVBoxLayout(self)

        # 参数面板
        param_layout = QHBoxLayout()
        self.motion_label = QLabel("运动模式:")
        self.motion_combo = QComboBox()
        self.motion_combo.addItems(["circle","line","spiral","random_walk"])
        
        self.mode_label = QLabel("组合方式:")
        self.mode_combo = QComboBox()
        self.mode_combo.addItems(["SINS+DVL","USBL","融合模式"])
        
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
        param_layout.addWidget(self.mode_label)
        param_layout.addWidget(self.mode_combo)
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
        mode_type   = self.mode_combo.currentText()
        sim_time = float(self.time_edit.text())
        dt = float(self.dt_edit.text())
        dynamic = self.dynamic_checkbox.isChecked()
        time_scale = float(self.scale_edit.text())

        T = int(sim_time / dt)
        t = np.arange(T) * dt
        Xtrue = generate_trajectory(motion_type, t)
        Xest  = run_ekf(Xtrue, t, mode=mode_type)
        pos_err = np.linalg.norm(Xtrue[0:3,:]-Xest[0:3,:],axis=0)
        print(f"[{motion_type} | {mode_type}] 仿真时长={sim_time}s, dt={dt}, 平均误差={pos_err.mean():.2f}, RMSE={np.sqrt(np.mean(pos_err**2)):.2f}")

        self.fig.clf()
        ax = self.fig.add_subplot(221, projection='3d')
        ax2 = self.fig.add_subplot(222)
        ax3 = self.fig.add_subplot(223)
        ax4 = self.fig.add_subplot(224)

        if dynamic:
            plt.ion()
            line_true, = ax.plot([],[],[],label="True")
            line_est,  = ax.plot([],[],[],'--',label="Estimator")
            ax.set_xlabel("X"); ax.set_ylabel("Y"); ax.set_zlabel("Z")
            ax.set_title(f"3D Motion - {motion_type} | {mode_type}")
            ax.legend()
            ax2.set_title("Error in Position"); ax2.grid(True)
            ax3.set_title("Error in X"); ax3.grid(True)
            ax4.set_title("Error in Z"); ax4.grid(True)

            for k in range(T):
                # 更新时间步
                line_true.set_data(Xtrue[0,:k+1], Xtrue[1,:k+1])
                line_true.set_3d_properties(Xtrue[2,:k+1])
                line_est.set_data(Xest[0,:k+1], Xest[1,:k+1])
                line_est.set_3d_properties(Xest[2,:k+1])

                # 动态调整坐标轴
                xs = np.concatenate([Xtrue[0,:k+1], Xest[0,:k+1]])
                ys = np.concatenate([Xtrue[1,:k+1], Xest[1,:k+1]])
                zs = np.concatenate([Xtrue[2,:k+1], Xest[2,:k+1]])
                ax.set_xlim(xs.min()-1, xs.max()+1)
                ax.set_ylim(ys.min()-1, ys.max()+1)
                ax.set_zlim(zs.min()-1, zs.max()+1)

                # 更新误差曲线
                ax2.plot(t[:k+1], pos_err[:k+1], 'r-')
                ax3.plot(t[:k+1], Xtrue[0,:k+1]-Xest[0,:k+1], 'g-')
                ax4.plot(t[:k+1], Xtrue[2,:k+1]-Xest[2,:k+1], 'b-')
                ax2.set_xlim(0, t[k]); ax2.set_ylim(0, max(pos_err[:k+1])*1.1)
                ax3.set_xlim(0, t[k]); ax3.set_ylim(min(Xtrue[0,:k+1]-Xest[0,:k+1])*1.1,
                                                   max(Xtrue[0,:k+1]-Xest[0,:k+1])*1.1)
                ax4.set_xlim(0, t[k]); ax4.set_ylim(min(Xtrue[2,:k+1]-Xest[2,:k+1])*1.1,
                                                   max(Xtrue[2,:k+1]-Xest[2,:k+1])*1.1)

                self.canvas.draw()
                QApplication.processEvents()
                time.sleep(dt / time_scale)
            plt.ioff()
        else:
            ax.plot(Xtrue[0],Xtrue[1],Xtrue[2],label="True")
            ax.plot(Xest[0],Xest[1],Xest[2],'--',label="Estimator")
            ax.set_xlabel("X"); ax.set_ylabel("Y"); ax.set_zlabel("Z")
            ax.set_title(f"3D Motion - {motion_type} | {mode_type}")
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