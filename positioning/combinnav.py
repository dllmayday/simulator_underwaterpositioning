import sys
import numpy as np
import matplotlib
matplotlib.use("Qt5Agg")
import matplotlib.pyplot as plt
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.backends.backend_qt5agg import NavigationToolbar2QT as NavigationToolbar
from matplotlib.figure import Figure
from PyQt5.QtCore import Qt
from PyQt5.QtGui import QDoubleValidator
from PyQt5.QtWidgets import (QApplication, QWidget, QVBoxLayout, QHBoxLayout,
                             QLabel, QComboBox, QLineEdit, QCheckBox, QPushButton, 
                             QTextEdit, QSplitter, QSizePolicy, QAction)
from math import atan2
import time

np.random.seed(1)

# =====================
# 轨迹生成函数
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
def run_ekf(Xtrue, t, mode="融合模式", sigma_azi=0.5, sigma_ele=0.5, sigma_r=0.4, sigma_v=0.04,
            q_pos=1e-3, q_vel=1e-3, usbl_station_pos=np.array([0.0,0.0,0.0])):
    dt = t[1] - t[0]
    T = len(t)
    sigma_azi = np.deg2rad(0.8)
    sigma_ele = np.deg2rad(0.8)
    sigma_r = 0.4
    sigma_v = 0.04
    
    if mode == "SINS+DVL":
        R = np.diag([sigma_v**2, sigma_v**2, sigma_v**2])
    elif mode == "USBL":
        R = np.diag([sigma_azi**2, sigma_ele**2, sigma_r**2])
    else:  # 融合模式
        R = np.diag([sigma_azi**2, sigma_ele**2, sigma_r**2,
                    sigma_v**2, sigma_v**2, sigma_v**2])

    q_pos, q_vel = 1e-4, 5e-4
    Q = np.block([
        [q_pos*np.eye(3), np.zeros((3,3))],
        [np.zeros((3,3)), q_vel*np.eye(3)]
    ])
    
    x_est = Xtrue[:,0] + np.array([0.1, -0.1, 0.05, 0.1, -0.1, 0.05])
    P = np.diag([5.0, 5.0, 2.0, 0.5, 0.5, 0.5])
    F = np.block([[np.eye(3), dt*np.eye(3)],
                  [np.zeros((3,3)), np.eye(3)]])
    
    Xest = np.zeros((6,T))
    def wrapToPi(a): return (a+np.pi)%(2*np.pi)-np.pi

    for k in range(T):
        xt = Xtrue[:,k]
        rel = xt[0:3] - usbl_station_pos  # 相对基准站
        rho = max(np.linalg.norm(rel),1e-6)
        rho_xy = max(np.hypot(rel[0],rel[1]),1e-6)
        z_all = np.array([
            atan2(rel[1],rel[0]) + sigma_azi*np.random.randn(),
            atan2(rel[2],rho_xy) + sigma_ele*np.random.randn(),
            rho + sigma_r*np.random.randn(),
            xt[3]+sigma_v*np.random.randn(),
            xt[4]+sigma_v*np.random.randn(),
            xt[5]+sigma_v*np.random.randn()
        ])
        if mode=="SINS+DVL":
            z = z_all[3:6]
        elif mode=="USBL":
            z = z_all[0:3]
        else:
            z = z_all

        # 预测
        x_pred = F @ x_est
        P_pred = F @ P @ F.T + Q
        px_e, py_e, pz_e, vx_e, vy_e, vz_e = x_pred
        rel_pred = x_pred[0:3] - usbl_station_pos
        rho_xy_pred = max(np.hypot(rel_pred[0],rel_pred[1]),1e-6)
        rho_pred = max(np.linalg.norm(rel_pred),1e-6)
        hx_all = np.array([
            atan2(rel_pred[1],rel_pred[0]),
            atan2(rel_pred[2],rho_xy_pred),
            rho_pred,
            vx_e, vy_e, vz_e
        ])
        if mode=="SINS+DVL":
            hx = hx_all[3:6]
            H = np.zeros((3,6)); H[0,3]=H[1,4]=H[2,5]=1.0
        elif mode=="USBL":
            hx = hx_all[0:3]
            H = np.zeros((3,6))
            denom = rel_pred[0]**2 + rel_pred[1]**2 + 1e-12
            H[0,0] = -rel_pred[1]/denom
            H[0,1] = rel_pred[0]/denom
            if rho_xy_pred>1e-6:
                H[1,0]=-rel_pred[0]*rel_pred[2]/(rho_pred**2*rho_xy_pred)
                H[1,1]=-rel_pred[1]*rel_pred[2]/(rho_pred**2*rho_xy_pred)
                H[1,2]=rho_xy_pred/(rho_pred**2)
            H[2,0:3]=rel_pred/rho_pred
        else:
            hx = hx_all
            H = np.zeros((6,6))
            denom = rel_pred[0]**2 + rel_pred[1]**2 + 1e-12
            H[0,0]=-rel_pred[1]/denom
            H[0,1]=rel_pred[0]/denom
            if rho_xy_pred>1e-6:
                H[1,0]=-rel_pred[0]*rel_pred[2]/(rho_pred**2*rho_xy_pred)
                H[1,1]=-rel_pred[1]*rel_pred[2]/(rho_pred**2*rho_xy_pred)
                H[1,2]=rho_xy_pred/(rho_pred**2)
            H[2,0:3]=rel_pred/rho_pred
            H[3,3]=H[4,4]=H[5,5]=1.0

        y = z - hx
        if len(y)>=2:
            y[0] = wrapToPi(y[0])
            y[1] = wrapToPi(y[1])
        S = H @ P_pred @ H.T + R
        try:
            K = P_pred @ H.T @ np.linalg.inv(S)
        except np.linalg.LinAlgError:
            K = np.zeros((6,len(z)))
        x_est = x_pred + K @ y
        P = (np.eye(6) - K @ H) @ P_pred
        P = (P + P.T)/2
        Xest[:,k] = x_est
    return Xest
# =====================
# Qt 主窗口
# =====================
class EKFSimulator(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("水下组合定位仿真")
        self.setGeometry(50, 50, 1300, 700)
        self.simulation_running = False
        self.current_simulation_thread = None
        # 定义多个基准站位置 [x, y, z]
        self.usbl_stations = np.array([
            [0.0, 0.0, 0.0],
            [50.0, 0.0, -5.0],
            [-30.0, 40.0, -10.0]
        ])
        
        self.current_expanded = None  # 记录当前放大的子图
        self.original_positions = {}  # 保存原始子图位置
        
        self.initUI()

    def initUI(self):
        main_layout = QVBoxLayout(self)

        # 左侧参数+图表
        left_layout = QVBoxLayout()

        # 顶部布局（参数和开始按钮）
        top_layout = QHBoxLayout()
        
        param_layout = QVBoxLayout()
        row1 = QHBoxLayout()
        self.motion_label = QLabel("平台运动模式:")
        self.motion_combo = QComboBox()
        self.motion_combo.addItems(["line", "spiral", "circle", "random_walk"])
        self.mode_label = QLabel("定位方式:")
        self.mode_combo = QComboBox()
        self.mode_combo.addItems(["SINS+DVL", "USBL", "融合模式"])
        self.time_label = QLabel("仿真时长(s):")
        self.time_edit = QLineEdit("500.0")
        time_validator = QDoubleValidator(0.1, 100.0, 5)
        self.time_edit.setValidator(time_validator)
        self.dt_label = QLabel("时间间隔(s):")
        self.dt_edit = QLineEdit("0.5")
        dt_validator = QDoubleValidator(0.1, 100.0, 5)
        self.dt_edit.setValidator(dt_validator)
        self.dynamic_checkbox = QCheckBox("动态显示")
        self.scale_label = QLabel("动画倍速(x):")
        self.scale_edit = QLineEdit("10.0")
        scale_validator = QDoubleValidator(10, 1000.0, 5)
        self.scale_edit.setValidator(scale_validator)
        
        for w in [self.motion_label, self.motion_combo, self.mode_label, self.mode_combo,
                 self.time_label, self.time_edit, self.dt_label, self.dt_edit,
                 self.dynamic_checkbox, self.scale_label, self.scale_edit]:
            row1.addWidget(w)
        row1.addStretch()
        
        # 第二行：观测噪声参数
        row2 = QHBoxLayout()
        noise_validator = QDoubleValidator(0.0, 1000.0, 5)
        self.sigma_azi_edit = QLineEdit("0.5")
        self.sigma_ele_edit = QLineEdit("0.5")
        self.sigma_r_edit = QLineEdit("0.4")
        self.sigma_v_edit = QLineEdit("0.04")
        self.sigma_azi_edit.setValidator(noise_validator)
        self.sigma_ele_edit.setValidator(noise_validator)
        self.sigma_r_edit.setValidator(noise_validator)
        self.sigma_v_edit.setValidator(noise_validator)
        row2.addWidget(QLabel("观测噪声不确定性:"))
        row2.addWidget(QLabel("方位角噪声(°):"))
        row2.addWidget(self.sigma_azi_edit)
        row2.addWidget(QLabel("俯仰角噪声(°):"))
        row2.addWidget(self.sigma_ele_edit)
        row2.addWidget(QLabel("距离标准差(m):"))
        row2.addWidget(self.sigma_r_edit)
        row2.addWidget(QLabel("速度标准差(m/s):"))
        row2.addWidget(self.sigma_v_edit)
        row2.addStretch()
        
        # 第三行：过程噪声参数
        row3 = QHBoxLayout()
        self.q_pos_edit = QLineEdit("0.001")
        self.q_vel_edit = QLineEdit("0.001")
        self.q_pos_edit.setValidator(noise_validator)
        self.q_vel_edit.setValidator(noise_validator)
        row3.addWidget(QLabel("过程噪声不确定性:"))
        row3.addWidget(QLabel("位置协方差(m²):"))
        row3.addWidget(self.q_pos_edit)
        row3.addWidget(QLabel("速度协方差((m/s)²):"))
        row3.addWidget(self.q_vel_edit)
        row3.addStretch()

        param_layout.addLayout(row1)
        param_layout.addLayout(row2)
        param_layout.addLayout(row3)
        top_layout.addLayout(param_layout)
        
        # 右侧开始按钮
        self.start_btn = QPushButton("开始仿真")
        self.start_btn.setMaximumHeight(80)
        self.start_btn.setSizePolicy(QSizePolicy.Preferred, QSizePolicy.Fixed)
        start_btn_container = QVBoxLayout()
        start_btn_container.addWidget(self.start_btn)
        top_layout.addLayout(start_btn_container)
        left_layout.addLayout(top_layout)

        # 默认隐藏时间系数相关控件
        self.scale_label.setVisible(False)
        self.scale_edit.setVisible(False)
        # 连接复选框状态改变的信号
        self.dynamic_checkbox.stateChanged.connect(self.toggle_scale_visibility)

        # 初始化图表
        self.init_plots()
        
        # 添加工具栏
        self.toolbar = NavigationToolbar(self.canvas, self)
        left_layout.addWidget(self.toolbar)
        left_layout.addWidget(self.canvas)

        # 右侧日志
        self.splitter = QSplitter(Qt.Horizontal)
        left_widget = QWidget()
        left_widget.setLayout(left_layout)
        self.log_text = QTextEdit()
        self.log_text.setReadOnly(True)
        self.log_text.setStyleSheet("background-color:#f4f4f4; font-family: monospace;")
        self.splitter.addWidget(left_widget)
        self.splitter.addWidget(self.log_text)
        self.splitter.setSizes([1000, 300])
        main_layout.addWidget(self.splitter)

        self.start_btn.clicked.connect(self.start_simulation)

    def init_plots(self):
        """初始化图表和子图"""
        self.fig = Figure(figsize=(10, 6))
        self.canvas = FigureCanvas(self.fig)
        
        # 创建子图
        self.ax = self.fig.add_subplot(221, projection='3d')
        self.ax2 = self.fig.add_subplot(222)
        self.ax3 = self.fig.add_subplot(223)
        self.ax4 = self.fig.add_subplot(224)
        
        # 保存原始位置
        self.original_positions = {
            'ax': self.ax.get_position(),
            'ax2': self.ax2.get_position(),
            'ax3': self.ax3.get_position(),
            'ax4': self.ax4.get_position()
        }
        
        # 连接鼠标事件
        self.fig.canvas.mpl_connect('button_press_event', self.on_plot_click)
        self.current_expanded = None

    def on_plot_click(self, event):
        """处理图表点击事件"""
        if event.dblclick:  # 双击事件
            if event.inaxes in [self.ax, self.ax2, self.ax3, self.ax4]:
                if self.current_expanded == event.inaxes:
                    self.restore_plot_layout()
                else:
                    self.expand_plot(event.inaxes)
                self.canvas.draw()

    def expand_plot(self, ax):
        """放大指定子图"""
        # 隐藏其他子图
        for a in [self.ax, self.ax2, self.ax3, self.ax4]:
            if a != ax:
                a.set_visible(False)
        
        # 调整被点击子图的位置和大小
        ax.set_position([0.1, 0.1, 0.8, 0.8])
        ax.set_visible(True)
        self.current_expanded = ax
        
        # 更新图表标题显示提示
        # title = ax.get_title()
        # if not title:
        #     title = "3D Motion" if ax == self.ax else "Error in Position" if ax == self.ax2 else "Error in X" if ax == self.ax3 else "Error in Z"
        # ax.set_title(f"{title} (最大化中，双击恢复)")

    def restore_plot_layout(self):
        """恢复原始布局"""
        for ax in [self.ax, self.ax2, self.ax3, self.ax4]:
            ax.set_visible(True)
        
        # 恢复原始位置
        self.ax.set_position(self.original_positions['ax'])
        self.ax2.set_position(self.original_positions['ax2'])
        self.ax3.set_position(self.original_positions['ax3'])
        self.ax4.set_position(self.original_positions['ax4'])

        # 恢复原始标题
        titles = {
            'ax': "3D Motion Trajectory",
            'ax2': "Position Error",
            'ax3': "X Error",
            'ax4': "Z Error"
        }
        self.ax.set_title(titles['ax'])
        self.ax2.set_title(titles['ax2'])
        self.ax3.set_title(titles['ax3'])
        self.ax4.set_title(titles['ax4'])
        
        self.current_expanded = None

    def toggle_scale_visibility(self, state):
        """根据动态显示复选框的状态显示或隐藏时间系数控件"""
        is_visible = state == Qt.Checked
        self.scale_label.setVisible(is_visible)
        self.scale_edit.setVisible(is_visible)

    def start_simulation(self):
        """开始仿真"""
        # 如果已有仿真在运行，则停止
        if self.simulation_running:
            self.stop_simulation()
            time.sleep(0.1)  # 等待一小段时间确保停止
        try:
            # 重置停止标志
            self.simulation_running = True

            # 检查图表状态
            if not hasattr(self, 'ax') or self.ax is None:
                self.init_plots()
            
            # 恢复原始布局
            self.clear_plots()
            
            motion_type = self.motion_combo.currentText()
            mode_type = self.mode_combo.currentText()
            sim_time = float(self.time_edit.text())
            dt = float(self.dt_edit.text())
            dynamic = self.dynamic_checkbox.isChecked()
            time_scale = float(self.scale_edit.text()) if self.dynamic_checkbox.isChecked() else 1.0
            
            T = int(sim_time/dt)
            t = np.arange(T)*dt
            
            # 读取噪声参数
            sigma_azi = float(self.sigma_azi_edit.text())
            sigma_ele = float(self.sigma_ele_edit.text())
            sigma_r = float(self.sigma_r_edit.text())
            sigma_v = float(self.sigma_v_edit.text())
            q_pos = float(self.q_pos_edit.text())
            q_vel = float(self.q_vel_edit.text())

            Xtrue = generate_trajectory(motion_type, t)
            # 使用第一个基准站作为 EKF USBL 观测
            Xest = run_ekf(Xtrue, t, mode_type, sigma_azi, sigma_ele, sigma_r, sigma_v, 
                          q_pos, q_vel, self.usbl_stations[0])
            pos_err = np.linalg.norm(Xtrue[0:3,:]-Xest[0:3,:], axis=0)

            # 清除并重新初始化图表
            self.fig.clf()
            self.ax = self.fig.add_subplot(221, projection='3d')
            self.ax2 = self.fig.add_subplot(222)
            self.ax3 = self.fig.add_subplot(223)
            self.ax4 = self.fig.add_subplot(224)
            

            # 绘制基准站
            self.ax.scatter(self.usbl_stations[0][0], self.usbl_stations[0][1], 
                           self.usbl_stations[0][2], c='black', marker='*', 
                           s=150, label="Station")

            if dynamic:
                plt.ion()
                line_true, = self.ax.plot([], [], [], label="True")
                line_est, = self.ax.plot([], [], [], '--', label="Estimator")
                self.ax.set_xlabel("X")
                self.ax.set_ylabel("Y")
                self.ax.set_zlabel("Z")
                self.ax.set_title(f"3D Motion - {motion_type}")
                self.ax.legend()
                
                self.ax2.set_title("Error in Position")
                self.ax2.grid(True)
                self.ax3.set_title("Error in X")
                self.ax3.grid(True)
                self.ax4.set_title("Error in Z")
                self.ax4.grid(True)


                for k in range(T):
                    if not self.simulation_running:
                        break

                    line_true.set_data(Xtrue[0,:k+1], Xtrue[1,:k+1])
                    line_true.set_3d_properties(Xtrue[2,:k+1])
                    line_est.set_data(Xest[0,:k+1], Xest[1,:k+1])
                    line_est.set_3d_properties(Xest[2,:k+1])
                    
                    # 自动调整坐标轴范围
                    xs = np.concatenate([Xtrue[0,:k+1], Xest[0,:k+1], [self.usbl_stations[0][0]]])
                    ys = np.concatenate([Xtrue[1,:k+1], Xest[1,:k+1], [self.usbl_stations[0][1]]])
                    zs = np.concatenate([Xtrue[2,:k+1], Xest[2,:k+1], [self.usbl_stations[0][2]]])
                    
                    self.ax.set_xlim(xs.min()-1, xs.max()+1)
                    self.ax.set_ylim(ys.min()-1, ys.max()+1)
                    self.ax.set_zlim(zs.min()-1, zs.max()+1)
                    
                    self.ax2.plot(t[:k+1], pos_err[:k+1], 'r-')
                    self.ax3.plot(t[:k+1], Xtrue[0,:k+1]-Xest[0,:k+1], 'g-')
                    self.ax4.plot(t[:k+1], Xtrue[2,:k+1]-Xest[2,:k+1], 'b-')
                    
                    self.ax2.set_xlim(0, t[k])
                    self.ax2.set_ylim(0, max(pos_err[:k+1])*1.1)
                    
                    self.ax3.set_xlim(0, t[k])
                    self.ax3.set_ylim(min(Xtrue[0,:k+1]-Xest[0,:k+1])*1.1,
                                     max(Xtrue[0,:k+1]-Xest[0,:k+1])*1.1)
                    
                    self.ax4.set_xlim(0, t[k])
                    self.ax4.set_ylim(min(Xtrue[2,:k+1]-Xest[2,:k+1])*1.1,
                                     max(Xtrue[2,:k+1]-Xest[2,:k+1])*1.1)
                    
                    self.canvas.draw()
                    QApplication.processEvents()
                    time.sleep(dt/time_scale)
                plt.ioff()
            else:
                self.ax.plot(Xtrue[0], Xtrue[1], Xtrue[2], label="True")
                self.ax.plot(Xest[0], Xest[1], Xest[2], '--', label="Estimator")
                self.ax.set_xlabel("X")
                self.ax.set_ylabel("Y")
                self.ax.set_zlabel("Z")
                self.ax.set_title(f"3D Motion - {motion_type} ")
                self.ax.legend()
                
                self.ax2.plot(t, pos_err)
                self.ax2.set_title("Error in Position")
                self.ax2.grid(True)
                
                self.ax3.plot(t, Xtrue[0]-Xest[0])
                self.ax3.set_title("Error in X")
                self.ax3.grid(True)
                
                self.ax4.plot(t, Xtrue[2]-Xest[2])
                self.ax4.set_title("Error in Z")
                self.ax4.grid(True)
                
                self.canvas.draw()
            if self.simulation_running:
                line = f"[{motion_type}] 仿真时长={sim_time:.1f}s, 定位方式={mode_type}, dt={dt}, 平均误差={pos_err.mean():.2f}, RMSE={np.sqrt(np.mean(pos_err**2)):.2f}"
                self.log_text.append(line)

        except Exception as e:
            print(f"仿真出错: {str(e)}")
            self.log_text.append(f"错误: {str(e)}")
        finally:
            self.simulation_running = False
    def stop_simulation(self):
        """停止当前仿真"""
        if self.simulation_running:
            self.simulation_running = False
    def clear_plots(self):
        """清理图表"""
        self.fig.clf()
        self.ax = self.fig.add_subplot(221, projection='3d')
        self.ax2 = self.fig.add_subplot(222)
        self.ax3 = self.fig.add_subplot(223)
        self.ax4 = self.fig.add_subplot(224)
        
        # 保存原始位置
        self.original_positions = {
            'ax': self.ax.get_position(),
            'ax2': self.ax2.get_position(),
            'ax3': self.ax3.get_position(),
            'ax4': self.ax4.get_position()
        }
        
        self.canvas.draw()
# =====================
# 运行
# =====================
if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = EKFSimulator()
    window.show()
    sys.exit(app.exec_())