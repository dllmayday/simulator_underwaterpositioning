import sys
import math
import matplotlib.pyplot as plt
from matplotlib.figure import Figure
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas, NavigationToolbar2QT
from mpl_toolkits.mplot3d import Axes3D
from PyQt5.QtWidgets import QApplication, QMainWindow, QVBoxLayout, QHBoxLayout, QWidget, QLineEdit, QPushButton, QComboBox, QLabel, QSizePolicy

# 定义不同的运动轨迹模型
def straight_line_motion(position, yaw, speed, delta_t):
    """匀速直线运动"""
    x, y, z = position
    v_x = speed * math.cos(math.radians(yaw))
    v_y = speed * math.sin(math.radians(yaw))
    return (x + v_x * delta_t, y + v_y * delta_t, z)

def circular_motion(position, radius, speed, delta_t):
    """匀速圆周运动"""
    x, y, z = position
    omega = speed / radius  # 角速度
    theta = omega * delta_t
    new_x = x + radius * math.cos(theta)
    new_y = y + radius * math.sin(theta)
    return new_x, new_y, z

def serpentine_motion(position, speed, amplitude, period, delta_t):
    """蛇形机动"""
    x, y, z = position
    v_x = speed
    new_x = x + v_x * delta_t
    new_y = y + amplitude * math.sin(2 * math.pi * delta_t / period)
    return new_x, new_y, z

def spiral_motion(position, radius, speed, height_rate, delta_t):
    """螺旋运动"""
    x, y, z = position
    omega = speed / radius  # 角速度
    theta = omega * delta_t
    new_x = x + radius * math.cos(theta)
    new_y = y + radius * math.sin(theta)
    new_z = z + height_rate * delta_t
    return new_x, new_y, new_z

# 主窗口类
class TrajectoryWindow(QMainWindow):
    def __init__(self):
        super().__init__()

        # 初始化界面
        self.setWindowTitle('Trajectory Simulation')
        self.setGeometry(100, 100, 800, 600)

        # 初始化参数
        self.position = (105.3, 75.6, -85.2)  # 初始位置
        self.yaw = 15.2  # 初始偏航角
        self.speed = 2.0  # 速度
        self.delta_t = 1.0  # 时间步长
        self.radius = 10.0  # 圆周半径
        self.amplitude = 5.0  # 蛇形机动的振幅
        self.period = 10.0  # 蛇形机动的周期
        self.height_rate = 0.2  # 螺旋运动的高度变化率
        self.total_time = 100  # 默认总时间

        # 设置布局
        main_layout = QVBoxLayout()

        # 创建设置面板并放置在一行中
        settings_layout = QHBoxLayout()

        # 输入框和选择框放在一行
        self.position_input = QLineEdit(self)
        self.position_input.setPlaceholderText("Enter position (x, y, z) - e.g. 105.3, 75.6, -85.2")
        self.position_input.setText("105.3, 75.6, -85.2")
        self.position_input.setMinimumWidth(150)
        self.position_input.setMaximumWidth(150)
        settings_layout.addWidget(QLabel("起点:"))
        settings_layout.addWidget(self.position_input)

        self.yaw_input = QLineEdit(self)
        self.yaw_input.setPlaceholderText("Enter yaw angle (degrees) - e.g. 15.2")
        self.yaw_input.setText("5.2")
        self.yaw_input.setMinimumWidth(40)
        self.yaw_input.setMaximumWidth(50)
        settings_layout.addWidget(QLabel("偏航角(degrees):"))
        settings_layout.addWidget(self.yaw_input)

        self.speed_input = QLineEdit(self)
        self.speed_input.setPlaceholderText("Enter speed (m/s) - e.g. 2.0")
        self.speed_input.setText("2.0")
        self.speed_input.setMinimumWidth(40)
        self.speed_input.setMaximumWidth(50)
        settings_layout.addWidget(QLabel("速度(m/s):"))
        settings_layout.addWidget(self.speed_input)

        # 运动模型选择
        self.model_label = QLabel("选择运动模式:", self)
        settings_layout.addWidget(self.model_label)

        self.model_select = QComboBox(self)
        self.model_select.addItem("Straight Line Motion")
        self.model_select.addItem("Circular Motion")
        self.model_select.addItem("Serpentine Motion")
        self.model_select.addItem("Spiral Motion")
        settings_layout.addWidget(self.model_select)

        # 模拟时间输入
        self.time_input = QLineEdit(self)
        self.time_input.setPlaceholderText("Enter total simulation time (seconds) - e.g. 100")
        self.time_input.setText("100")
        self.time_input.setMinimumWidth(40)
        self.time_input.setMaximumWidth(50)
        settings_layout.addWidget(QLabel("模拟时间(s)"))
        settings_layout.addWidget(self.time_input)

        # 开始按钮
        self.start_button = QPushButton('开始仿真', self)
        self.start_button.clicked.connect(self.start_simulation)
        self.start_button.setMaximumHeight(30)
        self.start_button.setSizePolicy(QSizePolicy.Fixed, QSizePolicy.Fixed)
        settings_layout.addWidget(self.start_button)
        settings_layout.addStretch()
        

        # 创建matplotlib画布
        self.fig = Figure(figsize=(10, 6))
        self.canvas = FigureCanvas(self.fig)
        # 添加matplotlib工具栏
        self.toolbar = NavigationToolbar2QT(self.canvas, self)
        
        # 设置工具栏和画布不做垂直方向的拉伸
        self.toolbar.setSizePolicy(QSizePolicy.Fixed, QSizePolicy.Fixed)  # 工具栏垂直方向固定
        self.canvas.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)  # 图表区域允许拉伸

        # 主布局：工具栏 + 设置面板 + 图表
        main_layout.addLayout(settings_layout)  # 设置面板
        main_layout.addWidget(self.toolbar)  # 工具栏
        main_layout.addWidget(self.canvas)  # 添加画布显示

        # 设置窗口中央控件
        container = QWidget(self)
        container.setLayout(main_layout)
        self.setCentralWidget(container)

    def start_simulation(self):
        # 获取用户输入
        try:
            position_input = self.position_input.text().split(',')
            self.position = (float(position_input[0]), float(position_input[1]), float(position_input[2]))
            self.yaw = float(self.yaw_input.text())
            self.speed = float(self.speed_input.text())
            self.total_time = float(self.time_input.text())
        except ValueError:
            print("Invalid input!")
            return

        # 获取运动模型
        model = self.model_select.currentText()

        # 模拟轨迹
        trajectory = []
        for t in range(1, int(self.total_time) + 1):
            if model == "Straight Line Motion":
                new_position = straight_line_motion(self.position, self.yaw, self.speed, self.delta_t * t)
            elif model == "Circular Motion":
                new_position = circular_motion(self.position, self.radius, self.speed, self.delta_t * t)
            elif model == "Serpentine Motion":
                new_position = serpentine_motion(self.position, self.speed, self.amplitude, self.period, self.delta_t * t)
            elif model == "Spiral Motion":
                new_position = spiral_motion(self.position, self.radius, self.speed, self.height_rate, self.delta_t * t)

            trajectory.append(new_position)

        # 解析轨迹数据
        x_vals, y_vals, z_vals = zip(*trajectory)

        # 使用matplotlib显示3D图表
        self.plot_trajectory(x_vals, y_vals, z_vals)

    def plot_trajectory(self, x_vals, y_vals, z_vals):
        # 创建3D图表
        self.fig.clf()
        ax = self.fig.add_subplot(111, projection='3d')

        # 绘制轨迹
        ax.plot(x_vals, y_vals, z_vals, label='Trajectory', color='b')

        # 设置标签
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')

        # 设置标题
        ax.set_title('3D Trajectory Simulation')

        # 绘制到画布上
        self.canvas.figure = self.fig
        self.canvas.draw()

# 运行应用程序
def main():
    app = QApplication(sys.argv)
    window = TrajectoryWindow()
    window.show()
    sys.exit(app.exec_())

if __name__ == '__main__':
    main()
