import sys
import numpy as np
import matplotlib.pyplot as plt
from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QLabel, QLineEdit, QPushButton, QSizePolicy, QMessageBox
)
from PyQt5.QtGui import QPixmap
from PyQt5.QtCore import Qt, QTimer

# PID Controller class
class PIDController:
    def __init__(self, Kp, Ki, Kd):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.previous_error = 0
        self.integral = 0
    
    def update(self, setpoint, current_value, dt):
        error = setpoint - current_value
        self.integral += error * dt
        derivative = (error - self.previous_error) / dt
        output = self.Kp * error + self.Ki * self.integral + self.Kd * derivative
        self.previous_error = error
        return output

# Simulation widget
class SimulationWidget(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.initUI()
        self.pid = PIDController(1.0, 0.1, 0.2)  # Initial PID gains
        self.setpoint = 1.0  # Initial setpoint
        self.current_value = 0.0
        self.time = 0.0
        self.dt = 0.1
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.updateSimulation)
        self.values = []
        self.is_running = False

    def initUI(self):
        layout = QVBoxLayout()

        # PID parameters input
        pid_params_layout = QHBoxLayout()
        pid_params_layout.addWidget(QLabel("Kp:"))
        self.kp_edit = QLineEdit("1.0")
        pid_params_layout.addWidget(self.kp_edit)
        pid_params_layout.addWidget(QLabel("Ki:"))
        self.ki_edit = QLineEdit("0.1")
        pid_params_layout.addWidget(self.ki_edit)
        pid_params_layout.addWidget(QLabel("Kd:"))
        self.kd_edit = QLineEdit("0.2")
        pid_params_layout.addWidget(self.kd_edit)
        layout.addLayout(pid_params_layout)

        # Setpoint input
        setpoint_layout = QHBoxLayout()
        setpoint_layout.addWidget(QLabel("Setpoint:"))
        self.setpoint_edit = QLineEdit("255.0")
        setpoint_layout.addWidget(self.setpoint_edit)
        layout.addLayout(setpoint_layout)

        # Buttons
        btn_layout = QHBoxLayout()
        self.start_btn = QPushButton("Start")
        self.start_btn.clicked.connect(self.startSimulation)
        btn_layout.addWidget(self.start_btn)
        self.stop_btn = QPushButton("Stop")
        self.stop_btn.clicked.connect(self.stopSimulation)
        btn_layout.addWidget(self.stop_btn)
        layout.addLayout(btn_layout)

        # Plot area
        self.plot_label = QLabel(self)
        self.plot_label.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        layout.addWidget(self.plot_label)

        self.setLayout(layout)

    def startSimulation(self):
        if not self.is_running:
            self.is_running = True
            self.pid.Kp = float(self.kp_edit.text())
            self.pid.Ki = float(self.ki_edit.text())
            self.pid.Kd = float(self.kd_edit.text())
            self.setpoint = float(self.setpoint_edit.text())
            self.current_value = 0.0
            self.time = 0.0
            self.values = []
            self.timer.start(int(self.dt * 1000))

    def stopSimulation(self):
        self.is_running = False
        self.timer.stop()

    def updateSimulation(self):
        output = self.pid.update(self.setpoint, self.current_value, self.dt)
        self.current_value += output * self.dt
        self.time += self.dt
        self.values.append(self.current_value)
        self.updatePlot()

    def updatePlot(self):
        plt.figure(figsize=(8, 4))
        plt.plot(np.arange(len(self.values)) * self.dt, self.values, label='Response')
        plt.axhline(self.setpoint, color='r', linestyle='--', label='Setpoint')
        plt.title('PID Response')
        plt.xlabel('Time')
        plt.ylabel('Value')
        plt.legend()
        plt.grid(True)

        # Convert plot to image
        plt.tight_layout()
        plt.savefig('plot.png')

        # Display plot image in QLabel
        pixmap = QPixmap('plot.png')
        self.plot_label.setPixmap(pixmap)

# Main Window
class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.initUI()

    def initUI(self):
        self.setGeometry(100, 100, 800, 600)
        self.setWindowTitle('PID Controller Simulation')

        sim_widget = SimulationWidget(self)
        self.setCentralWidget(sim_widget)

        self.show()

if __name__ == '__main__':
    app = QApplication(sys.argv)
    mainWindow = MainWindow()
    sys.exit(app.exec_())
