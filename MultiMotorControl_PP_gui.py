from PyQt5.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, 
                            QHBoxLayout, QPushButton, QLineEdit, QLabel, 
                            QTextEdit, QGroupBox, QGridLayout, QSplitter)
from PyQt5.QtCore import QTimer
from PyQt5.QtChart import QChart, QChartView, QLineSeries, QValueAxis
from PyQt5.QtCore import Qt
from PyQt5.QtGui import QPainter
import sys
import logging
from collections import deque
import time

logger = logging.getLogger(__name__)

class MotorControlGUI(QMainWindow):
    def __init__(self, motors, utils):
        super().__init__()
        self.motors = motors
        self.utils = utils
        
        # Dictionary to store input widgets for each motor
        self.motor_inputs = {}
        
        # Queue for storing time and position data
        self.time_data = deque(maxlen=100)
        self.position_data = deque(maxlen=100)
        self.velocity_data = deque(maxlen=100)
        self.start_time = time.time()
        self.is_cycle_running = False
        self.cycle_timer = QTimer()
        self.cycle_timer.timeout.connect(self.cycle_motion)
        self.current_direction = 1  # 1 for clockwise, -1 for counterclockwise
        
        # Create log display before initializing UI
        self.log_display = QTextEdit()
        self.log_display.setReadOnly(True)
        
        self.init_ui()
        
        # Timer for updating position display and chart
        self.update_timer = QTimer()
        self.update_timer.timeout.connect(self.update_data)
        self.update_timer.start(100)  # Update every 100ms
        
    def init_ui(self):
        self.setWindowTitle('Multi-Motor Control Panel')
        
        # Create main widget and layout
        main_widget = QWidget()
        self.setCentralWidget(main_widget)
        main_layout = QVBoxLayout(main_widget)

        # Create splitter for control panel and log display
        splitter = QSplitter(Qt.Vertical)
        
        # Create control panel widget
        control_panel = QWidget()
        control_layout = QHBoxLayout(control_panel)

        # Create control panels for each motor
        for motor in self.motors:
            motor_group = self.create_motor_control_group(motor)
            control_layout.addWidget(motor_group)

        # Add global control buttons
        global_control_group = self.create_global_control_group()
        control_layout.addWidget(global_control_group)

        # Add widgets to splitter
        splitter.addWidget(control_panel)
        splitter.addWidget(self.log_display)
        
        # Add splitter to main layout
        main_layout.addWidget(splitter)

        # Add Cycle Motion Control group
        cycleGroup = QGroupBox("Cycle Motion Control")
        cycleLayout = QHBoxLayout()

        self.cycleButton = QPushButton("Start Cycle Motion")
        self.cycleButton.clicked.connect(self.toggle_cycle_motion)
        
        self.cycleSpeedInput = QLineEdit()
        self.cycleSpeedInput.setPlaceholderText("Cycle Speed (rpm)")
        self.cycleSpeedInput.setText("10")  # Default speed

        cycleLayout.addWidget(QLabel("Speed:"))
        cycleLayout.addWidget(self.cycleSpeedInput)
        cycleLayout.addWidget(self.cycleButton)
        
        cycleGroup.setLayout(cycleLayout)
        
        # Add cycle control group to main layout
        main_layout.addWidget(cycleGroup)
        
        # Set window size and position
        self.setGeometry(100, 100, 1000, 600)
        
        # Set up logger
        self.setup_logger()
        
    def create_motor_control_group(self, motor):
        group = QGroupBox(f'Motor {motor.node_id}')
        layout = QGridLayout()

        # Create dictionary for this motor's inputs if it doesn't exist
        self.motor_inputs[motor.node_id] = {}

        # Position control
        pos_label = QLabel('Target Position (°):')
        pos_input = QLineEdit()
        self.motor_inputs[motor.node_id]['pos_input'] = pos_input
        go_button = QPushButton('Go to Position')
        go_button.clicked.connect(lambda: self.go_to_position(motor))

        # Velocity control
        vel_label = QLabel('Velocity (rpm):')
        vel_input = QLineEdit()
        self.motor_inputs[motor.node_id]['vel_input'] = vel_input
        vel_button = QPushButton('Set Velocity')
        vel_button.clicked.connect(lambda: self.set_velocity(motor))

        # Status display
        status_label = QLabel('Current Position:')
        status_value = QLabel('0.0°')
        self.motor_inputs[motor.node_id]['status_value'] = status_value

        # Add widgets to layout
        layout.addWidget(pos_label, 0, 0)
        layout.addWidget(pos_input, 0, 1)
        layout.addWidget(go_button, 0, 2)
        layout.addWidget(vel_label, 1, 0)
        layout.addWidget(vel_input, 1, 1)
        layout.addWidget(vel_button, 1, 2)
        layout.addWidget(status_label, 2, 0)
        layout.addWidget(status_value, 2, 1)

        group.setLayout(layout)
        return group

    def create_global_control_group(self):
        group = QGroupBox('Global Control')
        layout = QVBoxLayout()

        # Add global control buttons
        enable_all_btn = QPushButton('Enable All Motors')
        disable_all_btn = QPushButton('Disable All Motors')
        home_all_btn = QPushButton('Home All Motors')

        enable_all_btn.clicked.connect(self.enable_all_motors)
        disable_all_btn.clicked.connect(self.disable_all_motors)
        home_all_btn.clicked.connect(self.home_all_motors)

        layout.addWidget(enable_all_btn)
        layout.addWidget(disable_all_btn)
        layout.addWidget(home_all_btn)

        group.setLayout(layout)
        return group

    def update_data(self):
        """Update position and velocity data"""
        try:
            current_time = time.time() - self.start_time
            
            # Update status for each motor
            for motor in self.motors:
                current_position = motor.get_actual_position()
                if current_position is not None:
                    current_angle = self.utils.position_to_angle(current_position)
                    self.motor_inputs[motor.node_id]['status_value'].setText(f"{current_angle:.2f}°")
                    
        except Exception as e:
            logging.error(f"Error updating data: {str(e)}")
    
    def setup_logger(self):
        class QTextEditHandler(logging.Handler):
            def __init__(self, text_widget):
                super().__init__()
                self.text_widget = text_widget
                
            def emit(self, record):
                msg = self.format(record)
                self.text_widget.append(msg)
        
        handler = QTextEditHandler(self.log_display)
        formatter = logging.Formatter('%(asctime)s - %(levelname)s - %(message)s')
        handler.setFormatter(formatter)
        
        logger = logging.getLogger()
        logger.addHandler(handler)
            
    def go_to_position(self, motor):
        try:
            angle = float(self.motor_inputs[motor.node_id]['pos_input'].text())
            motor.go_to_position(angle)
        except ValueError:
            logging.error(f"Motor {motor.node_id}: Invalid position value")

    def set_velocity(self, motor):
        try:
            velocity = float(self.motor_inputs[motor.node_id]['vel_input'].text())
            motor.set_profile_velocity(velocity)
        except ValueError:
            logging.error(f"Motor {motor.node_id}: Invalid velocity value")

    def enable_all_motors(self):
        for motor in self.motors:
            motor.enable_motor()

    def disable_all_motors(self):
        for motor in self.motors:
            motor.set_controlword(0x06)  # Shutdown state

    def home_all_motors(self):
        for motor in self.motors:
            motor.go_to_position(0)

    def toggle_cycle_motion(self):
        """Toggle cycle motion state"""
        if not self.is_cycle_running:
            try:
                # Set speed for all motors
                speed = float(self.cycleSpeedInput.text())
                for motor in self.motors:
                    motor.set_profile_parameters(speed, speed, speed)
                
                self.is_cycle_running = True
                self.cycleButton.setText("Stop Cycle Motion")
                self.cycle_timer.start(100)  # Check every 100ms
                
                # Start first motion
                self.start_cycle_motion()
                
            except ValueError:
                logger.error("Please enter a valid speed value")
        else:
            self.stop_cycle_motion()

    def stop_cycle_motion(self):
        """Stop cycle motion"""
        self.is_cycle_running = False
        self.cycle_timer.stop()
        self.cycleButton.setText("Start Cycle Motion")

    def start_cycle_motion(self):
        """Start one cycle of motion"""
        target_angle = 360 if self.current_direction == 1 else -360
        for motor in self.motors:
            try:
                motor.go_to_position(target_angle)
            except Exception as e:
                logger.error(f"Motor {motor.node_id} motion failed: {str(e)}")

    def cycle_motion(self):
        """Check motion status and control cycle"""
        if not self.is_cycle_running:
            return

        # Check if all motors have reached target position
        all_reached = True
        for motor in self.motors:
            if not motor.check_target_reached():
                all_reached = False
                break

        if all_reached:
            # Change direction
            self.current_direction *= -1
            # Start new motion
            self.start_cycle_motion()

    def closeEvent(self, event):
        """窗口关闭时的处理"""
        self.stop_cycle_motion()
        event.accept()