from PyQt5.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, 
                            QHBoxLayout, QPushButton, QLineEdit, QLabel, 
                            QTextEdit, QGroupBox)
from PyQt5.QtCore import QTimer
from PyQt5.QtChart import QChart, QChartView, QLineSeries, QValueAxis
from PyQt5.QtCore import Qt
from PyQt5.QtGui import QPainter
import sys
import logging
from collections import deque
import time

class MotorControlGUI(QMainWindow):
    def __init__(self, motor, utils):
        super().__init__()
        self.motor = motor
        self.utils = utils
        
        # Queue for storing time and position data
        self.time_data = deque(maxlen=100)
        self.position_data = deque(maxlen=100)
        self.velocity_data = deque(maxlen=100)
        self.start_time = time.time()
        
        self.initUI()
        
        # Timer for updating position display and chart
        self.update_timer = QTimer()
        self.update_timer.timeout.connect(self.update_data)
        self.update_timer.start(100)  # Update every 100ms
        
    def initUI(self):
        self.setWindowTitle('Motor Control Panel')
        self.setGeometry(100, 100, 1200, 800)
        
        # Create central widget and main layout
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        main_layout = QVBoxLayout(central_widget)
        
        # Upper section: Control panel
        control_panel = QHBoxLayout()
        
        # Left control group
        left_control = QVBoxLayout()
        
        # Position control section
        position_group = QGroupBox("Position Control")
        position_layout = QVBoxLayout()
        
        # Current position display
        self.current_pos_label = QLabel("Current Position: 0.00°")
        position_layout.addWidget(self.current_pos_label)
        
        # Target position input
        target_layout = QHBoxLayout()
        self.angle_input = QLineEdit()
        self.angle_input.setPlaceholderText("Enter target angle")
        self.go_button = QPushButton("Move to Position")
        self.go_button.clicked.connect(self.go_to_position)
        target_layout.addWidget(self.angle_input)
        target_layout.addWidget(self.go_button)
        position_layout.addLayout(target_layout)
        
        position_group.setLayout(position_layout)
        left_control.addWidget(position_group)
        
        # Motion parameters control section
        params_group = QGroupBox("Motion Parameters")
        params_layout = QVBoxLayout()
        
        # Velocity setting
        velocity_layout = QHBoxLayout()
        velocity_layout.addWidget(QLabel("Velocity (°/s):"))
        self.velocity_input = QLineEdit("5")
        velocity_layout.addWidget(self.velocity_input)
        params_layout.addLayout(velocity_layout)
        
        # Acceleration setting
        accel_layout = QHBoxLayout()
        accel_layout.addWidget(QLabel("Acceleration (°/s²):"))
        self.acceleration_input = QLineEdit("5")
        accel_layout.addWidget(self.acceleration_input)
        params_layout.addLayout(accel_layout)
        
        # Deceleration setting
        decel_layout = QHBoxLayout()
        decel_layout.addWidget(QLabel("Deceleration (°/s²):"))
        self.deceleration_input = QLineEdit("5")
        decel_layout.addWidget(self.deceleration_input)
        params_layout.addLayout(decel_layout)
        
        # Apply parameters button
        self.apply_params_button = QPushButton("Apply Parameters")
        self.apply_params_button.clicked.connect(self.apply_motion_parameters)
        params_layout.addWidget(self.apply_params_button)
        
        params_group.setLayout(params_layout)
        left_control.addWidget(params_group)
        
        control_panel.addLayout(left_control)
        
        # Create charts
        self.create_charts()
        control_panel.addWidget(self.chart_view)
        
        main_layout.addLayout(control_panel)
        
        # Lower section: Log display
        log_group = QGroupBox("Operation Log")
        log_layout = QVBoxLayout()
        self.log_display = QTextEdit()
        self.log_display.setReadOnly(True)
        log_layout.addWidget(self.log_display)
        log_group.setLayout(log_layout)
        main_layout.addWidget(log_group)
        
        # Set up logger
        self.setup_logger()
        
    def create_charts(self):
        # Create chart
        self.chart = QChart()
        self.chart.setTitle("Motor Motion Curves")
        
        # Create position curve
        self.position_series = QLineSeries()
        self.position_series.setName("Position (°)")
        
        # Create velocity curve
        self.velocity_series = QLineSeries()
        self.velocity_series.setName("Velocity (°/s)")
        
        # Add curves to chart
        self.chart.addSeries(self.position_series)
        self.chart.addSeries(self.velocity_series)
        
        # Create axes
        self.axis_x = QValueAxis()
        self.axis_x.setTitleText("Time (s)")
        self.axis_x.setRange(0, 10)
        
        self.axis_y_position = QValueAxis()
        self.axis_y_position.setTitleText("Position (°)")
        self.axis_y_position.setRange(-360, 360)
        
        self.axis_y_velocity = QValueAxis()
        self.axis_y_velocity.setTitleText("Velocity (°/s)")
        self.axis_y_velocity.setRange(-50, 50)
        
        # Add axes to chart
        self.chart.addAxis(self.axis_x, Qt.AlignBottom)
        self.chart.addAxis(self.axis_y_position, Qt.AlignLeft)
        self.chart.addAxis(self.axis_y_velocity, Qt.AlignRight)
        
        self.position_series.attachAxis(self.axis_x)
        self.position_series.attachAxis(self.axis_y_position)
        self.velocity_series.attachAxis(self.axis_x)
        self.velocity_series.attachAxis(self.axis_y_velocity)
        
        # Create chart view
        self.chart_view = QChartView(self.chart)
        self.chart_view.setRenderHint(QPainter.Antialiasing)
        
    def update_data(self):
        """Update position and velocity data"""
        try:
            current_time = time.time() - self.start_time
            
            # Get current position
            current_position = self.motor.get_actual_position()
            if current_position is not None:
                current_angle = self.utils.position_to_angle(current_position)
                self.current_pos_label.setText(f"Current Position: {current_angle:.2f}°")
                
                # Update position data
                self.time_data.append(current_time)
                self.position_data.append(current_angle)
                
                # Calculate velocity
                if len(self.position_data) >= 2 and len(self.time_data) >= 2:
                    dt = self.time_data[-1] - self.time_data[-2]
                    if dt > 0:
                        velocity = (self.position_data[-1] - self.position_data[-2]) / dt
                        self.velocity_data.append(velocity)
                    else:
                        self.velocity_data.append(0)
                else:
                    self.velocity_data.append(0)
                
                # Update chart data
                self.position_series.clear()
                self.velocity_series.clear()
                for i in range(len(self.time_data)):
                    self.position_series.append(self.time_data[i], self.position_data[i])
                    self.velocity_series.append(self.time_data[i], self.velocity_data[i])
                
                # Dynamically adjust axes ranges
                if self.position_data:
                    # Get min and max values for position and velocity
                    min_pos = min(self.position_data)
                    max_pos = max(self.position_data)
                    min_vel = min(self.velocity_data)
                    max_vel = max(self.velocity_data)
                    
                    # Add 10% padding to ranges
                    pos_padding = (max_pos - min_pos) * 0.1
                    vel_padding = (max_vel - min_vel) * 0.1 if max_vel != min_vel else 1.0
                    
                    # Update position axis range
                    self.axis_y_position.setRange(
                        min_pos - pos_padding,
                        max_pos + pos_padding
                    )
                    
                    # Update velocity axis range
                    self.axis_y_velocity.setRange(
                        min_vel - vel_padding,
                        max_vel + vel_padding
                    )
                
                # Adjust X axis range to show recent data
                if current_time > 10:
                    # Show last 10 seconds of data
                    self.axis_x.setRange(current_time - 10, current_time)
                    
                    # Optional: Adjust time window based on data density
                    time_window = 10
                    if len(self.time_data) > 1000:  # If too many points
                        # Increase time window to reduce point density
                        time_window = 20
                    self.axis_x.setRange(current_time - time_window, current_time)
                else:
                    # For the first 10 seconds, show all data from 0
                    self.axis_x.setRange(0, max(10, current_time + 1))
    
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
            
    def apply_motion_parameters(self):
        """Apply motion parameters"""
        try:
            velocity = float(self.velocity_input.text())
            acceleration = float(self.acceleration_input.text())
            deceleration = float(self.deceleration_input.text())
            
            self.motor.set_profile_parameters(velocity, acceleration, deceleration)
            logging.info(f"Motion parameters updated - Velocity: {velocity}°/s, Acceleration: {acceleration}°/s², Deceleration: {deceleration}°/s²")
        except ValueError:
            logging.error("Please enter valid numbers")
        except Exception as e:
            logging.error(f"Error setting motion parameters: {str(e)}")
            
    def go_to_position(self):
        """Move to specified position"""
        try:
            angle = float(self.angle_input.text())
            self.motor.go_to_position(angle)
        except ValueError:
            logging.error("Please enter a valid number")
        except Exception as e:
            logging.error(f"Error moving to position: {str(e)}")