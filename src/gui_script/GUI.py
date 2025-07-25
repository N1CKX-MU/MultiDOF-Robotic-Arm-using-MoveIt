import sys
import subprocess
import threading
import time
import os
from PyQt5.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout,
                             QHBoxLayout, QComboBox, QPushButton, QSlider,
                             QLabel, QGroupBox, QTextEdit, QSplitter)
from PyQt5.QtCore import Qt, QProcess, QTimer, pyqtSignal, QThread, QProcessEnvironment
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

class ROSThread(QThread):
    joint_state_signal = pyqtSignal(object)
    
    def __init__(self):
        super().__init__()
        self.node = None
        
    def run(self):
        try:
            if not rclpy.ok():
                rclpy.init()
            self.node = JointStateMonitor(self.joint_state_callback)
            rclpy.spin(self.node)
        except Exception as e:
            print(f"ROS thread error: {e}")
    
    def joint_state_callback(self, msg):
        self.joint_state_signal.emit(msg)
    
    def shutdown(self):
        if self.node:
            self.node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

class JointStateMonitor(Node):
    def __init__(self, callback):
        super().__init__('joint_state_monitor')
        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',
            callback,
            10)

class RobotControlGUI(QMainWindow):
    
    def __init__(self):
        super().__init__()
        self.launch_process = None
        self.joint_gui_process = None
        self.current_dof = 0
        self.ros_thread = None
        self.init_ui()
        
        # Start ROS thread after a short delay
        QTimer.singleShot(1000, self.init_ros)

    def init_ros(self):
        """Initialize ROS 2 in a separate thread"""
        try:
            self.ros_thread = ROSThread()
            self.ros_thread.joint_state_signal.connect(self.update_joint_display)
            self.ros_thread.start()
            self.log_status("ROS 2 initialized successfully")
        except Exception as e:
            self.log_status(f"Failed to initialize ROS 2: {e}")

    def init_ui(self):
        self.setWindowTitle('ROS 2 Robot Controller with Joint State Publisher')
        self.setGeometry(100, 100, 1000, 700)

        main_widget = QWidget()
        layout = QVBoxLayout()

        # DOF Selection
        dof_group = QGroupBox("Robot Configuration")
        dof_layout = QHBoxLayout()
        self.dof_combo = QComboBox()
        self.dof_combo.addItems([
            '0 DOF', '1 DOF', '2 DOF', '3 DOF',
            '4 DOF', '5 DOF', '6 DOF', '7 DOF'
        ])

        self.launch_btn = QPushButton('Launch Robot Configuration')
        self.launch_btn.clicked.connect(self.launch_robot)
        self.joint_gui_btn = QPushButton('Launch Joint State Publisher GUI')
        self.joint_gui_btn.clicked.connect(self.launch_joint_gui)
        self.joint_gui_btn.setEnabled(False)
        
        dof_layout.addWidget(QLabel("Select DOF:"))
        dof_layout.addWidget(self.dof_combo)
        dof_layout.addWidget(self.launch_btn)
        dof_layout.addWidget(self.joint_gui_btn)
        dof_group.setLayout(dof_layout)
        
        # Status and Joint Information
        info_splitter = QSplitter(Qt.Horizontal)
        
        # Status display
        status_group = QGroupBox("System Status")
        status_layout = QVBoxLayout()
        self.status_text = QTextEdit()
        self.status_text.setMaximumHeight(150)
        self.status_text.setReadOnly(True)
        status_layout.addWidget(self.status_text)
        status_group.setLayout(status_layout)
        
        # Joint state display
        joint_info_group = QGroupBox("Current Joint States")
        joint_info_layout = QVBoxLayout()
        self.joint_info_text = QTextEdit()
        self.joint_info_text.setReadOnly(True)
        joint_info_layout.addWidget(self.joint_info_text)
        joint_info_group.setLayout(joint_info_layout)
        
        info_splitter.addWidget(status_group)
        info_splitter.addWidget(joint_info_group)
        info_splitter.setSizes([400, 600])
        
        # Control buttons
        control_group = QGroupBox("Process Control")
        control_layout = QHBoxLayout()
        self.stop_robot_btn = QPushButton('Stop Robot')
        self.stop_robot_btn.clicked.connect(self.stop_robot)
        self.stop_joint_gui_btn = QPushButton('Stop Joint GUI')
        self.stop_joint_gui_btn.clicked.connect(self.stop_joint_gui)
        self.refresh_btn = QPushButton('Refresh Status')
        self.refresh_btn.clicked.connect(self.refresh_status)
        
        control_layout.addWidget(self.stop_robot_btn)
        control_layout.addWidget(self.stop_joint_gui_btn)
        control_layout.addWidget(self.refresh_btn)
        control_group.setLayout(control_layout)
        
        layout.addWidget(dof_group)
        layout.addWidget(info_splitter)
        layout.addWidget(control_group)
        main_widget.setLayout(layout)
        self.setCentralWidget(main_widget)
        
        self.log_status("GUI initialized. Select a robot configuration to begin.")

    def launch_robot(self):
        """Launch the selected robot MoveIt configuration in RViz"""

        if self.launch_process and self.launch_process.state() != QProcess.NotRunning:
            self.log_status("Terminating existing robot process...")
            self.launch_process.kill()
            self.launch_process.waitForFinished(5000)

        selected_index = self.dof_combo.currentIndex()
        selected_text = self.dof_combo.currentText()

        # Mapping of index to word form
        index_to_word = {
            0: "zero",
            1: "one",
            2: "two",
            3: "three",
            4: "four",
            5: "five",
            6: "six",
            7: "seven"
        }

        # Construct package and launch file names
        package_name = f"{index_to_word[selected_index]}_dof_config"
        launch_file = "demo.launch.py"


        self.current_dof = selected_index

        self.launch_process = QProcess(self)
        env = QProcessEnvironment.systemEnvironment()
        self.launch_process.setProcessEnvironment(env)
        self.launch_process.readyReadStandardOutput.connect(self.read_robot_output)
        self.launch_process.readyReadStandardError.connect(self.read_robot_error)
        self.launch_process.finished.connect(self.robot_process_finished)
        self.launch_process.started.connect(lambda: self.log_status("Robot process started successfully"))

        command = f"ros2 launch {package_name} {launch_file}"

        self.log_status(f"Launching MoveIt config: {package_name}")
        self.log_status(f"Command: {command}")

        self.launch_process.start('bash', ['-c', f'source /opt/ros/$ROS_DISTRO/setup.bash && {command}'])

        QTimer.singleShot(5000, lambda: self.joint_gui_btn.setEnabled(True))

    def launch_joint_gui(self):
        """Launch the joint_state_publisher_gui"""
        if self.joint_gui_process and self.joint_gui_process.state() != QProcess.NotRunning:
            self.log_status("Terminating existing joint GUI process...")
            self.joint_gui_process.kill()
            self.joint_gui_process.waitForFinished(3000)
        
        self.joint_gui_process = QProcess(self)
        
        # Set up environment
        env = QProcessEnvironment.systemEnvironment()
        self.joint_gui_process.setProcessEnvironment(env)
        
        # Set up process monitoring
        self.joint_gui_process.readyReadStandardOutput.connect(self.read_joint_gui_output)
        self.joint_gui_process.readyReadStandardError.connect(self.read_joint_gui_error)
        self.joint_gui_process.finished.connect(self.joint_gui_process_finished)
        self.joint_gui_process.started.connect(lambda: self.log_status("Joint State Publisher GUI started successfully"))
        
        command = "ros2 run joint_state_publisher_gui joint_state_publisher_gui"
        
        self.log_status("Launching Joint State Publisher GUI...")
        self.log_status(f"Command: {command}")
        
        # Start the process
        self.joint_gui_process.start('bash', ['-c', f'source /opt/ros/$ROS_DISTRO/setup.bash && {command}'])

    def read_robot_output(self):
        """Read and display robot process output"""
        data = self.launch_process.readAllStandardOutput()
        output = bytes(data).decode('utf-8').strip()
        if output:
            self.log_status(f"Robot: {output}")

    def read_robot_error(self):
        """Read and display robot process errors"""
        data = self.launch_process.readAllStandardError()
        error = bytes(data).decode('utf-8').strip()
        if error:
            self.log_status(f"Robot Error: {error}")

    def read_joint_gui_output(self):
        """Read and display joint GUI process output"""
        data = self.joint_gui_process.readAllStandardOutput()
        output = bytes(data).decode('utf-8').strip()
        if output:
            self.log_status(f"Joint GUI: {output}")

    def read_joint_gui_error(self):
        """Read and display joint GUI process errors"""
        data = self.joint_gui_process.readAllStandardError()
        error = bytes(data).decode('utf-8').strip()
        if error:
            self.log_status(f"Joint GUI Error: {error}")

    def stop_robot(self):
        """Stop the robot launch process"""
        if self.launch_process and self.launch_process.state() != QProcess.NotRunning:
            self.launch_process.kill()
            self.log_status("Robot process terminated.")
            self.joint_gui_btn.setEnabled(False)
        else:
            self.log_status("No robot process running.")

    def stop_joint_gui(self):
        """Stop the joint state publisher GUI"""
        if self.joint_gui_process and self.joint_gui_process.state() != QProcess.NotRunning:
            self.joint_gui_process.kill()
            self.log_status("Joint State Publisher GUI terminated.")
        else:
            self.log_status("No Joint GUI process running.")

    def robot_process_finished(self, exit_code):
        """Handle robot process completion"""
        self.log_status(f"Robot process finished with exit code: {exit_code}")
        self.joint_gui_btn.setEnabled(False)

    def joint_gui_process_finished(self, exit_code):
        """Handle joint GUI process completion"""
        self.log_status(f"Joint State Publisher GUI finished with exit code: {exit_code}")

    def update_joint_display(self, joint_state_msg):
        """Update the joint state display with current values"""
        display_text = "Current Joint States:\n\n"
        
        if joint_state_msg.name:
            for i, name in enumerate(joint_state_msg.name):
                position = joint_state_msg.position[i] if i < len(joint_state_msg.position) else 0.0
                velocity = joint_state_msg.velocity[i] if i < len(joint_state_msg.velocity) else 0.0
                effort = joint_state_msg.effort[i] if i < len(joint_state_msg.effort) else 0.0
                
                display_text += f"Joint: {name}\n"
                display_text += f"  Position: {position:.4f} rad ({position*180/3.14159:.2f}°)\n"
                display_text += f"  Velocity: {velocity:.4f} rad/s\n"
                display_text += f"  Effort: {effort:.4f}\n\n"
        else:
            display_text += "No joint states received yet.\n"
        
        self.joint_info_text.setPlainText(display_text)

    def log_status(self, message):
        """Add a status message to the status display"""
        current_time = time.strftime("%H:%M:%S")
        self.status_text.append(f"[{current_time}] {message}")
        # Auto-scroll to bottom
        self.status_text.moveCursor(self.status_text.textCursor().End)

    def refresh_status(self):
        """Refresh the status of running processes"""
        robot_status = "Running" if (self.launch_process and 
                                   self.launch_process.state() != QProcess.NotRunning) else "Stopped"
        joint_gui_status = "Running" if (self.joint_gui_process and 
                                       self.joint_gui_process.state() != QProcess.NotRunning) else "Stopped"
        
        self.log_status(f"Robot Process: {robot_status}")
        self.log_status(f"Joint GUI Process: {joint_gui_status}")
        
        # Also check ROS environment
        ros_distro = os.environ.get('ROS_DISTRO', 'Not found')
        self.log_status(f"ROS_DISTRO: {ros_distro}")

    def closeEvent(self, event):
        """Clean up when closing the application"""
        self.log_status("Shutting down...")
        
        if self.launch_process and self.launch_process.state() != QProcess.NotRunning:
            self.launch_process.kill()
            self.launch_process.waitForFinished(3000)
        
        if self.joint_gui_process and self.joint_gui_process.state() != QProcess.NotRunning:
            self.joint_gui_process.kill()
            self.joint_gui_process.waitForFinished(3000)
        
        if self.ros_thread:
            self.ros_thread.shutdown()
            self.ros_thread.wait(3000)
        
        event.accept()

if __name__ == '__main__':
    app = QApplication(sys.argv)
    window = RobotControlGUI()
    window.show()
    sys.exit(app.exec_())