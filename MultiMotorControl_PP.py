import canopen
import time
import logging
import can
import threading
import sys
from PyQt5.QtWidgets import QApplication
from motor_control_gui import MotorControlGUI

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

class MotorAngleReader:
    def __init__(self, motors, utils, interval=1.0):
        """
        Initialize MotorAngleReader class
        :param motors: List of motors to read angles from
        :param interval: Time interval between readings (seconds)
        """
        self.motors = motors
        self.interval = interval
        self.running = False
        self.thread = None
        self.utils = utils

    def start(self):
        """Start the angle reading thread"""
        if not self.running:
            self.running = True
            self.thread = threading.Thread(target=self._read_angles)
            self.thread.start()

    def stop(self):
        """Stop the angle reading thread"""
        if self.running:
            self.running = False
            if self.thread is not None:
                self.thread.join()

    def _read_angles(self):
        """Periodically read angles from each motor"""
        while self.running:
            for motor in self.motors:
                actual_position = motor.motor.sdo[0x6064].raw  # Read actual position
                angle = self.utils.position_to_angle(actual_position)  # Convert to angle
                logger.debug(f"Motor {motor.node_id}: Current angle is {angle:.2f}°")
            time.sleep(self.interval)

class MotorUtils:
    def __init__(self, encoder_resolution=524288):
        self.encoder_resolution = encoder_resolution

    def angle_to_position(self, angle):
        """Convert angle to position pulse value"""
        position = (angle / 360) * self.encoder_resolution
        return int(position)

    def position_to_angle(self, position):
        """Convert position pulse value to angle"""
        angle = (position / self.encoder_resolution) * 360
        return angle
    
    def velocity_to_pulse(self, velocity_rpm):
        """Convert velocity to pulse value"""
        velocity_pulse_per_sec = (velocity_rpm / 60) * self.encoder_resolution
        return int(velocity_pulse_per_sec)

    def acceleration_to_pulse(self, acceleration_rpm2):
        """Convert acceleration to pulse value"""
        acceleration_pulse_per_sec2 = (acceleration_rpm2 / 60) * self.encoder_resolution
        return int(acceleration_pulse_per_sec2)

class Motor_PP:
    def __init__(self, node_id, network, encoder_resolution=524288):
        self.node_id = node_id
        self.network = network
        self.encoder_resolution = encoder_resolution
        self.motor = self.network.add_node(self.node_id, "ZeroErr Driver_V1.5.eds")
        self.motor.load_configuration()  # Load configuration to prevent custom PDO mapping override
        self.initialize_node()  # Initialize node
        self.configure_pdo()  # Configure PDO mapping
        time.sleep(0.1)
        self.start_node()  # Start node
        self.set_immediate_effect(True)  # Set immediate effect
        self.clear_fault()  # Clear fault
        self.enable_motor()  # Enable motor

    def angle_to_position(self, angle):
        """Convert angle to position pulse value"""
        position = (angle / 360) * self.encoder_resolution
        return int(position)

    def position_to_angle(self, position):
        """Convert position pulse value to angle"""
        angle = (position / self.encoder_resolution) * 360
        return angle
    
    def velocity_to_pulse(self, velocity_rpm):
        """Convert velocity to pulse value"""
        velocity_pulse_per_sec = (velocity_rpm / 60) * self.encoder_resolution
        return int(velocity_pulse_per_sec)

    def acceleration_to_pulse(self, acceleration_rpm2):
        """Convert acceleration to pulse value"""
        acceleration_pulse_per_sec2 = (acceleration_rpm2 / 60) * self.encoder_resolution
        return int(acceleration_pulse_per_sec2)

    def clear_fault(self):
        """Clear motor fault status"""
        logger.info(f"Motor {self.node_id}: Clearing fault")
        self.set_controlword(0x0080)  # Send fault reset command
        time.sleep(0.1)
        self.set_controlword(0x0006)  # Enter ready state
        time.sleep(0.1)

    def initialize_node(self):
        """Initialize node"""
        # Stop remote node
        message = can.Message(
            arbitration_id=0x000,
            data=[0x02, self.node_id],
            is_extended_id=False
        )
        self.network.bus.send(message)
        logger.info(f"Motor {self.node_id}: Stop remote node command sent")

        # Reset node
        message = can.Message(
            arbitration_id=0x000,
            data=[0x82, self.node_id],
            is_extended_id=False
        )
        self.network.bus.send(message)
        logger.info(f"Motor {self.node_id}: Reset node command sent")

        # Set profile position mode
        self.motor.sdo[0x6060].raw = 0x01
        logger.info(f"Motor {self.node_id}: Profile position mode set")

        # Verify PP mode
        if self.motor.sdo[0x6061].raw == 0x01:
            logger.info(f"Motor {self.node_id}: Operating mode is PP mode")
        else:
            logger.warning(f"Motor {self.node_id}: Operating mode is not PP mode, please check configuration")

        # Set profile velocity
        self.motor.sdo[0x6081].raw = self.velocity_to_pulse(5)
        logger.info(f"Motor {self.node_id}: Profile velocity set to 10°/s")

        # Set profile acceleration
        self.motor.sdo[0x6083].raw = self.acceleration_to_pulse(5)
        logger.info(f"Motor {self.node_id}: Profile acceleration set to 10°/s²")

        # Set profile deceleration
        self.motor.sdo[0x6084].raw = self.acceleration_to_pulse(5)
        logger.info(f"Motor {self.node_id}: Profile deceleration set to 10°/s²")

        # Disable sync generator
        self.motor.sdo[0x1006].raw = 0
        logger.info(f"Motor {self.node_id}: Sync generator disabled")

        # Set communication cycle to 1000us
        self.motor.sdo[0x1006].raw = 1000
        logger.info(f"Motor {self.node_id}: Communication cycle set to 1000us")

    def set_profile_velocity(self, velocity_rpm):
        """Set profile velocity"""
        self.motor.sdo[0x6081].raw = self.velocity_to_pulse(velocity_rpm)
        logger.info(f"Motor {self.node_id}: Profile velocity set to {velocity_rpm}°/s")

    def set_profile_acceleration(self, acceleration_rpm2):
        """Set profile acceleration"""
        self.motor.sdo[0x6083].raw = self.acceleration_to_pulse(acceleration_rpm2)
        logger.info(f"Motor {self.node_id}: Profile acceleration set to {acceleration_rpm2}°/s²")

    def set_profile_deceleration(self, deceleration_rpm2):
        """Set profile deceleration"""
        self.motor.sdo[0x6084].raw = self.acceleration_to_pulse(deceleration_rpm2)
        logger.info(f"Motor {self.node_id}: Profile deceleration set to {deceleration_rpm2}°/s²")

    def set_profile_parameters(self, velocity_rpm, acceleration_rpm2, deceleration_rpm2):
        """Set profile velocity, acceleration, and deceleration at once"""
        self.set_profile_velocity(velocity_rpm)
        self.set_profile_acceleration(acceleration_rpm2)
        self.set_profile_deceleration(deceleration_rpm2)
        logger.info(f"Motor {self.node_id}: Profile parameters set completed - Velocity: {velocity_rpm}°/s, Acceleration: {acceleration_rpm2}°/s², Deceleration: {deceleration_rpm2}°/s²")

    def configure_pdo(self):
        """Configure TxPDO1 and RxPDO1 mapping according to official manual"""
        
        # Calculate COB-ID based on node_id
        txpdo1_cob_id = 0x180 + self.node_id
        rxpdo1_cob_id = 0x200 + self.node_id
        self.motor.nmt.state = 'PRE-OPERATIONAL'
        logger.debug(f"Motor {self.node_id}: Entered PRE-OPERATIONAL state")

        # Configure TxPDO1
        logger.info(f"Motor {self.node_id}: Start configuring TxPDO1")
        
        # 1. Disable TxPDO1
        self.motor.sdo[0x1800][1].raw = txpdo1_cob_id | 0x80000000  # COB-ID + Disable bit
        logger.debug(f"Motor {self.node_id}: Disable TxPDO1")

        # 2. Set transmission type
        self.motor.sdo[0x1800][2].raw = 0x01  # Asynchronous transmission
        logger.debug(f"Motor {self.node_id}: Set TxPDO1 transmission type to Asynchronous (0x01)")

        # 3. Clear TxPDO1 mapping
        self.motor.sdo[0x1A00][0].raw = 0x00
        logger.debug(f"Motor {self.node_id}: Clear TxPDO1 original mapping")

        # 4. Set mapping object: Status word (Status Word)
        self.motor.sdo[0x1A00][1].raw = 0x60410010
        logger.debug(f"Motor {self.node_id}: Map Status word (Status Word) to TxPDO1")

        # 5. Set mapping object: Actual position (Actual Position)
        self.motor.sdo[0x1A00][2].raw = 0x60640020
        logger.debug(f"Motor {self.node_id}: Map Actual position (Actual Position) to TxPDO1")

        # 6. Set TxPDO1 mapping object count to 2
        self.motor.sdo[0x1A00][0].raw = 0x02
        logger.debug(f"Motor {self.node_id}: Set TxPDO1 mapping object count to 2")

        # 7. Set transmission type and enable TxPDO1
        self.motor.sdo[0x1800][2].raw = 0xFF  # Asynchronous transmission type
        self.motor.sdo[0x1800][1].raw = txpdo1_cob_id  # Remove Disable bit, set COB-ID
        logger.debug(f"Motor {self.node_id}: Enable TxPDO1 and set transmission type to Asynchronous (0xFF)")

        # Configure RxPDO1
        logger.info(f"Motor {self.node_id}: Start configuring RxPDO1")

        # 1. Disable RxPDO1
        self.motor.sdo[0x1400][1].raw = rxpdo1_cob_id | 0x80000000  # COB-ID + Disable bit
        logger.debug(f"Motor {self.node_id}: Disable RxPDO1")

        # 2. Set transmission type
        self.motor.sdo[0x1400][2].raw = 0x01  # Asynchronous transmission
        logger.debug(f"Motor {self.node_id}: Set RxPDO1 transmission type to Asynchronous (0x01)")

        # 3. Clear RxPDO1 mapping
        self.motor.sdo[0x1600][0].raw = 0x00
        logger.debug(f"Motor {self.node_id}: Clear RxPDO1 original mapping")

        # 4. Set mapping object: Control word (Control Word)
        self.motor.sdo[0x1600][1].raw = 0x60400010
        logger.debug(f"Motor {self.node_id}: Map Control word (Control Word) to RxPDO1")

        # 5. Set mapping object: Target position (Target Position)
        self.motor.sdo[0x1600][2].raw = 0x607A0020
        logger.debug(f"Motor {self.node_id}: Map Target position (Target Position) to RxPDO1")

        # 6. Set RxPDO1 mapping object count to 2
        self.motor.sdo[0x1600][0].raw = 0x02
        logger.debug(f"Motor {self.node_id}: Set RxPDO1 mapping object count to 2")

        # 7. Set transmission type and enable RxPDO1
        self.motor.sdo[0x1400][2].raw = 0xFF  # Asynchronous transmission type
        self.motor.sdo[0x1400][1].raw = rxpdo1_cob_id  # Remove Disable bit, set COB-ID
        logger.debug(f"Motor {self.node_id}: Enable RxPDO1 and set transmission type to Asynchronous (0xFF)")

        logger.info(f"Motor {self.node_id}: PDO configuration completed")
    
    def start_node(self):
        """Start node"""
        self.network.nmt.send_command(0x01)  # NMT start node
        logger.info(f"Motor {self.node_id}: NMT start remote node completed")
        # Get actual position
        actual_position = self.motor.sdo[0x6064].raw  # 0x6064 is the actual position object dictionary index
        actual_position = self.position_to_angle(actual_position)
        logger.info(f"Actual position: {round(actual_position, 2)}°")
        # Send sync frame
        self.send_sync_frame()

    def set_immediate_effect(self, immediate):
        """Set immediate effect or non-immediate effect"""
        controlword = self.motor.sdo[0x6040].raw
        if immediate:
            controlword |= (1 << 5)  # Set Bit 5 to 1 (Immediate effect)
            logger.info(f"Motor {self.node_id}: Set to Immediate effect")
        else:
            controlword &= ~(1 << 5)  # Set Bit 5 to 0 (Non-immediate effect)
            logger.info(f"Motor {self.node_id}: Set to Non-immediate effect")
        self.motor.sdo[0x6040].raw = controlword
        logger.debug(f"Motor {self.node_id}: Control word updated to: {hex(controlword)}")

    def send_sync_frame(self):
        """Send sync frame"""
        sync_message = can.Message(arbitration_id=0x080, data=[], is_extended_id=False)
        self.network.bus.send(sync_message)
        logger.debug(f"Motor {self.node_id}: Sync frame sent completed")

    def enable_motor(self):
        """Motor enable process, use PDO for control"""
        self.set_controlword(0x06)  # Shutdown (Close)
        self.send_sync_frame()
        time.sleep(0.1)
        self.set_controlword(0x07)  # Switch on (Ready to enable)
        self.send_sync_frame()
        time.sleep(0.1)
        self.set_controlword(0x0F)  # Enable operation (Enable)
        self.send_sync_frame()
        time.sleep(0.1)
        logger.info(f"Motor {self.node_id}: Motor enable completed")

    def check_target_reached(self):
        """Monitor whether the target position is reached"""
        #statusword = self.motor.pdo.tx[1]['Statusword'].raw
        statusword = self.motor.sdo[0x6041].raw  # Read Statusword (0x6041)
        return (statusword & 0x0400) != 0  # Check the 10th bit of Statusword

    def monitor_target_reached(self, timeout=10):
        """Monitor whether the motor has reached the target position"""
        start_time = time.time()
        while self.monitoring.is_set():
            if self.check_target_reached():
                actual_position = self.motor.sdo[0x6064].raw
                logger.info(f"Motor {self.node_id}: Reached target position, current actual position: {actual_position}")
                self.monitoring.clear()  # Stop monitoring
                break
            if time.time() - start_time > timeout:
                logger.warning(f"Motor {self.node_id}: Unable to reach target position within {timeout} seconds")
                self.monitoring.clear()
                break
            logger.info(f"Motor {self.node_id}: Target not reached. Current position: {self.get_actual_position()}")
            time.sleep(0.1)

    def get_actual_position(self):
        """Get the current actual position of the motor, read through SDO"""
        try:
            actual_position = self.motor.sdo[0x6064].raw
            logger.debug(f"Motor {self.node_id}: Current actual position is {actual_position} plus")
            return actual_position
        except Exception as e:
            logger.error(f"Motor {self.node_id}: Failed to get actual position: {str(e)}")
            return None
    
    def set_controlword(self, controlword):
        """Set control word to 0x6040"""
        self.motor.pdo.rx[1]['Controlword'].raw = controlword
        self.motor.pdo.rx[1].transmit()
        logger.debug(f"Motor {self.node_id}: Send control word through PDO: {hex(controlword)}")

    def go_to_position(self, angle):
        """Move the motor to the specified angle"""
        position = self.angle_to_position(angle)
        self.target_position = position
        
        # Set target position
        self.motor.pdo.rx[1]['Target Position'].raw = position
        self.send_sync_frame()
        
        # Reset command trigger bit (bit 4) first
        self.set_controlword(0x0F)  # Enable operation without command trigger
        self.send_sync_frame()
        time.sleep(0.1)  # Small delay to ensure edge triggering
        
        # Set command trigger bit to create rising edge
        self.set_controlword(0x1F)  # Command triggered (correct value: 0x1F)
        self.send_sync_frame()


def main():
    """Main program, use GUI interface to control motor"""
    network = canopen.Network()
    network.connect(bustype='socketcan', channel='can0', bitrate=1000000)
    Utils = MotorUtils()
    
    try:
        motor = Motor_PP(0x02, network)
        
        app = QApplication(sys.argv)
        gui = MotorControlGUI(motor, Utils)
        gui.show()
        
        sys.exit(app.exec_())
        
    finally:
        network.disconnect()
        logger.info("Disconnected from CAN network")

if __name__ == "__main__":
    main()