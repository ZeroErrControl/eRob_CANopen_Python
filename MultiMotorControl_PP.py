import canopen
import time
import logging
import can
import threading
import sys
from PyQt5.QtWidgets import QApplication
from MultiMotorControl_PP_gui import MotorControlGUI

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
        self.motor.load_configuration()
        self.initialize_node()
        time.sleep(0.1)
        self.configure_pdo()  # Configure PDO first
        time.sleep(0.1)
        self.start_node()
        time.sleep(0.1)
        self.set_immediate_effect(True)
        self.clear_fault()
        self.enable_motor()

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

    def set_controlword(self, controlword):
        """Set control word using SDO instead of PDO"""
        try:
            # First try using PDO
            logger.info(f"Motor {self.node_id}: Attempting to set control word {hex(controlword)} via PDO")
            self.motor.pdo.rx[1][0x6040].raw = controlword
            self.motor.pdo.rx[1].transmit()
            logger.info(f"Motor {self.node_id}: Control word set via PDO successfully")
        except Exception as e:
            # Fallback to SDO if PDO fails
            logger.warning(f"Motor {self.node_id}: PDO write failed for control word, using SDO: {str(e)}")
            self.motor.sdo[0x6040].raw = controlword
            logger.info(f"Motor {self.node_id}: Control word set via SDO successfully")

    def clear_fault(self):
        """Clear motor fault status with verification"""
        try:
            logger.info(f"Motor {self.node_id}: Clearing fault")
            
            # Send fault reset command
            self.motor.sdo[0x6040].raw = 0x0080
            time.sleep(0.1)
            
            # Check if fault was cleared
            status = self.motor.sdo[0x6041].raw
            if (status & 0x0008) != 0:  # Check if fault bit is still set
                logger.warning(f"Motor {self.node_id}: Fault still present after reset, retrying...")
                self.motor.sdo[0x6040].raw = 0x0080
                time.sleep(0.2)
            
            # Return to ready state
            self.motor.sdo[0x6040].raw = 0x0006
            time.sleep(0.1)
            
            status = self.motor.sdo[0x6041].raw
            logger.info(f"Motor {self.node_id}: Status after fault clear = {hex(status)}")
            
        except Exception as e:
            logger.error(f"Motor {self.node_id}: Failed to clear fault: {str(e)}")
            raise

    def initialize_node(self):
        """Initialize node"""
        try:
            # Stop remote node
            message = can.Message(
                arbitration_id=0x000,
                data=[0x02, self.node_id],
                is_extended_id=False
            )
            self.network.bus.send(message)
            time.sleep(0.1)  # Add delay
            logger.info(f"Motor {self.node_id}: Stop remote node command sent")

            # Reset node
            message = can.Message(
                arbitration_id=0x000,
                data=[0x82, self.node_id],
                is_extended_id=False
            )
            self.network.bus.send(message)
            time.sleep(0.5)  # Longer delay after reset
            logger.info(f"Motor {self.node_id}: Reset node command sent")

            # Set profile position mode
            self.motor.sdo[0x6060].raw = 0x01
            time.sleep(0.1)
            logger.info(f"Motor {self.node_id}: Profile position mode set")

            # Verify PP mode
            mode = self.motor.sdo[0x6061].raw
            if mode == 0x01:
                logger.info(f"Motor {self.node_id}: Operating mode is PP mode")
            else:
                logger.warning(f"Motor {self.node_id}: Operating mode is not PP mode (mode={mode}), attempting to set again")
                time.sleep(0.1)
                self.motor.sdo[0x6060].raw = 0x01
                time.sleep(0.1)

            # Set profile parameters with verification
            self.set_profile_parameters_with_verify(5, 5, 5)  # velocity, acceleration, deceleration

            # Disable sync generator
            self.motor.sdo[0x1006].raw = 0
            time.sleep(0.1)
            logger.info(f"Motor {self.node_id}: Sync generator disabled")

        except Exception as e:
            logger.error(f"Motor {self.node_id}: Failed to initialize node: {str(e)}")
            raise

    def set_profile_parameters_with_verify(self, velocity_rpm, acceleration_rpm2, deceleration_rpm2):
        """Set and verify profile parameters"""
        try:
            # Set profile velocity
            vel_pulse = self.velocity_to_pulse(velocity_rpm)
            self.motor.sdo[0x6081].raw = vel_pulse
            time.sleep(0.1)
            actual_vel = self.motor.sdo[0x6081].raw
            if actual_vel != vel_pulse:
                logger.warning(f"Motor {self.node_id}: Profile velocity verification failed")
                self.motor.sdo[0x6081].raw = vel_pulse

            # Set profile acceleration
            acc_pulse = self.acceleration_to_pulse(acceleration_rpm2)
            self.motor.sdo[0x6083].raw = acc_pulse
            time.sleep(0.1)
            actual_acc = self.motor.sdo[0x6083].raw
            if actual_acc != acc_pulse:
                logger.warning(f"Motor {self.node_id}: Profile acceleration verification failed")
                self.motor.sdo[0x6083].raw = acc_pulse

            # Set profile deceleration
            dec_pulse = self.acceleration_to_pulse(deceleration_rpm2)
            self.motor.sdo[0x6084].raw = dec_pulse
            time.sleep(0.1)
            actual_dec = self.motor.sdo[0x6084].raw
            if actual_dec != dec_pulse:
                logger.warning(f"Motor {self.node_id}: Profile deceleration verification failed")
                self.motor.sdo[0x6084].raw = dec_pulse

            logger.info(f"Motor {self.node_id}: Profile parameters set and verified")
        except Exception as e:
            logger.error(f"Motor {self.node_id}: Failed to set profile parameters: {str(e)}")
            raise

    def configure_pdo(self):
        """Configure TxPDO1 and RxPDO1 mapping"""
        try:
            # Calculate COB-ID based on node_id
            txpdo1_cob_id = 0x180 + self.node_id
            rxpdo1_cob_id = 0x200 + self.node_id
            
            # Enter pre-operational state
            self.motor.nmt.state = 'PRE-OPERATIONAL'
            time.sleep(0.2)  # Increased delay

            # Disable PDOs first
            self.motor.sdo[0x1400][1].raw = rxpdo1_cob_id | 0x80000000
            self.motor.sdo[0x1800][1].raw = txpdo1_cob_id | 0x80000000
            time.sleep(0.1)

            # Configure RxPDO1
            self.motor.sdo[0x1400][2].raw = 0xFF  # Asynchronous
            self.motor.sdo[0x1600][0].raw = 0  # Clear mapping
            time.sleep(0.1)
            self.motor.sdo[0x1600][1].raw = 0x60400010  # Control word
            self.motor.sdo[0x1600][2].raw = 0x607A0020  # Target position
            time.sleep(0.1)
            self.motor.sdo[0x1600][0].raw = 2  # Set number of mapped objects

            # Configure TxPDO1
            self.motor.sdo[0x1800][2].raw = 0xFF  # Asynchronous
            self.motor.sdo[0x1A00][0].raw = 0  # Clear mapping
            time.sleep(0.1)
            self.motor.sdo[0x1A00][1].raw = 0x60410010  # Status word
            self.motor.sdo[0x1A00][2].raw = 0x60640020  # Actual position
            time.sleep(0.1)
            self.motor.sdo[0x1A00][0].raw = 2  # Set number of mapped objects

            # Enable PDOs
            self.motor.sdo[0x1400][1].raw = rxpdo1_cob_id
            self.motor.sdo[0x1800][1].raw = txpdo1_cob_id
            
            time.sleep(0.2)
            logger.info(f"Motor {self.node_id}: PDO configuration completed")
            
        except Exception as e:
            logger.error(f"Motor {self.node_id}: Failed to configure PDO: {str(e)}")
            raise

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
        """Send sync frame with delay"""
        try:
            sync_message = can.Message(arbitration_id=0x080, data=[], is_extended_id=False)
            self.network.bus.send(sync_message)
            time.sleep(0.01)  # Short delay after sync
            logger.debug(f"Motor {self.node_id}: Sync frame sent")
        except Exception as e:
            logger.error(f"Motor {self.node_id}: Failed to send sync frame: {str(e)}")

    def enable_motor(self):
        """Motor enable process using SDO with complete state machine"""
        try:
            # Step 1: Read initial status
            status = self.motor.sdo[0x6041].raw
            logger.info(f"Motor {self.node_id}: Initial status = {hex(status)}")

            # Step 2: Clear any faults first
            self.clear_fault()
            time.sleep(0.1)

            # Step 3: State machine transition to Operation Enabled
            # Shutdown (Disable Voltage) - State transition 2
            self.motor.sdo[0x6040].raw = 0x0006
            time.sleep(0.1)
            
            # Switch On (Ready to Switch On) - State transition 3
            self.motor.sdo[0x6040].raw = 0x0007
            time.sleep(0.1)
            
            # Enable Operation (Operation Enabled) - State transition 4
            self.motor.sdo[0x6040].raw = 0x000F
            time.sleep(0.1)

            # Step 4: Verify operation mode is PP mode
            mode = self.motor.sdo[0x6061].raw
            if mode != 0x01:
                logger.warning(f"Motor {self.node_id}: Not in PP mode, setting PP mode")
                self.motor.sdo[0x6060].raw = 0x01
                time.sleep(0.1)

            # Step 5: Set default profile parameters
            self.set_profile_parameters(5, 5, 5)  # Default velocity and acceleration
            time.sleep(0.1)

            # Step 6: Verify enabled state
            status = self.motor.sdo[0x6041].raw
            if (status & 0x6F) == 0x27:  # Operation enabled state
                logger.info(f"Motor {self.node_id}: Successfully enabled and ready")
            else:
                logger.warning(f"Motor {self.node_id}: Unexpected status after enable: {hex(status)}")
                # Try one more time to enable
                self.motor.sdo[0x6040].raw = 0x000F
                time.sleep(0.1)

            # Step 7: Set immediate mode
            self.set_immediate_effect(True)
            time.sleep(0.1)

            # Step 8: Final verification
            status = self.motor.sdo[0x6041].raw
            mode = self.motor.sdo[0x6061].raw
            logger.info(f"Motor {self.node_id}: Final status={hex(status)}, Mode={hex(mode)}")

        except Exception as e:
            logger.error(f"Motor {self.node_id}: Failed to enable motor: {str(e)}")
            raise

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
    
    def go_to_position(self, angle):
        """Move the motor to the specified angle"""
        try:    
            # Check motor status and operation mode before moving
            status = self.motor.sdo[0x6041].raw
            mode = self.motor.sdo[0x6061].raw
            logger.info(f"Motor {self.node_id}: Status={hex(status)}, Mode={hex(mode)}")
            
            if mode != 0x01:
                logger.warning(f"Motor {self.node_id}: Not in PP mode, attempting to set PP mode")
                self.motor.sdo[0x6060].raw = 0x01
                time.sleep(0.1)
            
            # Convert angle to position
            position = self.angle_to_position(angle)
            self.target_position = position
            
            # Always use SDO for critical commands
            self.motor.sdo[0x607A].raw = position  # Set target position
            time.sleep(0.1)
            
            # Reset command trigger bit (bit 4) first using SDO
            self.motor.sdo[0x6040].raw = 0x0F
            time.sleep(0.1)
            
            # Send sync frame
            self.send_sync_frame()
            time.sleep(0.1)
            
            # Set command trigger bit using SDO
            self.motor.sdo[0x6040].raw = 0x1F
            time.sleep(0.1)
            
            # Send sync frame again
            self.send_sync_frame()
            
            # Verify movement started
            time.sleep(0.1)
            status = self.motor.sdo[0x6041].raw
            actual_pos = self.get_actual_position()
            logger.info(f"Motor {self.node_id}: Status after command={hex(status)}, Position={actual_pos}")

        except Exception as e:
            logger.error(f"Motor {self.node_id}: Failed to go to position: {str(e)}")
            self.attempt_recovery()

    def attempt_recovery(self):
        """Attempt to recover motor operation"""
        try:
            logger.info(f"Motor {self.node_id}: Attempting recovery")
            self.clear_fault()
            time.sleep(0.1)
            self.enable_motor()
            time.sleep(0.1)
            status = self.motor.sdo[0x6041].raw
            logger.info(f"Motor {self.node_id}: Status after recovery={hex(status)}")
        except Exception as e:
            logger.error(f"Motor {self.node_id}: Recovery failed: {str(e)}")

    def set_profile_velocity(self, velocity_rpm):
        """Set profile velocity"""
        try:
            vel_pulse = self.velocity_to_pulse(velocity_rpm)
            self.motor.sdo[0x6081].raw = vel_pulse
            time.sleep(0.1)
            actual_vel = self.motor.sdo[0x6081].raw
            if actual_vel != vel_pulse:
                logger.warning(f"Motor {self.node_id}: Profile velocity verification failed")
                self.motor.sdo[0x6081].raw = vel_pulse
            logger.info(f"Motor {self.node_id}: Profile velocity set to {velocity_rpm} rpm")
        except Exception as e:
            logger.error(f"Motor {self.node_id}: Failed to set profile velocity: {str(e)}")

    def set_profile_acceleration(self, acceleration_rpm2):
        """Set profile acceleration"""
        try:
            acc_pulse = self.acceleration_to_pulse(acceleration_rpm2)
            self.motor.sdo[0x6083].raw = acc_pulse
            time.sleep(0.1)
            actual_acc = self.motor.sdo[0x6083].raw
            if actual_acc != acc_pulse:
                logger.warning(f"Motor {self.node_id}: Profile acceleration verification failed")
                self.motor.sdo[0x6083].raw = acc_pulse
            logger.info(f"Motor {self.node_id}: Profile acceleration set to {acceleration_rpm2} rpm/s")
        except Exception as e:
            logger.error(f"Motor {self.node_id}: Failed to set profile acceleration: {str(e)}")

    def set_profile_deceleration(self, deceleration_rpm2):
        """Set profile deceleration"""
        try:
            dec_pulse = self.acceleration_to_pulse(deceleration_rpm2)
            self.motor.sdo[0x6084].raw = dec_pulse
            time.sleep(0.1)
            actual_dec = self.motor.sdo[0x6084].raw
            if actual_dec != dec_pulse:
                logger.warning(f"Motor {self.node_id}: Profile deceleration verification failed")
                self.motor.sdo[0x6084].raw = dec_pulse
            logger.info(f"Motor {self.node_id}: Profile deceleration set to {deceleration_rpm2} rpm/s")
        except Exception as e:
            logger.error(f"Motor {self.node_id}: Failed to set profile deceleration: {str(e)}")

    def set_profile_parameters(self, velocity_rpm, acceleration_rpm2, deceleration_rpm2):
        """Set all profile parameters at once"""
        try:
            self.set_profile_velocity(velocity_rpm)
            self.set_profile_acceleration(acceleration_rpm2)
            self.set_profile_deceleration(deceleration_rpm2)
            logger.info(f"Motor {self.node_id}: All profile parameters set successfully")
        except Exception as e:
            logger.error(f"Motor {self.node_id}: Failed to set profile parameters: {str(e)}")

def main():
    """Main program, use GUI interface to control motor"""
    network = canopen.Network()
    network.connect(bustype='socketcan', channel='can0', bitrate=1000000)
    Utils = MotorUtils()
    
    try:
        # Initialize motors with node IDs 1-6
        motors = []
        node_ids = [0x01, 0x02, 0x03, 0x04, 0x05, 0x06]  # List of available node IDs from 1 to 6
        
        for node_id in node_ids:
            try:
                motor = Motor_PP(node_id, network)
                motors.append(motor)
                logger.info(f"Successfully initialized motor with node ID {node_id}")
            except Exception as e:
                logger.warning(f"Failed to initialize motor with node ID {node_id}: {str(e)}")
        
        if not motors:
            logger.error("No motors were successfully initialized")
            return
        
        app = QApplication(sys.argv)
        gui = MotorControlGUI(motors, Utils)
        gui.show()
        
        sys.exit(app.exec_())
        
    finally:
        network.disconnect()
        logger.info("Disconnected from CAN network")

if __name__ == "__main__":
    main()