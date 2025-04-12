import time
import sys
import numpy as np

from getch import getch

from unitree_sdk2py.core.channel import ChannelSubscriber, ChannelPublisher
from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowState_
from controllers.synchronous_controller import SynchronousController
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_
from unitree_sdk2py.utils.thread import RecurrentThread
from utils import movements


class SafetyMonitor:
    """
    Class for monitoring robot safety conditions in a background thread.
    
    This class subscribes to robot state updates and continuously checks for
    safety violations (joint limits, velocities, etc).
    """
    
    def __init__(self, controller:SynchronousController, monitoring_interval=0.02, safety_limits=None):
        """
        Initialize the safety monitor.
        
        Args:
            monitoring_interval (float): Check interval (default 50 Hz)
            safety_limits (dict, optional): Dictionary of safety limits to check
        """
        self.monitoring_interval = monitoring_interval
        self.safety_limits = safety_limits or self._default_safety_limits()

        self.controller = controller
        
        # State variables
        self.low_state = None
        self.first_state_received = False
        self.is_initialized = False
        self.is_running = False
        self.monitor_thread = None
        self.safety_violations = []
        self.emergency_triggered = False
        
    def _default_safety_limits(self):
        """
        Define default safety limits if none provided.
        
        Returns:
            dict: Default safety limits configuration
        """
        limits = {
            "joint_limits": {
            # Левая нога
            0: {"min": -2.5307, "max": 2.8798, "name": "L_LEG_HIP_PITCH"},      # L_LEG_HIP_PITCH
            1: {"min": -0.5236, "max": 2.9671, "name": "L_LEG_HIP_ROLL"},       # L_LEG_HIP_ROLL
            2: {"min": -2.7576, "max": 2.7576, "name": "L_LEG_HIP_YAW"},        # L_LEG_HIP_YAW
            3: {"min": -0.087267, "max": 2.8798, "name": "L_LEG_KNEE"},         # L_LEG_KNEE
            4: {"min": -0.87267, "max": 0.5236, "name": "L_LEG_ANKLE_PITCH"},   # L_LEG_ANKLE_PITCH
            5: {"min": -0.2618, "max": 0.2618, "name": "L_LEG_ANKLE_ROLL"},     # L_LEG_ANKLE_ROLL
            
            # Правая нога
            6: {"min": -2.5307, "max": 2.8798, "name": "R_LEG_HIP_PITCH"},      # R_LEG_HIP_PITCH
            7: {"min": -2.9671, "max": 0.5236, "name": "R_LEG_HIP_ROLL"},       # R_LEG_HIP_ROLL
            8: {"min": -2.7576, "max": 2.7576, "name": "R_LEG_HIP_YAW"},        # R_LEG_HIP_YAW
            9: {"min": -0.087267, "max": 2.8798, "name": "R_LEG_KNEE"},         # R_LEG_KNEE
            10: {"min": -0.87267, "max": 0.5236, "name": "R_LEG_ANKLE_PITCH"},  # R_LEG_ANKLE_PITCH
            11: {"min": -0.2618, "max": 0.2618, "name": "R_LEG_ANKLE_ROLL"},    # R_LEG_ANKLE_ROLL

            #  Талия
            12: {"min": -2.618, "max": 2.618, "name": "WAIST_YAW"},      # WAIST_YAW
            13: {"min": -0.52, "max": 0.52, "name": "WAIST_ROLL"},       # WAIST_ROLL
            14: {"min": -0.52, "max": 0.52, "name": "WAIST_PITCH"},      # WAIST_PITCH

            #  Левая рука
            15: {"min": -3.089, "max": 2.669, "name": "L_SHOULDER_PITCH"},      # L_SHOULDER_PITCH
            16: {"min": -1.588, "max": 2.251, "name": "L_SHOULDER_ROLL"},         # L_SHOULDER_ROLL
            17: {"min": -2.618, "max": 2.618, "name": "L_SHOULDER_YAW"},          # L_SHOULDER_YAW
            18: {"min": -1.047, "max": 2.094, "name": "L_ELBOW"},               # L_ELBOW
            19: {"min": -1.971, "max": 1.971, "name": "L_WRIST_ROLL"},            # L_WRIST_ROLL
            20: {"min": -1.613, "max": 1.613, "name": "L_WRIST_PITCH"},           # L_WRIST_PITCH
            21: {"min": -1.613, "max": 1.613, "name": "L_WRIST_YAW"},           # L_WRIST_YAW

            #  Правая рука
            22: {"min": -3.089, "max": 2.669, "name": "R_SHOULDER_PITCH"},      # R_SHOULDER_PITCH
            23: {"min": -2.251, "max": 1.588, "name": "R_SHOULDER_ROLL"},         # R_SHOULDER_ROLL
            24: {"min": -2.607, "max": 2.607, "name": "R_SHOULDER_YAW"},          # R_SHOULDER_YAW
            25: {"min": -1.041, "max": 2.084, "name": "R_ELBOW"},               # R_ELBOW
            26: {"min": -1.971, "max": 1.971, "name": "R_WRIST_ROLL"},            # R_WRIST_ROLL
            27: {"min": -1.613, "max": 1.613, "name": "R_WRIST_PITCH"},           # R_WRIST_PITCH
            28: {"min": -1.613, "max": 1.613, "name": "R_WRIST_YAW"},           # R_WRIST_YAW
            }
        }

        return limits
    
    def init(self):
        """Initialize the safety monitoring system."""
        if self.is_initialized:
            print("[SafetyMonitor] Already initialized")
            return True
            
        # Create subscriber to robot state
        self.state_subscriber = ChannelSubscriber("rt/lowstate", LowState_)
        self.state_subscriber.Init(self.low_state_handler, 10)
        
        self.is_initialized = True
        print("[SafetyMonitor] Initialized successfully")
        return True
    
    def low_state_handler(self, msg: LowState_):
        """
        Callback handler for robot state updates.
        
        Args:
            msg (LowState_): The robot state message
        """
        self.low_state = msg
        
        if not self.first_state_received:
            self.first_state_received = True
            print("[SafetyMonitor] First state update received")
    
    def start(self):
        """Start the safety monitoring thread."""
        if not self.is_initialized:
            print("[SafetyMonitor] Must initialize before starting")
            return False
            
        if self.is_running:
            print("[SafetyMonitor] Already running")
            return True
            
        # Wait until we've received at least one state update
        timeout = 10  # seconds
        start_time = time.time()
        while not self.first_state_received:
            time.sleep(0.1)
            if time.time() - start_time > timeout:
                print("[SafetyMonitor] Timeout waiting for first state update")
                return False
        
        # Create and start the recurring thread
        self.monitor_thread = RecurrentThread(
            interval=self.monitoring_interval,
            target=self.check_safety,
            name="safety_monitor"
        )
        self.monitor_thread.Start()
        self.is_running = True
        print("[SafetyMonitor] Monitoring thread started")
        return True
    
    def stop(self):
        """Stop the safety monitoring thread."""
        if not self.is_running:
            return
            
        if self.monitor_thread:
            self.monitor_thread.Wait(1.0)
            self.monitor_thread = None
            
        self.is_running = False
        print("[SafetyMonitor] Monitoring stopped")
    
    def check_safety(self):
        """
        Check robot state against safety limits.
        This method runs periodically in the background thread.
        """
        if self.low_state is None:
            return
            
        # Reset violations for this check
        self.safety_violations = []
        
        # Check joint positions
        for i, motor_state in enumerate(self.low_state.motor_state):
            if i not in self.safety_limits["joint_limits"]:
                continue

            # Check position limits
            joint_limits = self.safety_limits["joint_limits"][i]
            joint_name = joint_limits["name"]

            if motor_state.q > joint_limits["max"] or motor_state.q < joint_limits["min"]:
                    self.safety_violations.append(
                        f"Joint {i} ({joint_name}) position out of range: {motor_state.q:.4f}, "
                        f"limits: [{joint_limits['min']:.4f}, {joint_limits['max']:.4f}]"
                    )
            
            # Check velocity limits
            if abs(motor_state.dq) > 1.0:
                self.safety_violations.append(f"Joint {i} velocity too high: {motor_state.dq}")
        
            if self.safety_violations:
                self.handle_safety_violation()
    
    def handle_safety_violation(self):
        """Handle any detected safety violations."""
        # Print violations
        for violation in self.safety_violations:
            print(f"[SafetyMonitor] VIOLATION: {violation}")
        
        # Trigger emergency stop if needed
        if not self.emergency_triggered:
            self.trigger_emergency()
    
    def trigger_emergency(self):
        """Trigger emergency stop procedure."""
        self.emergency_triggered = True
        print("[SafetyMonitor] EMERGENCY STOP TRIGGERED")
        
        while movements.go_home(self.controller):
            print("[SafetyMonitor] Going home")

        print("Press any key to continue")
        getch()
        self.reset_emergency()

    def reset_emergency(self):
        """Reset emergency stop state."""
        if self.emergency_triggered:
            self.emergency_triggered = False
            print("[SafetyMonitor] Emergency state reset")
    
    def get_safety_status(self):
        """
        Get current safety status.
        
        Returns:
            dict: Current safety status information
        """
        return {
            "is_monitoring": self.is_running,
            "emergency_triggered": self.emergency_triggered,
            "safety_violations": self.safety_violations.copy() if self.safety_violations else []
        }


# Example usage:
if __name__ == "__main__":    
    # Create and start safety monitor
    controller = SynchronousController(
            is_in_local=True
        )

    safety_monitor = SafetyMonitor(controller=controller,
                                    monitoring_interval=0.05)

    safety_monitor.init()
    
    if safety_monitor.start():
        print("Safety monitoring started.")
        
        try:
            while True:
                time.sleep(1)
                status = safety_monitor.get_safety_status()
                print(f"Monitoring active: {status['is_monitoring']}, " 
                      f"Violations: {len(status['safety_violations'])}")
        except KeyboardInterrupt:
            print("Stopping safety monitor...")
        finally:
            safety_monitor.stop()
            print("Safety monitor stopped.")