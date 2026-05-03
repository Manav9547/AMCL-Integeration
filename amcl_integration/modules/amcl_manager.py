"""
modules/amcl_manager.py
─────────────────────────────────────────────────────
Manages the dynamic launching and stopping of the AMCL
and Map Server nodes via subprocess.
"""

import os
import subprocess
import signal
import logging
import threading
from typing import Optional

from modules import state
import modules.amcl_state as amcl_state
from modules import ros_node

logger = logging.getLogger(__name__)

_amcl_process: Optional[subprocess.Popen] = None
_manager_lock = threading.Lock()

def start_amcl(map_yaml_path: str) -> bool:
    """Start the AMCL and Map Server nodes via amcl_launch.py."""
    global _amcl_process
    
    with _manager_lock:
        if _amcl_process is not None:
            logger.warning("AMCL is already running.")
            return True

        if not os.path.exists(map_yaml_path):
            logger.error(f"Map file not found: {map_yaml_path}")
            return False

        # First, pause SLAM to prevent map drift if it's running
        ros_node.pause_slam()
        with state.ros_lock:
            state.SLAM_MODE = "localization"

        # Locate the amcl_launch.py script
        # Assuming it's in the root of the control directory
        base_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        launch_file = os.path.join(base_dir, "amcl_launch.py")

        if not os.path.exists(launch_file):
            logger.error(f"AMCL launch file not found at: {launch_file}")
            return False

        cmd = [
            "ros2", "launch", launch_file,
            f"map:={map_yaml_path}"
        ]

        logger.info(f"Starting AMCL with command: {' '.join(cmd)}")
        try:
            # Run in a new session so we can kill the entire process group
            _amcl_process = subprocess.Popen(
                cmd,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                preexec_fn=os.setsid
            )
            with amcl_state.amcl_lock:
                amcl_state.AMCL_ACTIVE = True
                amcl_state.LOCALIZATION_MODE = "amcl"
                amcl_state.AMCL_INITIAL_POSE_SET = False
            return True
        except Exception as e:
            logger.error(f"Failed to start AMCL: {e}")
            return False

def stop_amcl() -> bool:
    """Stop the AMCL and Map Server nodes."""
    global _amcl_process
    
    with _manager_lock:
        if _amcl_process is None:
            return True
            
        logger.info("Stopping AMCL...")
        try:
            # Kill the process group to ensure child nodes die
            os.killpg(os.getpgid(_amcl_process.pid), signal.SIGTERM)
            _amcl_process.wait(timeout=5.0)
        except subprocess.TimeoutExpired:
            os.killpg(os.getpgid(_amcl_process.pid), signal.SIGKILL)
        except Exception as e:
            logger.error(f"Error stopping AMCL: {e}")
        finally:
            _amcl_process = None
            with amcl_state.amcl_lock:
                amcl_state.AMCL_ACTIVE = False
                amcl_state.LOCALIZATION_MODE = "slam"
            
            # Resume SLAM
            ros_node.resume_slam()
            ros_node.clear_map_override()
            with state.ros_lock:
                state.SLAM_MODE = "mapping"
                
            return True

def set_initial_pose(x: float, y: float, yaw: float) -> bool:
    """Set the initial pose for AMCL using the /initialpose topic."""
    if not amcl_state.AMCL_ACTIVE:
        logger.warning("Cannot set initial pose: AMCL is not active.")
        return False
        
    node = ros_node.get_node()
    if node is None:
        return False
        
    try:
        from geometry_msgs.msg import PoseWithCovarianceStamped
        from modules.ros_node import _yaw_to_quaternion
        
        # Create a temporary publisher for /initialpose
        pub = node.create_publisher(PoseWithCovarianceStamped, "/initialpose", 10)
        
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = node.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.position.z = 0.0
        msg.pose.pose.orientation = _yaw_to_quaternion(yaw)
        
        # Moderate initial covariance
        msg.pose.covariance[0] = 0.25  # xx
        msg.pose.covariance[7] = 0.25  # yy
        msg.pose.covariance[35] = 0.068  # yaw-yaw (~15 degrees)
        
        pub.publish(msg)
        logger.info(f"Published initial pose: x={x}, y={y}, yaw={yaw}")
        
        with amcl_state.amcl_lock:
            amcl_state.AMCL_INITIAL_POSE_SET = True
            
        # Optional: destroy publisher if you don't want to keep it around
        node.destroy_publisher(pub)
        return True
    except Exception as e:
        logger.error(f"Failed to publish initial pose: {e}")
        return False

def reinitialize_global_localization() -> bool:
    """Trigger global relocalization by scattering particles randomly."""
    if not amcl_state.AMCL_ACTIVE:
        return False
        
    node = ros_node.get_node()
    if node is None:
        return False
        
    try:
        from std_srvs.srv import Empty
        client = node.create_client(Empty, "/reinitialize_global_localization")
        if not client.wait_for_service(timeout_sec=2.0):
            logger.warning("/reinitialize_global_localization service not available")
            return False
            
        future = client.call_async(Empty.Request())
        logger.info("Triggered global localization")
        
        with amcl_state.amcl_lock:
            amcl_state.AMCL_INITIAL_POSE_SET = False
            amcl_state.AMCL_CONVERGED = False
            
        return True
    except Exception as e:
        logger.error(f"Failed to trigger global localization: {e}")
        return False
