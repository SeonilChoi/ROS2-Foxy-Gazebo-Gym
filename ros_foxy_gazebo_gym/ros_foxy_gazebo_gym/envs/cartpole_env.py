import sys
import time
import gym
import psutil
import numpy as np

from gym import spaces
from ros_foxy_gazebo_gym.launch import gazebo_launch

import rclpy

from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSDurabilityPolicy
from std_msgs.msg import Float64, Float64MultiArray
from sensor_msgs.msg import JointState

class CartPoleEnv(gym.Env):
    def __init__(self):
        # RL settings
        self.action_space = spaces.Box(low=-0.5, high=0.5, dtype=np.float32)
        self.observation_space = spaces.Box(
           low=np.array([-np.inf, -np.inf, -np.inf, -np.inf]),
           high=np.array([np.inf, np.inf, np.inf, np.inf]),
           dtype=np.float32)
  
        self.iterator = 0
        self.max_episode_length = 1000
        self.obs = np.zeros(4, dtype=np.float32)

        # ROS2
        rclpy.init()
        
        qos = QoSProfile(
           history=QoSHistoryPolicy.KEEP_LAST,
           durability=QoSDurabilityPolicy.VOLATILE,
           depth = 1
        )
        
        self.node = rclpy.create_node(self.__class__.__name__)
        
        self.velocity_pub = self.node.create_publisher(
           Float64MultiArray,
           "/cartpole/joint_velocity_command",
           qos
        )
        
        self.position_pub = self.node.create_publisher(
           Float64MultiArray,
           "/cartpole/joint_position_command",
           qos
        )
        
        self.joint_state_sub = self.node.create_subscription(
           JointState,
           "/cartpole/joint_states",
           self.joint_state_callback,
           qos
        )
        
        # Gazebo
        spawn_pose = {'x': '0.0', 'y': '0.0', 'z': '1.0', 'R': '0.0', 'P': '0.0', 'Y': '0.0'}
        self.gazebo_launcher = gazebo_launch.startLaunchServiceProcess(
            gazebo_launch.generate_launch_description(
               'empty.world',
               'cartpole.urdf.xacro',
               'cartpole',
               spawn_pose
            )
        )
        
        time.sleep(7) # Loading the Gazebo
        
    def reset(self, seed=None, options=None):
        msg = Float64MultiArray()
        msg.data = [0.0, 0.0]
        self.position_pub.publish(msg) 
        time.sleep(0.01)

        rclpy.spin_once(self.node)
        info = {}
        
        self.iterator = 0
        return self.obs, info

    def step(self, action):
        msg = Float64MultiArray()
        msg.data = [float(action)]
        self.velocity_pub.publish(msg)       
        time.sleep(0.01)

        rclpy.spin_once(self.node)
        value = np.abs(self.obs[1])
        reward = 1.0 if value < 0.3 else 0.0
        terminated = True if value > 0.3 else False
        truncated = True if self.iterator >= self.max_episode_length else False
        info = {}
        
        self.iterator+=1
        return self.obs, reward, terminated, truncated, info
    
    def close(self):
        self.node.destroy_node()
        parent = psutil.Process(self.gazebo_launcher.pid)
        for child in parent.children(recursive=True):
            child.kill()
        rclpy.shutdown()
        parent.kill()

    def joint_state_callback(self, msg):
        data = [msg.position[0], msg.position[1], msg.velocity[0], msg.velocity[1]]
        self.obs = np.array(data, dtype=np.float32)
        
