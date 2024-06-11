import math
import rospy
import numpy as np
import copy
import threading
import sys
from interbotix_xs_msgs.msg import *
from interbotix_xs_msgs.srv import *
from sensor_msgs.msg import JointState

### @brief Standalone Module to control an Interbotix Arm and Gripper
### @param robot_model - Interbotix Arm model (ex. 'wx200' or 'vx300s')
### @param group_name - joint group name that contains the 'arm' joints as defined in the 'motor_config' yaml file; typically, this is 'arm'
### @param gripper_name - name of the gripper joint as defined in the 'motor_config' yaml file; typically, this is 'gripper'
### @param robot_name - defaults to value given to 'robot_model'; this can be customized if controlling two of the same arms from one computer (like 'arm1/wx200' and 'arm2/wx200')
### @param moving_time - time [s] it should take for all joints in the arm to complete one move
### @param accel_time - time [s] it should take for all joints in the arm to accelerate/decelerate to/from max speed
### @param use_gripper - True if the gripper module should be initialized; otherwise, it won't be.
### @param gripper_pressure - fraction from 0 - 1 where '0' means the gripper operates at 'gripper_pressure_lower_limit' and '1' means the gripper operates at 'gripper_pressure_upper_limit'
### @param gripper_pressure_lower_limit - lowest 'effort' that should be applied to the gripper if gripper_pressure is set to 0; it should be high enough to open/close the gripper (~150 PWM or ~400 mA current)
### @param gripper_pressure_upper_limit - largest 'effort' that should be applied to the gripper if gripper_pressure is set to 1; it should be low enough that the motor doesn't 'overload' when gripping an object for a few seconds (~350 PWM or ~900 mA)
### @param init_node - set to True if the InterbotixRobotXSCore class should initialize the ROS node - this is the most Pythonic approach; to incorporate a robot into an existing ROS node though, set to False
class InterbotixManipulatorXS(object):
    def __init__(self, robot_model, group_name="arm", gripper_name=None, robot_name=None, moving_time=2.0, accel_time=0.3, gripper_pressure=0.5, gripper_pressure_lower_limit=150, gripper_pressure_upper_limit=350, init_node=True):
        
        print("Init InterbotixRobotXSCore")
        self.dxl = InterbotixRobotXSCore(robot_model, robot_name, init_node)
        
        print("Init InterbotixArmXSInterface")
        self.arm = InterbotixArmXSInterface(self.dxl, robot_model, group_name, moving_time, accel_time)
        # if gripper_name is not None:
            # self.gripper = InterbotixGripperXSInterface(self.dxl, gripper_name, gripper_pressure, gripper_pressure_lower_limit, gripper_pressure_upper_limit)


### @brief Definition of the Interbotix Arm Module
### @param core - reference to the InterbotixRobotXSCore class containing the internal ROS plumbing that drives the Python API
### @param robot_model - Interbotix Arm model (ex. 'wx200' or 'vx300s')
### @param group_name - joint group name that contains the 'arm' joints as defined in the 'motor_config' yaml file; typically, this is 'arm'
### @param moving_time - time [s] it should take for all joints in the arm to complete one move
### @param accel_time - time [s] it should take for all joints in the arm to accelerate/decelerate to/from max speed
class InterbotixArmXSInterface(object):

    def __init__(self, core, robot_model, group_name, moving_time=2.0, accel_time=0.3):
        self.core = core
        # self.group_info = self.core.srv_get_info("group", group_name)
        # if (self.group_info.profile_type != "time"):
            # rospy.logerr("Please set the group's 'profile type' to 'time'.")
        # if (self.group_info.mode != "position"):
            # rospy.logerr("Please set the group's 'operating mode' to 'position'.")
        # self.robot_des = getattr(mrd, robot_model)
        # self.initial_guesses = [[0.0] * self.group_info.num_joints for i in range(3)]
        # self.initial_guesses[1][0] = np.deg2rad(-120)
        # self.initial_guesses[2][0] = np.deg2rad(120)
        self.moving_time = None
        self.accel_time = None
        self.group_name = group_name
        self.joint_commands = []
        self.rev = 2 * math.pi
        # for name in self.group_info.joint_names:
            # self.joint_commands.append(self.core.joint_states.position[self.core.js_index_map[name]])
        # self.T_sb = mr.FKinSpace(self.robot_des.M, self.robot_des.Slist, self.joint_commands)
        self.set_trajectory_time(moving_time, accel_time)
        # self.info_index_map = dict(zip(self.group_info.joint_names, range(self.group_info.num_joints)))
        print("Arm Group Name: %s\nMoving Time: %.2f seconds\nAcceleration Time: %.2f seconds\nDrive Mode: Time-Based-Profile" % (group_name, moving_time, accel_time))
        print("Initialized InterbotixArmXSInterface!")

    ### @brief Helper function to publish joint positions and block if necessary
    ### @param positions - desired joint positions
    ### @param moving_time - duration in seconds that the robot should move
    ### @param accel_time - duration in seconds that that robot should spend accelerating/decelerating (must be less than or equal to half the moving_time)
    ### @param blocking - whether the function should wait to return control to the user until the robot finishes moving
    def publish_positions(self, positions, moving_time=None, accel_time=None, blocking=True):
        # self.set_trajectory_time(moving_time, accel_time)
        # self.joint_commands = list(positions)
        # joint_commands = JointGroupCommand(self.group_name, self.joint_commands)
        # self.core.pub_group.publish(joint_commands)
        # if blocking:
        #     rospy.sleep(self.moving_time)
        # self.T_sb = mr.FKinSpace(self.robot_des.M, self.robot_des.Slist, self.joint_commands)
        print("publish_positions")

    ### @brief Helper function to command the 'Profile_Velocity' and 'Profile_Acceleration' motor registers
    ### @param moving_time - duration in seconds that the robot should move
    ### @param accel_time - duration in seconds that that robot should spend accelerating/decelerating (must be less than or equal to half the moving_time)
    def set_trajectory_time(self, moving_time=None, accel_time=None):
        # if (moving_time is not None and moving_time != self.moving_time):
        #     self.moving_time = moving_time
        #     self.core.srv_set_reg(cmd_type="group", name=self.group_name, reg="Profile_Velocity", value=int(moving_time * 1000))
        # if (accel_time is not None and accel_time != self.accel_time):
        #     self.accel_time = accel_time
        #     self.core.srv_set_reg(cmd_type="group", name=self.group_name, reg="Profile_Acceleration", value=int(accel_time * 1000))
        print("set_trajectory_time")

    ### @brief Helper function to check to make sure the desired arm group's joint positions are all within their respective joint limits
    ### @param positions - the positions [rad] to check
    ### @return <bool> - True if all positions are within limits; False otherwise
    def check_joint_limits(self, positions):
        # theta_list = [int(elem * 1000)/1000.0 for elem in positions]
        # speed_list = [abs(goal - current)/float(self.moving_time) for goal,current in zip(theta_list, self.joint_commands)]
        # check position and velocity limits
        # for x in range(self.group_info.num_joints):
        #     if not (self.group_info.joint_lower_limits[x] <= theta_list[x] <= self.group_info.joint_upper_limits[x]):
        #         rospy.logwarn(
        #             "Would exceed position limits on joint %s." % x
        #         )
        #         rospy.logwarn(
        #             "Limits are [%f, %f], value was %f." %
        #             (self.group_info.joint_lower_limits[x],
        #             self.group_info.joint_upper_limits[x], theta_list[x])
        #         )
        #         return False
        #     if (speed_list[x] > self.group_info.joint_velocity_limits[x]):
        #         rospy.logwarn(
        #             "Would exceed velocity limits on joint %s." % x
        #         )
        #         rospy.logwarn(
        #             "Limit is %f, value was %f." %
        #             (self.group_info.joint_velocity_limits[x], theta_list[x])
        #         )
        #         return False
        return True

    ### @brief Helper function to check to make sure a desired position for a given joint is within its limits
    ### @param joint_name - desired joint name
    ### @param position - desired joint position [rad]
    ### @return <bool> - True if within limits; False otherwise
    def check_single_joint_limit(self, joint_name, position):
        theta = int(position * 1000)/1000.0
        # speed = abs(theta - self.joint_commands[self.info_index_map[joint_name]])/float(self.moving_time)
        # ll = self.group_info.joint_lower_limits[self.info_index_map[joint_name]]
        # ul = self.group_info.joint_upper_limits[self.info_index_map[joint_name]]
        # vl = self.group_info.joint_velocity_limits[self.info_index_map[joint_name]]
        # if not (ll <= theta <= ul):
        #     return False
        # if speed > vl:
        #     return False
        return True

    ### @brief Command positions to the arm joints
    ### @param joint_positions - desired joint positions [rad]
    ### @param moving_time - duration in seconds that the robot should move
    ### @param accel_time - duration in seconds that that robot should spend accelerating/decelerating (must be less than or equal to half the moving_time)
    ### @param blocking - whether the function should wait to return control to the user until the robot finishes moving
    ### @return <bool> - True if position was commanded; False if it wasn't due to being outside limits
    def set_joint_positions(self, joint_positions, moving_time=None, accel_time=None, blocking=True):
        return True
        # if (self.check_joint_limits(joint_positions)):
        #     self.publish_positions(joint_positions, moving_time, accel_time, blocking)
        #     return True
        # else:
        #     return False

    ### @brief Command the arm to go to its Home pose
    ### @param moving_time - duration in seconds that the robot should move
    ### @param accel_time - duration in seconds that that robot should spend accelerating/decelerating (must be less than or equal to half the moving_time)
    ### @param blocking - whether the function should wait to return control to the user until the robot finishes moving
    def go_to_home_pose(self, moving_time=None, accel_time=None, blocking=True):
        # self.publish_positions([0] * self.group_info.num_joints, moving_time, accel_time, blocking)
        print("go_to_home_pose")

    ### @brief Command the arm to go to its Sleep pose
    ### @param moving_time - duration in seconds that the robot should move
    ### @param accel_time - duration in seconds that that robot should spend accelerating/decelerating (must be less than or equal to half the moving_time)
    ### @param blocking - whether the function should wait to return control to the user until the robot finishes moving
    def go_to_sleep_pose(self, moving_time=None, accel_time=None, blocking=True):
        # self.publish_positions(self.group_info.joint_sleep_positions, moving_time, accel_time, blocking)
        print("go_to_sleep_pose")

    ### @brief Command a single joint to a desired position
    ### @param joint_name - name of the joint to control
    ### @param position - desired position [rad]
    ### @param moving_time - duration in seconds that the robot should move
    ### @param accel_time - duration in seconds that that robot should spend accelerating/decelerating (must be less than or equal to half the moving_time)
    ### @param blocking - whether the function should wait to return control to the user until the robot finishes moving
    ### @details - Note that if a moving_time or accel_time is specified, the changes affect ALL the arm joints, not just the specified one
    def set_single_joint_position(self, joint_name, position, moving_time=None, accel_time=None, blocking=True):
        if not self.check_single_joint_limit(joint_name, position):
            return False
        self.set_trajectory_time(moving_time, accel_time)
        self.joint_commands[self.core.js_index_map[joint_name]] = position
        single_command = JointSingleCommand(joint_name, position)
        self.core.pub_single.publish(single_command)
        if blocking:
            rospy.sleep(self.moving_time)
        self.T_sb = mr.FKinSpace(self.robot_des.M, self.robot_des.Slist, self.joint_commands)
        return True

    ### @brief Command a desired end-effector pose
    ### @param T_sd - 4x4 Transformation Matrix representing the transform from the /<robot_name>/base_link frame to the /<robot_name>/ee_gripper_link frame
    ### @param custom_guess - list of joint positions with which to seed the IK solver
    ### @param execute - if True, this moves the physical robot after planning; otherwise, only planning is done
    ### @param moving_time - duration in seconds that the robot should move
    ### @param accel_time - duration in seconds that that robot should spend accelerating/decelerating (must be less than or equal to half the moving_time)
    ### @param blocking - whether the function should wait to return control to the user until the robot finishes moving
    ### @return theta_list - joint values needed to get the end-effector to the desired pose
    ### @return <bool> - True if a valid solution was found; False otherwise
    def set_ee_pose_matrix(self, T_sd, custom_guess=None, execute=True, moving_time=None, accel_time=None, blocking=True):
        
        return [], False

    ### @brief Command a desired end-effector pose w.r.t. the Space frame
    ### @param x - linear position along the X-axis of the Space frame [m]
    ### @param y - linear position along the Y-axis of the Space frame [m]
    ### @param z - linear position along the Z-axis of the Space frame [m]
    ### @param roll - angular position around the X-axis of the Space frame [rad]
    ### @param pitch - angular position around the Y-axis of the Space frame [rad]
    ### @param yaw - angular position around the Z-axis of the Space frame [rad]
    ### @param custom_guess - list of joint positions with which to seed the IK solver
    ### @param execute - if True, this moves the physical robot after planning; otherwise, only planning is done
    ### @param moving_time - duration in seconds that the robot should move
    ### @param accel_time - duration in seconds that that robot should spend accelerating/decelerating (must be less than or equal to half the moving_time)
    ### @param blocking - whether the function should wait to return control to the user until the robot finishes moving
    ### @return theta_list - joint values needed to get the end-effector to the desired pose
    ### @return <bool> - True if a valid solution was found; False otherwise
    ### @details - Do not set 'yaw' if using an arm with fewer than 6dof
    def set_ee_pose_components(self, x=0, y=0, z=0, roll=0, pitch=0, yaw=None, custom_guess=None, execute=True, moving_time=None, accel_time=None, blocking=True):
        
        return self.set_ee_pose_matrix([], custom_guess, execute, moving_time, accel_time, blocking)

    ### @brief Command a desired end-effector displacement that will follow a straight line path (when in 'position' control mode)
    ### @param x - linear displacement along the X-axis w.r.t. T_sy [m]
    ### @param y - linear displacement along the Y-axis w.r.t. T_sy [m]
    ### @param z - linear displacement along the Z-axis w.r.t. T_sy [m]
    ### @param roll - angular displacement around the X-axis w.r.t. T_sy [rad]
    ### @param pitch - angular displacement around the Y-axis w.r.t. T_sy [rad]
    ### @param yaw - angular displacement around the Z-axis w.r.t. T_sy [rad]
    ### @param moving_time - duration in seconds that the robot should move
    ### @param wp_moving_time - duration in seconds that each waypoint in the trajectory should move
    ### @param wp_accel_time - duration in seconds that each waypoint in the trajectory should be accelerating/decelerating (must be equal to or less than half of wp_moving_time)
    ### @param wp_period - duration in seconds between each waypoint
    ### @return <bool> - True if a trajectory was successfully planned and executed; otherwise False
    ### @details - T_sy is a 4x4 transformation matrix representing the pose of a virtual frame w.r.t. /<robot_name>/base_link.
    ###            This virtual frame has the exact same x, y, z, roll, and pitch of /<robot_name>/base_link but contains the yaw
    ###            of the end-effector frame (/<robot_name>/ee_gripper_link).
    ###            Note that 'y' and 'yaw' must equal 0 if using arms with less than 6dof.
    def set_ee_cartesian_trajectory(self, x=0, y=0, z=0, roll=0, pitch=0, yaw=0, moving_time=None, wp_moving_time=0.2, wp_accel_time=0.1, wp_period=0.05):
        
        return True

    ### @brief Command displacements to the end effector's position w.r.t. the Space frame
    ### @param dx - linear displacement along the X-axis of the Space frame [m]
    ### @param dy - linear displacement along the Y-axis of the Space frame [m]
    ### @param dz - linear displacement along the Z-axis of the Space frame [m]
    ### @param custom_guess - list of joint positions with which to seed the IK solver
    ### @param execute - if True, this moves the physical robot after planning; otherwise, only planning is done
    ### @param moving_time - duration in seconds that the robot should move
    ### @param accel_time - duration in seconds that that robot should spend accelerating/decelerating (must be less than or equal to half the moving_time)
    ### @param blocking - whether the function should wait to return control to the user until the robot finishes moving
    ### @return theta_list - joint values needed to get the end-effector to the desired pose
    ### @return <bool> - True if a valid solution was found; False otherwise
    ### @details - Do not set 'yaw' if using an arm with fewer than 6dof
    def set_relative_ee_position_wrt_to_base_frame(self, *, dx=0, dy=0, dz=0, custom_guess=None, execute=True, moving_time=None, accel_time=None, blocking=True):
        return self.set_ee_pose_components(
            x=self.T_sb[0, 3] + dx,
            y=self.T_sb[1, 3] + dy,
            z=self.T_sb[2, 3] + dz,
            custom_guess=custom_guess,
            execute=execute,
            moving_time=moving_time,
            accel_time=accel_time,
            blocking=blocking,
        )

    ### @brief Get the latest commanded joint positions
    ### @return - list of latest commanded joint positions [rad]
    def get_joint_commands(self):
        return list(self.joint_commands)

    ### @brief Get the latest commanded position for a given joint
    ### @param joint_name - joint for which to get the position
    ### @return - desired position [rad]
    def get_single_joint_command(self, joint_name):
        # return self.joint_commands[self.info_index_map[joint_name]]
        print("get_single_joint_command")
        return [0.0]

    ### @brief Get the latest commanded end-effector pose w.r.t the Space frame
    ### @return <4x4 matrix> - Transformation matrix
    def get_ee_pose_command(self):
        return np.array(self.T_sb)

    ### @brief Get the actual end-effector pose w.r.t the Space frame
    ### @return <4x4 matrix> - Transformation matrix
    def get_ee_pose(self):
        # Example transformation matrix (replace with your specific values)
        transformation_matrix = np.array([
            [1, 0, 0, 0],
            [0, 1, 0, 0],
            [0, 0, 1, 0],
            [0, 0, 0, 1]
        ], dtype=np.float64)
    
        return transformation_matrix

    ### @brief Resets self.joint_commands to be the actual positions seen by the encoders
    ### @details - should be used whenever joints are torqued off, right after torquing them on again
    def capture_joint_positions(self):
        print(__file__)

### @brief Class that interfaces with the xs_sdk node ROS interfaces
### @param robot_model - Interbotix Arm model (ex. 'wx200' or 'vx300s')
### @param robot_name - defaults to value given to 'robot_model'; this can be customized if controlling two of the same arms from one computer (like 'arm1/wx200' and 'arm2/wx200')
### @param init_node - set to True if the InterbotixRobotXSCore class should initialize the ROS node - this is the most Pythonic approach; to incorporate a robot into an existing ROS node though, set to False
### @param joint_state_topic - the specifc JointState topic output by the xs_sdk node
class InterbotixRobotXSCore(object):
    def __init__(self, robot_model, robot_name=None, init_node=True, joint_state_topic="joint_states"):
        self.joint_states = None
        self.js_mutex = threading.Lock()
        self.robot_name = robot_name
        if (self.robot_name is None):
            self.robot_name = robot_model
        if (init_node):
            print("Init node")
            rospy.init_node(self.robot_name + "_robot_manipulation")

        # Try to find the xs_sdk services under the 'robot_name' namespace
        # If the services can't be found after 5 seconds, we catch the exception
        #   and gracefully exit the program with a hint
        # try:
        #     rospy.wait_for_service(
        #         "/" + self.robot_name + "/set_operating_modes", timeout=5.0)
        #     rospy.wait_for_service("/" + self.robot_name + "/set_motor_pid_gains")
        #     rospy.wait_for_service("/" + self.robot_name + "/set_motor_registers")
        #     rospy.wait_for_service("/" + self.robot_name + "/get_motor_registers")
        #     rospy.wait_for_service("/" + self.robot_name + "/get_robot_info")
        #     rospy.wait_for_service("/" + self.robot_name + "/torque_enable")
        #     rospy.wait_for_service("/" + self.robot_name + "/reboot_motors")
        # except rospy.exceptions.ROSException as e:
        #     print(str(e.args[0]))
        #     print((
        #         "The robot '%s' is not discoverable. "
        #         "Did you enter the correct robot_name parameter? "
        #         "Is the xs_sdk node running? "
        #         "Quitting..." % self.robot_name))
        #     sys.exit(1)

        # self.srv_set_op_modes = rospy.ServiceProxy("/" + self.robot_name + "/set_operating_modes", OperatingModes)
        # self.srv_set_pids = rospy.ServiceProxy("/" + self.robot_name + "/set_motor_pid_gains", MotorGains)
        # self.srv_set_reg = rospy.ServiceProxy("/" + self.robot_name + "/set_motor_registers", RegisterValues)
        # self.srv_get_reg = rospy.ServiceProxy("/" + self.robot_name + "/get_motor_registers", RegisterValues)
        # self.srv_get_info = rospy.ServiceProxy("/" + self.robot_name + "/get_robot_info", RobotInfo)
        # self.srv_torque = rospy.ServiceProxy("/" + self.robot_name + "/torque_enable", TorqueEnable)
        # self.srv_reboot = rospy.ServiceProxy("/" + self.robot_name + "/reboot_motors", Reboot)
        # self.pub_group = rospy.Publisher("/" + self.robot_name + "/commands/joint_group", JointGroupCommand, queue_size=1)
        # self.pub_single = rospy.Publisher("/" + self.robot_name + "/commands/joint_single", JointSingleCommand, queue_size=1)
        # self.pub_traj = rospy.Publisher("/" + self.robot_name + "/commands/joint_trajectory", JointTrajectoryCommand, queue_size=1)
        # self.sub_joint_states = rospy.Subscriber("/" + self.robot_name + "/" + joint_state_topic, JointState, self.joint_state_cb)
        self.sub_joint_states = rospy.Subscriber("/" + joint_state_topic, JointState, self.joint_state_cb)
        while (self.joint_states == None and not rospy.is_shutdown()): 
            print("InterbotixRobotXSCore::Init::Waiting joint states populated.") 
            rospy.sleep(0.2)
        self.js_index_map = dict(zip(self.joint_states.name, range(len(self.joint_states.name))))
        rospy.sleep(0.5)
        print("Robot Name: %s\nRobot Model: %s" % (self.robot_name, robot_model))
        print("Initialized InterbotixRobotXSCore!\n")

    ### @brief Set the operating mode for either a single motor or a group of motors
    ### @param cmd_type - can be "group" for a group of motors or "single" for a single motor
    ### @param name - group name if cmd_type is 'group' or the motor name if cmd_type is 'single'
    ### @param mode - desired operating mode like "position" or "velocity". See the OperatingModes Service description for all choices
    ### @param profile_type - can be "time" or "velocity". See the OperatingModes Service description for details
    ### @param profile_velocity - passthrough to the Profile_Velocity register. See the OperatingModes Service description for details
    ### @param profile_acceleration - passthrough to the Profile_Acceleration register. See the OperatingModes Service description for details
    def robot_set_operating_modes(self, cmd_type, name, mode, profile_type="velocity", profile_velocity=0, profile_acceleration=0):
        print("robot_set_operating_modes")
        # self.srv_set_op_modes(cmd_type, name, mode, profile_type, profile_velocity, profile_acceleration)

    ### @brief Set the internal PID gains for either a single motor or a group of motors
    ### @param cmd_type - can be "group" for a group of motors or "single" for a single motor
    ### @param name - group name if cmd_type is 'group' or the motor name if cmd_type is 'single'
    ### @param kp_pos - passthrough to the Position_P_Gain register. See the MotorGains Service description for details
    ### @param ki_pos - passthrough to the Position_I_Gain register. See the MotorGains Service description for details
    ### @param kd_pos - passthrough to the Position_D_Gain register. See the MotorGains Service description for details
    ### @param k1 - passthrough to the Feedforward_1st_Gain register. See the MotorGains Service description for details
    ### @param k2 - passthrough to the Feedforward_2nd_Gain register. See the MotorGains Service description for details
    ### @param kp_vel - passthrough to the Velocity_P_Gain register. See the MotorGains Service description for details
    ### @param ki_vel - passthrough to the Velocity_I_Gain register. See the MotorGains Service description for details
    def robot_set_motor_pid_gains(self, cmd_type, name, kp_pos, ki_pos=0, kd_pos=0, k1=0, k2=0, kp_vel=100, ki_vel=1920):
        print("robot_set_motor_pid_gains")

    ### @brief Set the desired register for either a single motor or a group of motors
    ### @param cmd_type - can be "group" for a group of motors or "single" for a single motor
    ### @param name - group name if cmd_type is 'group' or the motor name if cmd_type is 'single'
    ### @param reg - desired register name
    ### @param value - desired value for the above register
    def robot_set_motor_registers(self, cmd_type, name, reg, value):
        print("robot_set_motor_registers")
        # self.srv_set_reg(cmd_type, name, reg, value)

    ### @brief Get the desired register value from either a single motor or a group of motors
    ### @param cmd_type - can be "group" for a group of motors or "single" for a single motor
    ### @param name - group name if cmd_type is 'group' or the motor name if cmd_type is 'single'
    ### @param reg - desired register name
    ### @return response - list of register values
    def robot_get_motor_registers(self, cmd_type, name, reg):
        print("robot_get_motor_registers")
        # response = self.srv_get_reg(cmd_type=cmd_type, name=name, reg=reg)
        return []

    ### @brief Get information about the robot - mostly joint limit data
    ### @param cmd_type - can be "group" for a group of motors or "single" for a single motor
    ### @param name - group name if cmd_type is 'group' or the motor name if cmd_type is 'single'
    ### @return response - an object with the same structure as a RobotInfo Service description
    def robot_get_robot_info(self, cmd_type, name):
        print("robot_get_motor_registers")
        # response = self.srv_get_info(cmd_type, name)
        return []

    ### @brief Torque a single motor or a group of motors to be on or off
    ### @param cmd_type - can be "group" for a group of motors or "single" for a single motor
    ### @param name - group name if cmd_type is 'group' or the motor name if cmd_type is 'single'
    ### @param enable - True to torque on or False to torque off
    def robot_torque_enable(self, cmd_type, name, enable):
        print("robot_torque_enable")
        # self.srv_torque(cmd_type, name, enable)

    ### @brief Reboot a single motor or a group of motors if they are in an error state
    ### @param cmd_type - can be "group" for a group of motors or "single" for a single motor
    ### @param name - group name if cmd_type is 'group' or the motor name if cmd_type is 'single'
    ### @param enable - True to torque on or False to leave torqued off after rebooting
    ### @param smart_reboot - if 'cmd_type' is set to 'group', setting this to True will only reboot
    ###                       those motors that are in an error state (as opposed to all motors
    ###                       within the group regardless of if they are in an error state)
    def robot_reboot_motors(self, cmd_type, name, enable, smart_reboot=False):
        print("robot_reboot_motors")
        # self.srv_reboot(cmd_type, name, enable, smart_reboot)

    ### @brief Command a group of motors (refer to the JointGroupCommand Message description for more info)
    ### @param group_name - the group name of the motors to command
    ### @param commands - desired list of commands
    def robot_write_commands(self, group_name, commands):
        print("robot_write_commands")

    ### @brief Command a single motor (refer to the JointSingleCommand Message description for more info)
    ### @param joint_name - the name of the motor to command
    ### @param command - desired command
    def robot_write_joint_command(self, joint_name, command):
        print("robot_write_joint_command")

    ### @brief Command a trajectory of positions or velocities to a single motor or a group of motors
    ### @param cmd_type - can be "group" for a group of motors or "single" for a single motor
    ### @param name - group name if cmd_type is 'group' or the motor name if cmd_type is 'single'
    ### @param type - "position" if the trajectory is a list of positions [rad]; otherwise "velocity" if the
    ###               trajectory is a list of velocities [rad/s]
    ### @param raw_traj - list of dictionaries where each dictionary is made up of a float / list of float pairs.
    ###                   the 'key' is the desired time [sec] from start that the 'value' (list of floats) should be executed.
    ### @details - an example input trajectory for a pan/tilt mechanism could look like...
    ###            [{0, [1,1]},
    ###             {1.5, [-1,0.75]},
    ###             {2.3, [0,0]}]
    def robot_write_trajectory(self, cmd_type, name, type, raw_traj):
        print("robot_write_trajectory")

    ### @brief Get the current joint states (position, velocity, effort) of all Dynamixel motors
    ### @return joint_states - JointState ROS message. Refer to online documentation to see its structure
    def robot_get_joint_states(self):
        joint_states = None
        with self.js_mutex:
            joint_states = copy.deepcopy(self.joint_states)
        return joint_states

    ### @brief Get a single joint state for the specified Dynamixel motor
    ### @param name - desired motor name for which to get the joint state
    ### @return joint_info - dictionary with 3 keys: "position", "velocity", and "effort".
    ###                      Units are rad, rad/s, and mA
    def robot_get_single_joint_state(self, name):
        joint_states = None
        with self.js_mutex:
            joint_states = copy.deepcopy(self.joint_states)
        joint_index = joint_states.name.index(name)
        joint_info = {}
        joint_info["position"] = joint_states.position[joint_index]
        joint_info["velocity"] = joint_states.velocity[joint_index]
        joint_info["effort"] = joint_states.effort[joint_index]
        return joint_info

    ### @brief ROS Subscriber Callback function to get the latest JointState message
    def joint_state_cb(self, msg):
        with self.js_mutex:
            self.joint_states = msg