#!/usr/bin/env python

#
# Copyright (c) 2019 Intel Corporation
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
#
"""
Control Carla ego vehicle by using AckermannDrive messages
"""

import sys

import numpy
from simple_pid import PID  # pylint: disable=import-error,wrong-import-order

import ros_compatibility as roscomp
from ros_compatibility.node import CompatibleNode

from carla_ackermann_control import carla_control_physics as phys

from ackermann_msgs.msg import AckermannDrive  # pylint: disable=import-error,wrong-import-order
from std_msgs.msg import Header # pylint: disable=wrong-import-order
from carla_msgs.msg import CarlaEgoVehicleStatus  # pylint: disable=no-name-in-module,import-error
from carla_msgs.msg import CarlaEgoVehicleControl  # pylint: disable=no-name-in-module,import-error
from carla_msgs.msg import CarlaEgoVehicleInfo  # pylint: disable=no-name-in-module,import-error
from carla_ackermann_msgs.msg import EgoVehicleControlInfo  # pylint: disable=no-name-in-module,import-error
from geometry_msgs.msg import PoseArray

ROS_VERSION = roscomp.get_ros_version()

if ROS_VERSION == 1:
    from carla_ackermann_control.cfg import EgoVehicleControlParameterConfig # pylint: disable=no-name-in-module,import-error,ungrouped-imports
if ROS_VERSION == 2:
    from rcl_interfaces.msg import SetParametersResult


class CarlaAckermannControl(CompatibleNode):

    """
    Convert ackermann_drive messages to carla VehicleCommand with a PID controller
    """

    def __init__(self):
        """
        Constructor

        """
        super(CarlaAckermannControl, self).__init__("carla_ackermann_control")

        # PID controller
        # the controller has to run with the simulation time, not with real-time
        Kp=self.get_param("speed_Kp", 2.0)
        Kd=self.get_param("speed_Kd", 0.5)
        Ki=self.get_param("speed_Ki", 0.5)
        self.pid_controller=PID_Controller(Kp,Ki,Kd,0.0,1.0)
        self.previous_time = self.get_time()-0.1

        self.control_loop_rate = self.get_param("control_loop_rate", 0.1)
        self.last_ackermann_msg_received_sec =  self.get_time()
        self.vehicle_status = CarlaEgoVehicleStatus()
        self.vehicle_info = CarlaEgoVehicleInfo()
        self.role_name = self.get_param('role_name', 'ego_vehicle')
        # control info
        self.info = EgoVehicleControlInfo()

        # set initial maximum values
        self.vehicle_info_updated(self.vehicle_info)

        # target values
        self.info.target.steering_angle = 0.
        self.info.target.speed = 0.
        self.info.target.speed_abs = 0.
        self.info.target.accel = 0.
        self.info.target.jerk = 0.

        # current values
        self.info.current.time_sec = self.get_time()
        self.info.current.speed = 0.
        self.info.current.speed_abs = 0.
        self.info.current.accel = 0.

        # control values
        self.info.status.status = 'n/a'
        self.info.status.speed_control_activation_count = 0
        self.info.status.speed_control_accel_delta = 0.
        self.info.status.speed_control_accel_target = 0.
        self.info.status.accel_control_pedal_delta = 0.
        self.info.status.accel_control_pedal_target = 0.
        self.info.status.brake_upper_border = 0.
        self.info.status.throttle_lower_border = 0.

        # control output
        self.info.output.throttle = 0.
        self.info.output.brake = 1.0
        self.info.output.steer = 0.
        self.info.output.reverse = False
        self.info.output.hand_brake = False

        # ackermann drive commands
        self.control_subscriber = self.new_subscription(
            AckermannDrive,
            "/carla/" + self.role_name + "/ackermann_cmd",
            self.ackermann_command_updated,
            qos_profile=10
        )

        # current status of the vehicle
        self.vehicle_status_subscriber = self.new_subscription(
            CarlaEgoVehicleStatus,
            "/carla/" + self.role_name + "/vehicle_status",
            self.vehicle_status_updated,
            qos_profile=10
        )

        # vehicle info
        self.vehicle_info_subscriber = self.new_subscription(
            CarlaEgoVehicleInfo,
            "/carla/" + self.role_name + "/vehicle_info",
            self.vehicle_info_updated,
            qos_profile=10
        )

        # to send command to carla
        self.carla_control_publisher = self.new_publisher(
            CarlaEgoVehicleControl,
            "/carla/" + self.role_name + "/vehicle_control_cmd",
            qos_profile=1)

        # report controller info
        self.control_info_publisher = self.new_publisher(
            EgoVehicleControlInfo,
            "/carla/" + self.role_name + "/ackermann_control/control_info",
            qos_profile=1)
        

    def get_msg_header(self):
        """
        Get a filled ROS message header
        :return: ROS message header
        :rtype: std_msgs.msg.Header
        """
        header = Header()
        header.frame_id = "map"
        header.stamp = roscomp.ros_timestamp(sec=self.get_time(), from_sec=True)
        return header

    def vehicle_status_updated(self, vehicle_status):
        """
        Stores the ackermann drive message for the next controller calculation

        :param ros_ackermann_drive: the current ackermann control input
        :type ros_ackermann_drive: ackermann_msgs.AckermannDrive
        :return:
        """

        # set target values
        self.vehicle_status = vehicle_status

    def vehicle_info_updated(self, vehicle_info):
        """
        Stores the ackermann drive message for the next controller calculation

        :param ros_ackermann_drive: the current ackermann control input
        :type ros_ackermann_drive: ackermann_msgs.AckermannDrive
        :return:
        """
        # set target values
        self.vehicle_info = vehicle_info

        # calculate restrictions
        self.info.restrictions.max_steering_angle = self.get_param("max_steering", 24.0)
        self.info.restrictions.max_speed = self.get_param("max_speed", 1.3)
        # clipping the pedal in both directions to the same range using the usual lower
        # border: the max_accel to ensure the the pedal target is in symmetry to zero
        self.info.restrictions.max_pedal = min(
            self.info.restrictions.max_accel, self.info.restrictions.max_decel)

    def ackermann_command_updated(self, ros_ackermann_drive):
        """
        Stores the ackermann drive message for the next controller calculation

        :param ros_ackermann_drive: the current ackermann control input
        :type ros_ackermann_drive: ackermann_msgs.AckermannDrive
        :return:
        """
        self.last_ackermann_msg_received_sec = self.get_time()
        # set target values
        self.set_target_steering_angle(ros_ackermann_drive.steering_angle)
        self.set_target_speed(ros_ackermann_drive.speed)

    def set_target_steering_angle(self, target_steering_angle):
        """
        set target sterring angle
        """
        self.info.target.steering_angle = -target_steering_angle*180/numpy.pi #Rad to Deg
        self.info.target.steering_angle = numpy.clip(
                self.info.target.steering_angle,
                -self.info.restrictions.max_steering_angle,
                self.info.restrictions.max_steering_angle)

    def set_target_speed(self, target_speed):
        """
        set target speed
        """
        if abs(target_speed) > self.info.restrictions.max_speed:
            self.logerr("Max speed reached, clipping value")
            self.info.target.speed = numpy.clip(
                target_speed, -self.info.restrictions.max_speed, self.info.restrictions.max_speed)
        else:
            self.info.target.speed = target_speed
        self.info.target.speed_abs = abs(self.info.target.speed)


    def vehicle_control_cycle(self):
        """
        Perform a vehicle control cycle and sends out CarlaEgoVehicleControl message
        """
        # perform actual control
        self.control_steering()
        self.run_speed_control_loop()

        # only send out the Carla Control Command if AckermannDrive messages are
        # received in the last second (e.g. to allows manually controlling the vehicle)
        if (self.last_ackermann_msg_received_sec + 1.0) > \
                self.get_time():
            self.info.output.header = self.get_msg_header()
            self.carla_control_publisher.publish(self.info.output)

    def control_steering(self):
        """
        Basic steering control
        """
        target_steer = numpy.clip(self.info.target.steering_angle / \
            self.info.restrictions.max_steering_angle,-1.0,1.0)
        if target_steer > self.info.output.steer + 0.2:
            self.info.output.steer = self.info.output.steer + 0.2
        elif target_steer < self.info.output.steer - 0.2:
            self.info.output.steer = self.info.output.steer - 0.2
        else:
            self.info.output.steer=target_steer

    def run_speed_control_loop(self):
        """
        Run the PID control loop for the speed
        """
        control_gear = -1 if self.vehicle_status.control.reverse else 1
        diference=self.info.target.speed-self.info.current.speed_abs
        if self.info.target.speed *control_gear>0 and diference*control_gear>-0.7:
            dt = self.get_time() - self.previous_time
            self.info.output.throttle = self.pid_controller.step(self.info.target.speed*control_gear,self.info.current.speed_abs, dt)
            self.info.output.brake = 0
        else:
            self.info.output.throttle = 0
            if  self.info.current.speed_abs <= 0.1 and abs(self.info.target.speed)>0.0:
                self.info.output.reverse = not self.vehicle_status.control.reverse
                self.info.output.brake  = 0
            else:
                self.info.output.brake  = 0.1
        self.previous_time = self.get_time()


    # from ego vehicle
    def send_ego_vehicle_control_info_msg(self):
        """
        Function to send carla_ackermann_control.msg.EgoVehicleControlInfo message.

        :return:
        """
        self.info.header = self.get_msg_header()
        self.control_info_publisher.publish(self.info)

    def update_current_values(self):
        """
        Function to update vehicle control current values.

        we calculate the acceleration on ourselves, because we are interested only in
        the acceleration in respect to the driving direction
        In addition a small average filter is applied

        :return:
        """
        current_time_sec = self.get_time()
        delta_time = current_time_sec - self.info.current.time_sec
        current_speed = self.vehicle_status.velocity
        if delta_time > 0:
            delta_speed = current_speed - self.info.current.speed
            current_accel = delta_speed / delta_time
            # average filter
            self.info.current.accel = (self.info.current.accel * 4 + current_accel) / 5
        self.info.current.time_sec = current_time_sec
        self.info.current.speed = current_speed
        self.info.current.speed_abs = abs(current_speed)

    def run(self):
        """

        Control loop

        :return:
        """

        def loop(timer_event=None):
            self.update_current_values()
            self.vehicle_control_cycle()
            self.send_ego_vehicle_control_info_msg()

        self.new_timer(self.control_loop_rate, loop)
        self.spin()


# ==============================================================================
# -- PID Controller ------------------------------------------------------------
# ==============================================================================

class PID_Controller(object):
    def __init__(self, kp, ki, kd, min_value, max_value):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.max_value, self.min_value = max_value, min_value
        self.de, self.ie = 0, 0
        self.previous = 0

    def step(self, target_speed, current_speed, dt):
        error = target_speed - current_speed

        self.de = (self.previous-error) / dt

        #Reset integral part when the target speed changes significally
        self.ie = min(max(self.ie+error * dt,-1.0),1.0) if abs(error)<0.25 else 0.0

        self.previous=error
        return numpy.clip(self.kp * error + self.ki*self.ie + self.kd*self.de, self.min_value, self.max_value)
    
def main(args=None):
    """

    main function

    :return:
    """
    roscomp.init("carla_ackermann_control", args=args)

    try:
        controller = CarlaAckermannControl()
        controller.run()
    except KeyboardInterrupt:
        pass
    finally:
        roscomp.shutdown()

if __name__ == "__main__":
    main()
