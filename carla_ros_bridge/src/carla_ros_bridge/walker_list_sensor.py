#!/usr/bin/env python
#
# Copyright (c) 2019 Intel Corporation
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
#
"""
handle a actor list sensor
"""

from carla_ros_bridge.actor import Actor
from carla_ros_bridge.pseudo_actor import PseudoActor

from detection_msgs.msg import PositionList
from geometry_msgs.msg import Point


class WalkerListSensor(PseudoActor):

    """
    Pseudo actor list sensor
    """

    def __init__(self, uid, name, carla_world, parent, node):
        """
        Constructor
        :param uid: unique identifier for this object
        :type uid: int
        :param name: name identiying this object
        :type name: string
        :param carla_world: carla world object
        :type carla_world: carla.World
        :param parent: the parent of this
        :type parent: carla_ros_bridge.Parent
        :param node: node-handle
        :type node: carla_ros_bridge.CarlaRosBridge
        """

        super(WalkerListSensor, self).__init__(uid=uid,
                                              name=name,
                                              parent=parent,
                                              node=node)
        self.world = carla_world
        self.actor_list_publisher = node.new_publisher(PositionList, self.get_topic_prefix(), qos_profile=10)

    def destroy(self):
        """
        Function to destroy this object.
        :return:
        """
        super(WalkerListSensor, self).destroy()
        self.actor_list = None
        self.node.destroy_publisher(self.actor_list_publisher)

    @staticmethod
    def get_blueprint_name():
        """
        Get the blueprint identifier for the pseudo sensor
        :return: name
        """
        return "sensor.pseudo.actor_list"

    def update(self, frame, timestamp):
        """
        Function (override) to update this object.
        """
        ros_actor_list = PositionList()

        walkers = self.world.get_actors().filter("walker.*")
        for actor in walkers:
            loc = actor.get_location()
            robot_loc=self.world.get_actors().find(self.parent.uid).get_location()
            if loc.distance(robot_loc) > 10:
                continue
            _id = actor.id
            position = Point(loc.x-robot_loc.x, -(loc.y-robot_loc.y), loc.z-robot_loc.z) #CARLA and ROS have different axis. 
            ros_actor_list.positions.append(position)
            ros_actor_list.ids.append(_id)
        ros_actor_list.header=self.get_msg_header(timestamp=timestamp)
        self.actor_list_publisher.publish(ros_actor_list)
