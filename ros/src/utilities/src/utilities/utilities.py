""" Helper functions """

import numpy as np
import rospy

from scipy.spatial import KDTree

from geometry_msgs.msg import Point, Pose, PoseStamped
from tf.transformations import euler_from_quaternion
from styx_msgs.msg import Waypoint, Lane

__all__ = ['Logger', 'WaypointTree', 'set_linear_speed_of',
           'get_position_from', 'get_orientation_from', 'get_yaw_from']

LOG_THRESHOLD = 0.5  # [s], passed time to print next log messages


class Logger(object):
    def __init__(self):
        self._last_time = rospy.get_time()
        self._active = None

    @property
    def active(self):
        if self._active is None:
            current_time = rospy.get_time()
            if current_time - self._last_time > LOG_THRESHOLD:
                self._last_time = current_time
                self._active = True
            else:
                self._active = False
        return self._active

    def reset(self):
        self._active = None

    def info(self, message, *args):
        if self.active:
            rospy.loginfo(message, *args)

    @staticmethod
    def warn(message, *args):
        print("")
        rospy.logwarn(message, *args)
        print("")


class WaypointTree(object):
    def __init__(self, waypoints):
        """
        Args:
            waypoints (Lane)
        """
        self.header = waypoints.header
        self.waypoints = waypoints.waypoints
        self.num_waypoints = len(self.waypoints)
        self.xy = [get_position_from(wp)[:-1] for wp in self.waypoints]
        self.tree = KDTree(self.xy)

    def __getitem__(self, item):
        """ Slice the refrence waypoints of the tree.

        Indices exceeding the length of the waypoint list are wrapped to the
        beginning of the list.

        Args:
            item (slice|int):
                the slice's step attribute is not accounted
        """
        if isinstance(item, slice):
            start = item.start % self.num_waypoints
            stop = item.stop % self.num_waypoints
            if stop < start:
                return self.waypoints[start:] + self.waypoints[:stop]
            else:
                return self.waypoints[start:stop]
        else:
            return self.waypoints[item]

    def get_closest_idx_from(self, message):
        """ Get reference waypoint index closest to provided message's position.

        Args:
            message (Pose|PoseStamped|Waypoint|list):
                the message to get the closest waypoint index for. If a list is
                passed, it must contain the position's coordinates.

        Returns:
            the index to the closest reference waypoint in the tree.
        """
        if isinstance(message, list):
            xy_position = message[:2]
        else:
            xy_position = get_position_from(message)[:-1]
        return self.tree.query(xy_position, 1)[1]

    def get_cumulated_dists_from(self, start, stop):
        """ Get a vector of cumulated distances from a start to a stop waypoint.

        Each entry in the vector is the distance from the respective waypoint to
        the stop waypoint, computed by summing up the piecewise distances
        of all waypoints in between. For instance, the first entry is the full
        distance from the start waypoint to the stop waypoint.

        Args:
            start (int):
                waypoint index of the start waypoint in the reference tree
            stop (int):
                waypoint index of the stop waypoint in the reference tree

        Returns:
            array of cumulated distances from the start to the stop waypoint
        """
        p0 = [get_position_from(wp) for wp in self[start:stop]]
        p1 = [get_position_from(wp) for wp in self[start + 1: stop + 1]]
        piecewise_dists = np.linalg.norm(np.subtract(p1, p0), axis=1)
        return np.cumsum(piecewise_dists[::-1])[::-1]


def set_linear_speed_of(waypoint, speed):
    """ Set the linear speed component of the given waypoint's twist.

    Args:
        waypoint (Waypoint):
            the waypoint to set the speed for
        speed:
            the linear speed to set

    Returns:
        the updated waypoint copy
    """
    wp = Waypoint()
    wp.pose = waypoint.pose
    wp.twist.twist.linear.x = min(speed, waypoint.twist.twist.linear.x)
    return wp


def get_position_from(message):
    """ Extract and convert the `message's` position into a list.

    Args:
        message (Pose|PoseStamped|Waypoint):
            the message to get the position from

    Returns:
        position list [x, y, z]
    """
    if isinstance(message, (Pose, PoseStamped)):
        return [message.pose.position.x,
                message.pose.position.y,
                message.pose.position.z]
    elif isinstance(message, Waypoint):
        return [message.pose.pose.position.x,
                message.pose.pose.position.y,
                message.pose.pose.position.z]


def get_orientation_from(pose):
    """ Extract and convert the pose's orientation into a list.

    Args:
        pose (PoseStamped): the pose to extract the position from

    Returns:
        the orientation quaternion list [x, y, z, w]
    """
    return [pose.pose.orientation.x,
            pose.pose.orientation.y,
            pose.pose.orientation.z,
            pose.pose.orientation.w]


def get_yaw_from(pose):
    """ Extract the yaw from the given pose

    Args:
        pose (PoseStamped|Pose): the pose to convert and extract the yaw from

    Returns:
        the pose's yaw
    """
    return euler_from_quaternion(get_orientation_from(pose))[2]
