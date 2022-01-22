import numpy as np

from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import PoseStamped, TwistStamped

from utilities import *

class Stanley(object):
    def __init__(self, reference_waypoints, wheel_base, max_steering_angle):

        self.wheel_base = wheel_base
        self.max_steering_angle = max_steering_angle

    def step(self, current_pose):
        """

        Parameters
        ----------
        current_pose: PoseStamped
            the current pose of the ego vehicle

        Returns
        -------
        steering: float
            the steering angle in degrees
        """

        # Transform coordinates onto front axis
        ego_x, ego_y, _ = get_position_from(current_pose)
        ego_yaw = get_yaw_from(current_pose)
        ego_front_x = ego_x + self.wheel_base * np.cos(ego_yaw)
        ego_front_y = ego_y + self.wheel_base * np.sin(ego_yaw)

        # TODO: Yaw desired on basis of next waypoint
        # TODO: Crosstrack error on basis of next waypoint

        # Calculate cross track error
        offset = 100
        min_dist = float("inf")
        min_idx = 0
        for i in range(len(waypoints)):
            dist = np.linalg.norm(
                np.array([waypoints[i][0] - xf, waypoints[i][1] - yf]))
            if dist < min_dist:
                min_dist = dist
                min_idx = i
        wp1 = np.array(self._waypoints[min_idx][0:2])
        if min_idx < len(self._waypoints) - offset - 1:
            wp2 = np.array(self._waypoints[min_idx + offset][0:2])
            yaw_desired = np.arctan2(wp2[1] - wp1[1], wp2[0] - wp1[0])
        else:
            wp2 = np.array(self._waypoints[-offset][0:2])
            yaw_desired = np.arctan2(wp1[1] - wp2[1], wp1[0] - wp2[0])

        pf = np.array([xf, yf])
        cte = np.cross(wp2 - wp1, wp1 - pf) / np.linalg.norm(wp2 - wp1)
        heading_error = yaw_desired - yaw  # in rad

        # Parameters
        k = 1
        k_soft = 1

        # Change the steer output with the lateral controller.
        cte_steer = np.arctan(k * cte / (k_soft + v))
        steer_output = heading_error + cte_steer

        if steer_output > 1.22:
            steer_output = 1.22
        elif steer_output < -1.22:
            steer_output = -1.22
