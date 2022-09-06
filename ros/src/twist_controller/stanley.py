import numpy as np

from geometry_msgs.msg import PoseStamped, TwistStamped

from utilities import *


class Stanley(object):
    def __init__(self, wheel_base, max_steering_angle, k, k_soft):
        """ Initialize the Stanley controller.

        Args:
            wheel_base (float):
                distance between front and rear axle
            max_steering_angle (float):
                the maximum steering angle in rad
            k:
                position gain. This value determines how much the cross track
                error affects the steering angle. Increase these value to
                increase the magnitude of the steering angle
            k_soft:
                softening gain to prevent numerical instability due to inverse
                speed
        """
        self.wheel_base = wheel_base
        self.max_steering_angle = max_steering_angle
        self.k = k
        self.k_soft = k_soft

    def step(self, current_pose, current_speed, tree):
        """ Compute the next steering angle.

        Parameters
        ----------
        current_pose: PoseStamped
            the current pose of the ego vehicle
        current_speed: float
            the current linear speed of the vehicle (twist.linear.x component)
        tree: WaypointTree
            the reference waypoints tree

        Returns
        -------
        steering: float
            the steering angle in rad
        """

        # Transform coordinates onto front axis
        ego_x, ego_y, _ = get_position_from(current_pose)
        ego_yaw = get_yaw_from(current_pose)
        ego_front_x = ego_x + self.wheel_base * np.cos(ego_yaw)
        ego_front_y = ego_y + self.wheel_base * np.sin(ego_yaw)

        # Index of the closest reference waypoint in front of the ego vehicle
        idx = tree.get_closest_idx_in_front_of([ego_front_x, ego_front_y])

        # idx = tree.get_closest_idx_in_front_of(current_pose)

        # Closest reference waypoint in front of the ego vehicle
        wp2 = tree.waypoints[idx]
        # Coordinates of the reference waypoint in front of ego vehicle
        wp2_xy = np.array(tree.xy[idx])
        # Coordinates of its predecessor
        wp1_xy = np.array(tree.xy[idx - 1])
        # Position of the ego vehicle based on the front axis
        ego_xy = np.array([ego_front_x, ego_front_y])

        # Compute the crosstrack error, that is the distance of the ego
        # vehicle's front axis from the reference trajectory
        cte = (np.cross(wp2_xy - wp1_xy, wp1_xy - ego_xy)
               / np.linalg.norm(wp2_xy - wp1_xy))

        # Compute the heading error, that is the deviation  of the ego vehicle's
        # yaw from the reference yaw
        yaw_desired = get_yaw_from(wp2.pose)
        heading_error = yaw_desired - ego_yaw
        # Handle possible yaw phase shifts
        if heading_error > np.pi:
            heading_error -= 2 * np.pi
        elif heading_error < -np.pi:
            heading_error += 2 * np.pi

        # Compute the new steering value
        cte_steer = np.arctan(self.k * cte / (self.k_soft + current_speed))
        angle = heading_error + cte_steer

        return max(-self.max_steering_angle,
                   min(self.max_steering_angle, angle))
