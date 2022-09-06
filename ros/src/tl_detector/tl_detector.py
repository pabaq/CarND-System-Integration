#!/usr/bin/env python
import math
import os.path

import rospy
import tf
import cv2
import yaml
import numpy as np

from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped, Pose, TwistStamped
from styx_msgs.msg import TrafficLightArray, TrafficLight
from styx_msgs.msg import Lane
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from light_classification.tl_classifier import TLClassifier

from utilities import *

# used for logging
LIGHT_STATES_LOG = {TrafficLight.UNKNOWN: "Unknown", TrafficLight.RED: "red",
                    TrafficLight.YELLOW: "yellow", TrafficLight.GREEN: "green"}


class TLDetector(object):
    def __init__(self):
        # Initialize the /tl_detector node
        rospy.init_node('tl_detector')
        # Subscribe to the required topics, which are
        # - the reference waypoints to be followed by the ego vehicle
        rospy.Subscriber('/base_waypoints', Lane, self.base_waypoints_cb)
        # - the current pose of the ego vehicle
        rospy.Subscriber('/current_pose', PoseStamped, self.current_pose_cb)
        # - the current twist of the ego vehicle
        rospy.Subscriber('/current_velocity', TwistStamped,
                         self.current_velocity_cb)
        # - the location of all traffic ligths on the map
        rospy.Subscriber('/vehicle/traffic_lights', TrafficLightArray,
                         self.traffic_lights_cb)
        # - the images taken from the car's camera
        rospy.Subscriber('/image_color', Image, self.image_color_cb)

        # - the /traffic_waypoint publisher, which will provide the waypoint
        #   index of the nearest red traffic light in front of the ego vehicle
        self.traffic_light_pub = rospy.Publisher('/traffic_waypoint', Int32,
                                                 queue_size=1)

        # - the WaypointTree instance of the reference waypoints
        self.tree = None  # type: WaypointTree | None
        # - the ego vehicle's current pose
        self.current_pose = None  # type: PoseStamped | None
        # - the ego vehicle's current twist
        self.current_twist = None  # type: TwistStamped | None
        # - the list of traffic lights on the map
        self.traffic_lights = []  # type: list[TrafficLight]
        # - the current camera image
        self.current_camera_image = None
        # - the current traffic light state and location
        self.current_light_state = TrafficLight.UNKNOWN
        self.current_stopline_idx = UNKNOWN
        # - the traffic light classifier and required helper classes
        self.light_classifier = TLClassifier()
        self.bridge = CvBridge()
        self.listener = tf.TransformListener()
        # Load the traffic light config data
        self.config = yaml.load(rospy.get_param("/traffic_light_config"))
        # Get the stopline positions (list of xy coordinates)
        self.stoplines = self.config["stop_line_positions"]  # type: list[list]
        # Initialize the node's logger
        self.logger = Logger()

        rospy.spin()

    def current_pose_cb(self, current_pose):
        """ Store the current pose of the ego vehicle.

        Args:
            current_pose (PoseStamped)
        """
        self.current_pose = current_pose

    def current_velocity_cb(self, current_twist):
        """ Store the current velocity of the ego vehicle.

        Args:
            current_twist (TwistStamped)
        """
        self.current_twist = current_twist

    def base_waypoints_cb(self, reference_waypoints):
        """ Initialize the reference waypoints' WaypointTree.

        Args:
            reference_waypoints (Lane)
        """

        # Since the /waypoint_loader node keeps publishing the same reference
        # waypoints all the time, they need to be stored only once.
        if self.tree is None:
            self.tree = WaypointTree(reference_waypoints)

    def traffic_lights_cb(self, traffic_ligths_array):
        """ Store the traffic light array.

        Args:
            traffic_ligths_array (TrafficLightArray)
        """
        self.traffic_lights = traffic_ligths_array.lights

    def image_color_cb(self, image):
        """ Process incoming camera image and publish stop line waypoint index.

        Identifies red lights in the incoming camera image and publishes the
        index of the waypoint closest to the red light's stop line to
        /traffic_waypoint

        Args:
            image (Image):
                image from car-mounted camera
        """
        self.logger.reset()
        self.current_camera_image = image
        stopline_idx, light_state = self.detect_next_traffic_light()

        if self.current_stopline_idx != stopline_idx:
            self.current_stopline_idx = stopline_idx
            if stopline_idx != UNKNOWN:
                self.logger.warn("Detected traffic light.")

        # If the current light state changed
        if self.current_light_state != light_state:
            self.logger.warn("The light's color changed to %s",
                             LIGHT_STATES_LOG[light_state])
            self.current_light_state = light_state  # Store the new light state
        # If the current light state did not change
        else:
            self.logger.info("Light color: %s", LIGHT_STATES_LOG[light_state])

        # Only stop lines of red lights will be handled
        if light_state == TrafficLight.RED:
            self.traffic_light_pub.publish(Int32(stopline_idx))
        # On all other cases the vehicle shall just keep moving
        else:
            self.traffic_light_pub.publish(Int32(UNKNOWN))

    def detect_next_traffic_light(self):
        """ Determine the closest visible traffic light and its color state.

        Returns:
            tuple of the stop line index and the traffic light color id. If no
             traffic light was detected, returns (UNKNOWN, TrafficLight.UNKNOWN)
        """

        closest_light = None  # type: TrafficLight | None
        stopline_idx = None  # type: int | None

        # Begin the search if the required data is initialized
        if self.tree and self.current_pose and self.current_twist:
            num_lookahead = NUM_LOOKAHEAD
            # Index of closest reference waypoint to ego vehicle's current pose
            ego_idx = self.tree.get_closest_idx_from(self.current_pose)
            # Find closest upcoming traffic light within the lookahead distance
            for light, stopline_xy in zip(self.traffic_lights, self.stoplines):
                # Index of closest reference waypoint to current traffic light
                light_idx = self.tree.get_closest_idx_from(light.pose)
                # Number of waypoints beetween traffic light and ego vehicle
                num_waypoints = light_idx - ego_idx
                # If it is the closest traffic light ahead of the ego vehicle
                if 0 <= num_waypoints < num_lookahead:
                    # Store traffic light and its stopline as the closest ones
                    closest_light = light
                    stopline_idx = self.tree.get_closest_idx_from(stopline_xy)
                    # Update the lookahead distance
                    num_lookahead = num_waypoints

        # If a traffic light was found within the lookahead distance and an
        # image from the camera is available
        if closest_light and self.current_camera_image:
            # Convert the image message into opencv format
            cv_image = self.bridge.imgmsg_to_cv2(self.current_camera_image,
                                                 "bgr8")
            # Classify the traffic light state
            light_state = self.light_classifier.get_classification(cv_image)
            # Return the traffic light's reference waypoint index and its state
            return stopline_idx, light_state
        # If no traffic light was detected
        else:
            # Reset the classifier's buffer
            self.light_classifier.reset_buffer()
            # Return dummy values for the stop line location and light state
            return UNKNOWN, TrafficLight.UNKNOWN


if __name__ == '__main__':
    try:
        TLDetector()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start traffic node.')
