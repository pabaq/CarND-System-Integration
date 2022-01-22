from styx_msgs.msg import TrafficLight

from utilities import *

PUBLISHING_FREQUENCY = 20  # [Hz], publishing frequency
NUM_LOOKAHEAD_PUB = 75  # number of waypoints to publish in ego vehicle's front
NUM_LOOKAHEAD_LIGHT = 50  # traffic light detection zone in number of waypoints
NUM_STOPLINE_THRES = 4  # come to stop this waypoint margin in front of stopline
MAX_DECEL = -1.0  # [m/s**2], maximum deceleration
UNKNOWN = -1  # flag for undefined stopline index

# Parameters for the longitudinal controller
KP = 0.3  # PID gains
KI = 0.1
KD = 0.0
GAS_DENSITY = 2.858  # [kg/gal]

# Parameters for the lateral controller
MIN_SPEED = 0.1  # [m/s]

# Low pass filter parameters
TAU = 0.5
TS = 0.02

# used for logging
LIGHT_STATES = {TrafficLight.UNKNOWN: "Unknown", TrafficLight.RED: "red",
                TrafficLight.YELLOW: "yellow", TrafficLight.GREEN: "green"}

