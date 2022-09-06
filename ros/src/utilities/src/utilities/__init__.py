from utilities import *

PUBLISHING_FREQUENCY = 50  # [Hz], publishing frequency
NUM_LOOKAHEAD = 100  # number of waypoints to publish in ego vehicle's front
MAX_DECEL = -1.0  # [m/s**2], maximum deceleration used for reference wp update
UNKNOWN = -1  # flag for undefined stopline index
