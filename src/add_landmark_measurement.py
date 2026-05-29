import math
import numpy as np
import gtsam
from gtsam.symbol_shorthand import L, X

PRIOR_NOISE = gtsam.noiseModel.Diagonal.Sigmas(np.array([0.1, 0.1, 0.05]))  # (x, y, theta)
ODOMETRY_NOISE = gtsam.noiseModel.Diagonal.Sigmas(np.array([0.2, 0.2, 0.1]))  # (dx, dy, dtheta)
MEASUREMENT_NOISE = gtsam.noiseModel.Diagonal.Sigmas(np.array([0.05, 0.1]))  # (bearing, range)

def add_landmark_measurement(graph, initial_estimate, result):
    # Determine the correct rotation (bearing) and distance from X(4) to L(2) 
    rotation = math.degrees(math.atan2(result.atPoint2(L(2))[1] - result.atPose2(X(4)).y(), result.atPoint2(L(2))[0] - result.atPose2(X(4)).x()) - result.atPose2(X(4)).theta())
    distance = math.sqrt((result.atPoint2(L(2))[0] - result.atPose2(X(4)).x())**2 + (result.atPoint2(L(2))[1] - result.atPose2(X(4)).y())**2)

    graph.add(gtsam.BearingRangeFactor2D(X(4), L(2), gtsam.Rot2.fromDegrees(rotation), distance, MEASUREMENT_NOISE))
    return graph