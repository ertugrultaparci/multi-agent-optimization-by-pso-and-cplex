import math
from numpy import ones, vstack
from numpy.linalg import lstsq


def findMinDistanceToBoundaryCells(instance, agent):
    basePosition = agent.getBasePosition()
    x = basePosition[0]
    y = basePosition[1]

    distance_matrix = []

    line1 = [(0, 0), (instance.width, 0)]
    line2 = [(instance.width, 0), (instance.width, instance.height)]
    line3 = [(0, 0), (0, instance.height)]
    line4 = [(0, instance.height), (instance.width, instance.height)]

    # distance 1:

    line1_x_coords, line1_y_coords = zip(*line1)
    A1 = vstack([line1_x_coords, ones(len(line1_x_coords))]).T
    m1, c1 = lstsq(A1, line1_y_coords, rcond=None)[0]

    denominator1 = math.sqrt(1 + math.pow(m1, 2))

    distance1 = abs(m1 * x + y * -1 + c1) / denominator1

    # distance 2:

    line2_x_coords, line2_y_coords = zip(*line2)
    A2 = vstack([line2_x_coords, ones(len(line2_x_coords))]).T
    m2, c2 = lstsq(A2, line2_y_coords, rcond=None)[0]

    denominator2 = math.sqrt(1 + math.pow(m2, 2))

    distance2 = abs(m2 * x + y * -1 + c2) / denominator2

    # distance 3:

    line3_x_coords, line3_y_coords = zip(*line3)
    A3 = vstack([line3_x_coords, ones(len(line3_x_coords))]).T
    m3, c3 = lstsq(A3, line3_y_coords, rcond=None)[0]

    denominator3 = math.sqrt(1 + math.pow(m3, 2))

    distance3 = abs(m3 * x + y * -1 + c3) / denominator3

    # distance 4:

    line4_x_coords, line4_y_coords = zip(*line4)
    A4 = vstack([line4_x_coords, ones(len(line4_x_coords))]).T
    m4, c4 = lstsq(A4, line4_y_coords, rcond=None)[0]

    denominator4 = math.sqrt(1 + math.pow(m4, 2))

    distance4 = abs(m4 * x + y * -1 + c4) / denominator4

    print('distance 1:', distance1)
    print('distance 2:', distance2)
    print('distance 3:', distance3)
    print('distance 4:', distance4)

    return min(distance1, distance2, distance3, distance4)
