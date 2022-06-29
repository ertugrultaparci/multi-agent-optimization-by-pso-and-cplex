import airsim
import time


# client = airsim.MultirotorClient()


def drone_state(client, drone):
    droneState = client.getMultirotorState(vehicle_name=drone.getName()).kinematics_estimated.position
    basePosition = drone.getBasePosition()
    x_val = droneState.x_val + float(basePosition[0])
    y_val = droneState.y_val + float(basePosition[1])
    z_val = droneState.z_val + float(basePosition[2])

    stateDict = {
        'x': x_val,
        'y': y_val,
        'z': z_val
    }
    return stateDict


def distanceX(client, drone1, drone2):
    drone1State = drone_state(client, drone1)
    drone1_x = float(drone1State['x'])

    drone2State = drone_state(client, drone2)
    drone2_x = float(drone2State['x'])

    dist = drone1_x - drone2_x

    return dist


def distanceY(client, drone1, drone2):
    drone1State = drone_state(client, drone1)
    drone1_y = float(drone1State['y'])

    drone2State = drone_state(client, drone2)
    drone2_y = float(drone2State['y'])

    dist = drone1_y - drone2_y

    return dist


def distanceZ(client, drone1, drone2):
    drone1State = drone_state(client, drone1)
    drone1_z = float(drone1State['z'])

    drone2State = drone_state(client, drone2)
    drone2_z = float(drone2State['z'])

    dist = drone1_z - drone2_z

    return dist


def matrixX(client, instance):
    n = len(instance.Agents)
    distance_matrix = [[0] * n for i in range(n)]

    r = 0

    for agent in instance.Agents:
        k = 0
        for agent2 in instance.Agents:
            if agent == agent2:
                distance_matrix[r][k] = 0
            else:
                distance_matrix[r][k] = distanceX(client, agent, agent2)
            k += 1
        r += 1

    return distance_matrix


def matrixY(client, instance):
    n = len(instance.Agents)
    distance_matrix = [[0] * n for i in range(n)]

    r = 0

    for agent in instance.Agents:
        k = 0
        for agent2 in instance.Agents:
            if agent == agent2:
                distance_matrix[r][k] = 0
            else:
                distance_matrix[r][k] = distanceY(client, agent, agent2)
            k += 1
        r += 1

    return distance_matrix


def matrixZ(client, instance):
    n = len(instance.Agents)
    distance_matrix = [[0] * n for i in range(n)]

    r = 0

    for agent in instance.Agents:
        k = 0
        for agent2 in instance.Agents:
            if agent == agent2:
                distance_matrix[r][k] = 0
            else:
                distance_matrix[r][k] = distanceZ(client, agent, agent2)
            k += 1
        r += 1

    return distance_matrix


def sum_2_dim(a_list):
    total = 0
    for i in range(len(a_list)):
        for j in range(len(a_list[0])):
            total = total + abs(a_list[i][j])
    return total


def smallerThan(target_matrix, k):
    res = 0
    for i in range(len(target_matrix)):
        for j in range(len(target_matrix[0])):
            if abs(target_matrix[i][j]) <= k:
                res = res + 1

    if res == len(target_matrix) * len(target_matrix[0]):
        return True
    else:
        return False


def errorfunc(client, instance, targetX, targetY, targetZ):
    currentX = matrixX(client, instance)
    currentY = matrixY(client, instance)
    currentZ = matrixZ(client, instance)

    errorX = [[0] * 4 for i in range(4)]
    errorY = [[0] * 4 for i in range(4)]
    errorZ = [[0] * 4 for i in range(4)]

    for i in range(4):
        for j in range(4):
            errorX[i][j] = targetX[i][j] - currentX[i][j]
            errorY[i][j] = targetY[i][j] - currentY[i][j]
            errorZ[i][j] = targetZ[i][j] - currentZ[i][j]

    return errorX, errorY, errorZ


def isMove(client, instance):
    motion = True

    targetX = [[0, -10, 10, -20], [10, 0, 0, -10], [10, 0, 0, -10], [20, 10, 10, 0]]
    targetY = [[0, 10, -10, 0], [-10, 0, -20, -10], [10, 20, 0, 10], [0, 10, -10, 0]]
    targetZ = [[0] * 4 for i in range(4)]

    errorX, errorY, errorZ = errorfunc(client, instance, targetX, targetY, targetZ)

    if smallerThan(errorX, 15) and smallerThan(errorY, 15):
        motion = False

    return motion


def pid_formation(client, instance):
    # Drones are in their initial cells!

    targetX = [[0, -10, 10, -20], [10, 0, 0, -10], [10, 0, 0, -10], [20, 10, 10, 0]]
    targetY = [[0, 10, -10, 0], [-10, 0, -20, -10], [10, 20, 0, 10], [0, 10, -10, 0]]
    targetZ = [[0] * 4 for i in range(4)]

    prev_errorX, prev_errorY, prev_errorZ = [[0] * 4 for i in range(4)], [[0] * 4 for i in range(4)], [[0] * 4 for i in
                                                                                                       range(4)]
    f_list_for_z = []
    f_list_for_formation = []

    while isMove(client, instance):
        curr_errorX, curr_errorY, curr_errorZ = errorfunc(client, instance, targetX, targetY, targetZ)

        kp = 100
        kd = 1

        for i in range(len(instance.Agents)):
            drone_vx = sum(curr_errorX[i]) * kp + kd * (sum(curr_errorX[i]) - sum(prev_errorX[i]))
            drone_vy = sum(curr_errorY[i]) * kp + kd * (sum(curr_errorY[i]) - sum(prev_errorY[i]))
            drone_vz = sum(curr_errorZ[i]) * kp + kd * (sum(curr_errorZ[i]) - sum(prev_errorZ[i]))

            f = client.moveByVelocityAsync(0, 0, 0, 0.2, vehicle_name=instance.Agents[i].getName())
            f_list_for_z.append(f)

            f = client.moveByVelocityAsync(drone_vx, drone_vy, drone_vz, 0.2, vehicle_name=instance.Agents[i].getName())
            f_list_for_formation.append(f)

        prev_errorX, prev_errorY, prev_errorZ = curr_errorX, curr_errorY, curr_errorZ

        for f in f_list_for_z:
            f.join()

        time.sleep(0.5)

        for f in f_list_for_formation:
            f.join()

        time.sleep(0.5)

        f_list_for_z.clear()
        f_list_for_formation.clear()

        for j in instance.Agents:
            if drone_state(client, j)['z'] > 0:
                f = client.moveByVelocityAsync(0, 0, -50, 5, vehicle_name=j.getName())
                f.join()
                time.sleep(1)

    f_list_state_stability = []

    for j in instance.Agents:
        f_list_state_stability.append(client.moveByVelocityAsync(0, 0, 0, 1, vehicle_name=j.getName()))

    for c in f_list_state_stability:
        c.join()

    time.sleep(1)
