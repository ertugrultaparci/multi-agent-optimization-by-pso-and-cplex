import formation
import time


def takeoff(client, instance, f_list):
    for j in instance.Agents:
        client.enableApiControl(True, j.getName())
        client.armDisarm(True, j.getName())
        f_list.append(client.takeoffAsync(vehicle_name=j.getName()))

    for c in f_list:
        c.join()

    time.sleep(1)

    f_list.clear()


def waitASecond(client, instance, f_list):
    for j in instance.Agents:
        f_list.append(client.moveByVelocityAsync(0, 0, 0, 1, vehicle_name=j.getName()))

    for f in f_list:
        f.join()

    time.sleep(1)

    f_list.clear()


def moveUp(client, instance, f_list):
    for j in instance.Agents:
        f_list.append(client.moveByVelocityAsync(0, 0, -50, 10, vehicle_name=j.getName()))

    for f in f_list:
        f.join()

    time.sleep(1)

    f_list.clear()


def moveInitialCell(client, instance, f_list, t):
    for j in instance.Agents:
        agent_cell_center = j.getCurrCell().getCenter()
        v_x = (agent_cell_center[0] - formation.drone_state(client, j)['x']) / t
        v_y = (agent_cell_center[1] - formation.drone_state(client, j)['y']) / t
        f_list.append(client.moveByVelocityAsync(v_x, v_y, 0, t, vehicle_name=j.getName()))

    for f in f_list:
        f.join()

    time.sleep(1)

    f_list.clear()


def moveACell(client, instance, f_list, t, cell_index):
    for j in instance.Agents:
        v_x = (instance.Cells[cell_index].getCenter()[0] - formation.drone_state(client, j)['x']) / t
        v_y = (instance.Cells[cell_index].getCenter()[1] - formation.drone_state(client, j)['y']) / t
        f_list.append(client.moveByVelocityAsync(v_x, v_y, 0, t, vehicle_name=j.getName()))

    for f in f_list:
        f.join()

    time.sleep(1)

    f_list.clear()


def moveACellSingleDrone(client, instance, t, cell_index, agent, z):
    v_x = (instance.Cells[cell_index].getCenter()[0] - formation.drone_state(client, agent)['x']) / t
    v_y = (instance.Cells[cell_index].getCenter()[1] - formation.drone_state(client, agent)['y']) / t
    return client.moveByVelocityZAsync(v_x, v_y, z, t, vehicle_name=agent.getName())


def moveForIteration(client, instance, t, f_list, cell_list):
    for i in range(len(instance.Agents)):
        f_list.append(moveACellSingleDrone(client, instance, t, cell_index=cell_list[i], agent=instance.Agents[i], z=-20-i))
        print(client.getMultirotorState(vehicle_name=instance.Agents[i].getName()).kinematics_estimated.position.z_val)

    for f in f_list:
        f.join()

    time.sleep(1)

    f_list.clear()


def scanArea(client, instance, f_list, t):
    row = instance.nRow

    for i in range(1, instance.nCol + 1):
        formation.pid_formation(client, instance)

        waitASecond(client, instance, f_list)

        moveACell(client, instance, f_list, t, ((i - 1) * row))

        waitASecond(client, instance, f_list)

        formation.pid_formation(client, instance)

        waitASecond(client, instance, f_list)

        moveACell(client, instance, f_list, t, (i * row - 1))

        waitASecond(client, instance, f_list)


def moveBasePosition(client, instance, f_list, t):
    for j in instance.Agents:
        agent_base_position = j.getBasePosition()
        v_x = (agent_base_position[0] - formation.drone_state(client, j)['x']) / t
        v_y = (agent_base_position[1] - formation.drone_state(client, j)['y']) / t
        f_list.append(client.moveByVelocityAsync(v_x, v_y, 0, t, vehicle_name=j.getName()))

    for f in f_list:
        f.join()

    time.sleep(1)

    f_list.clear()

    landing(client, instance, f_list)

    """
    for j in instance.Agents:
        agent_base_position = j.basePosition
        v_z = (agent_base_position[2] - formation.drone_state(client, j)['z']) / t
        f_list.append(client.moveByVelocityAsync(0, 0, v_z, t, vehicle_name=j.getName()))

    for f in f_list:
        f.join()

    time.sleep(1)

    f_list.clear()
    
    """


def landing(client, instance, f_list):
    for j in instance.Agents:
        f_list.append(client.landAsync(5, vehicle_name=j.getName()))

    for c in f_list:
        c.join()

    time.sleep(1)

    for j in instance.Agents:
        client.armDisarm(False, vehicle_name=j.getName())
        client.enableApiControl(False, vehicle_name=j.getName())

    f_list.clear()
