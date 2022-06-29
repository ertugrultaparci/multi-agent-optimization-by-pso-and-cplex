import json
import math
from itertools import permutations

import pandas as pd

from model.model_definition.Instance import variableDivision


def isPassingOverDeniedCells(instance, cell1_id, cell2_id):
    cell1 = instance.findCellFromId(cell1_id)
    cell2 = instance.findCellFromId(cell2_id)

    res = False
    cell1_x = int(cell1.getCenter()[0])
    cell2_x = int(cell2.getCenter()[0])
    cell1_y = int(cell1.getCenter()[1])
    cell2_y = int(cell2.getCenter()[1])

    x1 = 0
    x2 = 0
    y1 = 0
    y2 = 0

    if cell1_x <= cell2_x:
        x1 = cell1_x
        x2 = cell2_x
    elif cell1_x > cell2_x:
        x1 = cell2_x
        x2 = cell1_x

    if cell1_y <= cell2_y:
        y1 = cell1_y
        y2 = cell2_y
    elif cell1_y > cell2_y:
        y1 = cell2_y
        y2 = cell1_y

    for d in instance.DeniedCells:
        for x in range(x1, x2 + 1):
            for y in range(y1, y2 + 1):
                if d.isInCell(x, y):
                    res = True
                    break
    return res


def sigmaMatrix(instance):
    agent_id_list = []
    for agent in instance.Agents:
        agent_id_list.append(agent.getType()) if agent.getType() not in agent_id_list else agent_id_list

    R_cells = range(0, len(instance.Cells))
    id_cells = [j for j in R_cells]

    cell_list = list(permutations(id_cells, 2))

    sigma_matrix = [[0 for col in range(len(cell_list))] for row in range(len(agent_id_list))]

    i = 0
    for t in agent_id_list:
        k = 0
        for cell in cell_list:
            v_real = instance.distanceBtwTwoCells(cell[0], cell[1]) / instance.periodLength
            if instance.getVMinFromAgentType(t) <= v_real <= instance.getVMaxFromAgentType(t) and (
                    not instance.isDeniedCell(cell[0])) and (not instance.isDeniedCell(cell[1])) and (
                    not isPassingOverDeniedCells(instance, cell[0], cell[1])):
                sigma_matrix[i][k] = 1
            else:
                sigma_matrix[i][k] = 0
            k += 1
        i += 1

    df = pd.DataFrame(data=sigma_matrix, index=agent_id_list, columns=cell_list)

    return df


def aMatrix(instance):
    agent_id_list = []
    for agent in instance.Agents:
        agent_id_list.append(agent.getType()) if agent.getType() not in agent_id_list else agent_id_list

    R_cells = range(0, len(instance.Cells))
    id_cells = [j for j in R_cells]

    cell_list = list(permutations(id_cells, 2))

    a_matrix = [[0 for col in range(len(cell_list))] for row in range(len(agent_id_list))]

    i = 0
    for t in agent_id_list:
        k = 0
        r_sense = instance.getRSenseOfGivenType(t)
        for cell in cell_list:
            cell_1 = instance.findCellFromId(cell[0])
            cell_2 = instance.findCellFromId(cell[1])
            t1 = math.sqrt(
                math.pow(cell_1.center[0] - cell_2.corner1[0], 2) + math.pow(cell_1.center[1] - cell_2.corner1[1],
                                                                             2))
            t2 = math.sqrt(
                math.pow(cell_1.center[0] - cell_2.corner2[0], 2) + math.pow(cell_1.center[1] - cell_2.corner2[1],
                                                                             2))
            t3 = math.sqrt(
                math.pow(cell_1.center[0] - cell_2.corner3[0], 2) + math.pow(cell_1.center[1] - cell_2.corner3[1],
                                                                             2))
            t4 = math.sqrt(
                math.pow(cell_1.center[0] - cell_2.corner4[0], 2) + math.pow(cell_1.center[1] - cell_2.corner4[1],
                                                                             2))
            if t1 <= r_sense and t2 <= r_sense and t3 <= r_sense and t4 <= r_sense and not cell_1.isDenied and not cell_2.isDenied:
                a_matrix[i][k] = 1
            else:
                a_matrix[i][k] = 0
            k += 1
        i += 1

    df = pd.DataFrame(data=a_matrix, index=agent_id_list, columns=cell_list)

    return df


def dictOfCoveredCells(instance):
    for cell in instance.Cells:
        instance.dictOfCoverage[cell.getID()] = 0


def findScannedCells(instance, a_matrix):
    whichCellScanned = []
    for k, v in instance.dictOfCoverage.items():
        for agent in instance.Agents:
            if k == agent.getCurrCell().getID():
                instance.dictOfCoverage[k] = v + 1
                whichCellScanned.append(k) if k not in whichCellScanned else whichCellScanned
            if k != agent.getCurrCell().getID():
                pair = (agent.getCurrCell().getID(), k)
                index = a_matrix.loc[[agent.getType()], [pair]]
                a_value = index[pair].iloc[0]
                if a_value == 1:
                    instance.dictOfCoverage[k] = v + 1
                    whichCellScanned.append(k) if k not in whichCellScanned else whichCellScanned

    for k, v in instance.dictOfCoverage.items():
        if v >= instance.findCellFromId(k).coverageRequirement:
            instance.coveredCells.append(k) if k not in instance.coveredCells else instance.coveredCells

    instance.coveredCells.sort()
    return whichCellScanned


def findNotScannedCells(instance):
    my_list = []
    for k, v in instance.dictOfCoverage.items():
        if v < 2:
            my_list.append(k)
    return my_list


def scanIsDone(instance):
    n = 0
    for k, v in instance.dictOfCoverage.items():
        if v >= 2:
            n += 1
    if n == len(instance.Cells) - len(instance.DeniedCells):
        isDone = True
        return isDone


def stage4Solution(instance, filename):
    chosen_cell = []
    with open(filename) as json_file:
        data = json.load(json_file)

        for var in data['CPLEXSolution']['variables']:
            var_list = variableDivision(var['name'])
            chosen_cell.append(var_list)

    for i in range(len(instance.Agents)):
        if chosen_cell[i][0] == instance.Agents[i].getType() and chosen_cell[i][1] == instance.Agents[i].getIndis():
            instance.Agents[i].setCurrCell(instance.findCellFromId(chosen_cell[i][2]))

    instance.Route.append([chosen_cell])


def changeLocationToNotScannedCell(instance):
    min = 10
    id = None
    for k, v in instance.dictOfCoverage.items():
        if v < min:
            min = v

    for k, v in instance.dictOfCoverage.items():
        if v == min:
            id = k
            break

    if id > len(instance.Cells) - len(instance.Agents):
        id -= len(instance.Agents)

    if id > len(instance.Agents):
        id -= len(instance.Agents)
    elif id < len(instance.Agents):
        id += len(instance.Agents)

    while instance.isDeniedCell(id):
        id += 1
        if id > len(instance.Agents):
            id -= len(instance.Agents)
        elif id < len(instance.Agents):
            id += len(instance.Agents)

    for i in range(len(instance.Agents)):
        instance.Agents[i].setCurrCell(instance.findCellFromId(id + i))
