import json
import math
from itertools import permutations, product

import pandas as pd

from model.model_definition.Cell import Cell
from model.model_definition.Instance import variableDivision
from model.model_stages.Stage4 import dictOfCoveredCells


def initializeInstanceForChosenDrones(instance, filename):
    chosen_list = chosen_drones(instance, filename)

    instance.TypeUAV.clear()
    instance.TypeAgents.clear()
    instance.UAVs.clear()
    instance.UGVs.clear()
    instance.Agents.clear()
    instance.TypeAgentIndices.clear()
    instance.AgentIndexSet.clear()

    for i in chosen_list:
        instance.TypeAgents.append(i.getType()) if i.getType() not in instance.TypeAgents else instance.TypeAgents
        instance.Agents.append(i)
        if i.getType() != 0:
            instance.TypeUAV.append(i.getType()) if i.getType() not in instance.TypeUAV else instance.TypeUAV
            instance.UAVs.append(i) if i not in instance.UAVs else instance.UAVs
        else:
            instance.UGVs.append(i)

    # to find how many drones we have for each type:
    type_0 = []
    type_1 = []
    type_2 = []

    for agent in instance.Agents:
        if agent.type == 0:
            type_0.append(agent.indis)
        else:
            if agent.type == 1:
                type_1.append(agent.indis)
            elif agent.type == 2:
                type_2.append(agent.indis)

    instance.TypeAgentIndices['0'] = list(range(0, len(type_0)))
    instance.TypeAgentIndices['1'] = list(range(0, len(type_1)))
    instance.TypeAgentIndices['2'] = list(range(0, len(type_2)))

    instance.AgentIndexSet.append(instance.TypeAgentIndices.get('0'))
    instance.AgentIndexSet.append(instance.TypeAgentIndices.get('1'))
    instance.AgentIndexSet.append(instance.TypeAgentIndices.get('2'))

    instance.numUAVTypes = len(instance.TypeUAV)
    instance.numTypelAgent = len(instance.TypeAgents)


def chosen_drones(instance, filename):
    chosen_list = []
    with open(filename) as json_file:
        data = json.load(json_file)

        for var in data['CPLEXSolution']['variables']:
            var_list = variableDivision(var['name'])

            for agent in instance.Agents:
                if var_list[0] == agent.getType() and var_list[1] == agent.getIndis():
                    chosen_list.append(agent)

    return chosen_list


def edgeLengthDetermination(instance):
    scanTime = []

    for agent in instance.Agents:
        scanTime.append(agent.scanTime)

    worst_scan_time = max(scanTime)
    A = (instance.periodLength * instance.planningHorizon) / worst_scan_time

    instance.scanPercentage = (instance.width * instance.height) / A

    sens_range = []

    for agent in instance.Agents:
        sens_range.append(math.sqrt((instance.scanPercentage * instance.periodLength) / (math.pi * agent.scanTime)))

    edgeLength = math.sqrt(2) * min(sens_range)

    instance.edgeLength = edgeLength

    return edgeLength


def initializeCells(instance):
    print('The edgeLength is ', instance.edgeLength)

    instance.nCol = math.ceil(instance.width / instance.edgeLength)
    instance.nRow = math.ceil(instance.height / instance.edgeLength)
    print('width:', instance.width)
    print('height:', instance.height)

    instance.nCells = instance.nCol * instance.nRow

    print(f'(nCol, nRow) = {(instance.nCol, instance.nRow)}')

    for j in range(instance.nRow):
        if j % 2 == 0:
            for i in range(instance.nCol):
                currCell = Cell()
                currCell.Id = j * instance.nCol + (1 - j % 2) * i + (j % 2) * (instance.nCol - i - 1)
                currCell.row = j
                currCell.col = i
                currCell.edgeLength = instance.edgeLength
                currCell.center = [(1 - j % 2) * (instance.edgeLength / 2.0 + instance.edgeLength * i)
                                   + (j % 2) * (
                                           instance.nCol * instance.edgeLength - instance.edgeLength / 2.0 - instance.edgeLength * (
                                           instance.nCol - i - 1))
                    , instance.edgeLength / 2.0 + instance.edgeLength * j]

                currCell.corner1 = [currCell.center[0] - instance.edgeLength / 2.0,
                                    currCell.center[1] - instance.edgeLength / 2.0]
                currCell.corner2 = [currCell.center[0] + instance.edgeLength / 2.0,
                                    currCell.center[1] - instance.edgeLength / 2.0]
                currCell.corner3 = [currCell.center[0] + instance.edgeLength / 2.0,
                                    currCell.center[1] + instance.edgeLength / 2.0]
                currCell.corner4 = [currCell.center[0] - instance.edgeLength / 2.0,
                                    currCell.center[1] + instance.edgeLength / 2.0]

                instance.Cells.append(currCell)
        else:
            for i in range(instance.nCol - 1, -1, -1):
                currCell = Cell()
                currCell.Id = j * instance.nCol + (1 - j % 2) * i + (j % 2) * (instance.nCol - i - 1)
                currCell.row = j
                currCell.col = i
                currCell.center = [(1 - j % 2) * (instance.edgeLength / 2.0 + instance.edgeLength * i)
                                   + (j % 2) * (
                                           instance.nCol * instance.edgeLength - instance.edgeLength / 2.0 - instance.edgeLength * (
                                           instance.nCol - i - 1))
                    , instance.edgeLength / 2.0 + instance.edgeLength * j]

                currCell.corner1 = [currCell.center[0] - instance.edgeLength / 2.0,
                                    currCell.center[1] - instance.edgeLength / 2.0]
                currCell.corner2 = [currCell.center[0] + instance.edgeLength / 2.0,
                                    currCell.center[1] - instance.edgeLength / 2.0]
                currCell.corner3 = [currCell.center[0] + instance.edgeLength / 2.0,
                                    currCell.center[1] + instance.edgeLength / 2.0]
                currCell.corner4 = [currCell.center[0] - instance.edgeLength / 2.0,
                                    currCell.center[1] + instance.edgeLength / 2.0]

                instance.Cells.append(currCell)

    dictOfCoveredCells(instance)


def initializeBoundaryCells(instance):
    for i in range(instance.nCol - 1):
        instance.BoundaryCells.append(instance.Cells[i])
        instance.Cells[i].setBoundary()

    for i in range(instance.nRow - 1):
        instance.BoundaryCells.append(instance.Cells[((i + 1) * instance.nCol) - 1])
        instance.Cells[((i + 1) * instance.nCol) - 1].setBoundary()
        instance.BoundaryCells.append(instance.Cells[(i + 1) * instance.nCol])
        instance.Cells[(i + 1) * instance.nCol].setBoundary()

    for i in range(instance.nCol * (instance.nRow - 1) + 1, len(instance.Cells)):
        instance.BoundaryCells.append(instance.Cells[i])
        instance.Cells[i].setBoundary()


def distanceBtwBaseLocationAndBoundaryCells(instance):
    distance_matrix = [[0 for col in range(len(instance.BoundaryCells))] for row in range(len(instance.Agents))]

    r = 0

    for agent in instance.Agents:
        k = 0
        for cell in instance.BoundaryCells:
            dist = math.sqrt(math.pow(agent.basePosition[0] - cell.center[0], 2) + math.pow(
                agent.basePosition[1] - cell.center[1], 2))
            distance_matrix[r][k] = dist
            k += 1
        r += 1

    return distance_matrix


def matrixWithinCommRangeCellJAgentTypeL(instance):
    agent_list = list(product(instance.TypeAgents, repeat=2))

    R_cells = range(0, len(instance.Cells))
    id_cells = [j for j in R_cells]

    cell_list = list(permutations(id_cells, 2))

    b_matrix = [[0 for col in range(len(cell_list))] for row in range(len(agent_list))]

    i = 0
    for agent in agent_list:
        k = 0
        for cell in cell_list:
            distance = instance.distanceBtwTwoCells(cell[0], cell[1])
            r_comm = instance.getRCommOfGivenType(agent[0])
            if distance <= r_comm:
                b_matrix[i][k] = 1
            else:
                b_matrix[i][k] = 0
            k += 1
        i += 1

    df = pd.DataFrame(data=b_matrix, index=agent_list, columns=cell_list)

    return df


def initialLocation(instance, filename):
    with open(filename) as json_file:
        data = json.load(json_file)
        for var in data['CPLEXSolution']['variables']:
            var_list = variableDivision(var['name'])

            for agent in instance.Agents:
                if var_list[0] == agent.getType() and var_list[1] == agent.getIndis():
                    print(f' z_{agent.getType()}_{agent.getIndis()}_{instance.BoundaryCells[var_list[2]].getID()}_0')
                    agent.setCurrCell(instance.BoundaryCells[var_list[2]])


def cellCenterListXLoc(instance):
    for cell in instance.Cells:
        if not cell.isDenied:
            instance.cellCenter[cell.getID()] = cell.getCenter()[0], cell.getCenter()[1]

