from heuristics.pso_for_scanning import PSOforScanning
from model.Optimization import Optimization
from model.Timer import ContextTimer
from model.model_definition.Cell import Cell
from model.model_definition.Instance import Instance, objective_value
from model.model_stages.Stage1 import findMinDistanceToBoundaryCells
from model.model_stages.Stage2 import initialLocation
from model.model_stages.Stage4 import *
import os
import glob

instance = Instance()
instance.readInstance(
    'C:/Users/ertug/OneDrive/Masaüstü/multi-agent-optimization-by-pso-and-cplex/model/model_input/Agents.csv')
optimization = Optimization()
"""
optimization.Stage1(instance)
optimization.Stage2(instance)
instance.Agents[0].setCurrCell(instance.findCellFromId(99))
optimization.Stage3(instance)
optimization.Stage3_part2(instance)
print(instance.communicationGraph())
print(instance.connectivity_matrix())


with ContextTimer("stage=%d" % 1):
    optimization.Stage1(instance)

with ContextTimer("stage=%d" % 2):
    optimization.Stage2(instance)
"""


def findRoute(instance, t):
    a_list = []
    for agent in instance.Agents:
        a_list.append(agent.getCurrCell().getID())

    instance.visitedCells[str(t)] = a_list


def agentCellList(ins):
    my_list = []
    for agent in ins.Agents:
        my_list.append(agent.getCurrCell().getID())

    return my_list


def swarm(instance, optimization):
    # solve optimization model of stage 1 and 2, run stage 3:
    optimization.Stage1(instance)
    optimization.Stage2(instance)
    optimization.Stage3(instance)
    print('Stage 4:')

    # initialize 'a_matrix' to be used in order to find coverage of cells:
    a_matrix = aMatrix(instance)

    # initialize 'sigma_matrix' :
    sigma_matrix = sigmaMatrix(instance)

    PSOforScanning(instance, MaxIt=100, nPop=1, w=1, wDamp=0.99, c1=2, c2=2, sigma_matrix=sigma_matrix,
                   a_matrix=a_matrix)


    t = 1



    with open(
            "C:/Users/ertug/OneDrive/Masaüstü/multi-agent-optimization-by-pso-and-cplex/model/model_output/visitedCell.txt",
            "w") as file:
        file.write(str(instance.visitedCells))


with ContextTimer("stage=%d" % 1):
    swarm(instance, optimization)
