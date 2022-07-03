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

    return a_list


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

    # check coverage:
    a = findScannedCells(instance, a_matrix)
    print('which cells scanned in this iteration:', a)
    print('which cells completed:', instance.coveredCells)

    # find visited cells to use in 'Airsim Simulation'
    findRoute(instance, t=0)




    t = 1
    while not scanIsDone(instance):

        optimization.Stage4(instance, t, sigma_matrix)

        # find the path of the solution of stage 4:
        filename_stage4solution = 'C:/Users/ertug/OneDrive/Masaüstü/multi-agent-optimization-by-pso-and-cplex/model/model_output/stage4solution.json'

        # change agents location according to the solution of stage 4:
        stage4Solution(instance, filename_stage4solution)

        # find visited cells to use in 'Airsim Simulation'
        findRoute(instance, t)

        # check coverage:
        a = findScannedCells(instance, a_matrix)
        print('which cells scanned in this iteration:', a)
        print('which cells completed:', instance.coveredCells)

        if objective_value(filename_stage4solution) == len(instance.Agents) * 100:
            # increase time by 1
            t += 1

            # change location of Agents to not scanned cells:
            changeLoc(instance)

            # find visited cells to use in 'Airsim Simulation'
            a = findRoute(instance, t)

            print("cells in this iteration ", t, ":",   a)

            # check coverage:
            a = findScannedCells(instance, a_matrix)
            print('which cells scanned in this iteration:', a)
            print('which cells completed:', instance.coveredCells)

        t += 1

    optimization.L = t - 1
    print("Lifetime: ", optimization.L)
    # print("Length:", len(instance.coveredCells), 'should equal to = ', len(instance.Cells) - len(instance.DeniedCells))

    # create a file named 'visitedCell.txt' to hold the route of agents according to stage 4:
    with open(
            "C:/Users/ertug/OneDrive/Masaüstü/multi-agent-optimization-by-pso-and-cplex/model/model_output/visitedCell.txt",
            "w") as file:
        file.write(str(instance.visitedCells))


#with ContextTimer("stage=%d" % 1):
    #swarm(instance, optimization)
