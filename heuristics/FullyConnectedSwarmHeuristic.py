from random import random

from heuristics.Bfs import graphForBFS
from model.model_definition.Cell import Cell


def heuristicForStage2(self):
    comm_graph = self.communicationGraph()

    C = self.connectivity_matrix(graphForBFS(comm_graph))
    D = self.distanceMatrixOfAgents(comm_graph)

    # print(C)

    r_sum_list = []

    for agent in self.Agents:
        index = agent.getName().replace('Drone', '')
        # for agent2 in self.Agents:
        # index2 = agent2.getName().replace('Drone', '')
        if self.isConnected(C):
            return
        else:
            r_sum = sum(list(C[index]))
            r_sum = r_sum + sum(list(C.transpose()[index]))
            r_sum_list.append((index, r_sum)) if (index, r_sum) not in r_sum_list else r_sum_list

    # print(r_sum_list)

    r_sums = []
    r_indis = []
    for x, y in r_sum_list:
        r_indis.append(x)
        r_sums.append(y)

    min_value = min(r_sums)

    min_list = []
    for val in r_sum_list:
        if val[1] == min_value:
            min_list.append(val)

    if len(min_list) < 2:
        agent_name = 'Drone' + min_list[0][0]
        self.relocation(agent_name)
    else:
        sum_list = []
        for agent in min_list:
            summation = sum(D.transpose().loc[agent[0]])
            sum_list.append(summation)
        agent_name = 'Drone' + min_list[sum_list.index(max(sum_list))][0]
        self.relocation(agent_name)

    print('heuristic runs')
    self.heuristicForStage2()


def relocation(self, agent_name):
    agent_id = 0
    new_cell = Cell()

    for a in self.Agents:
        if agent_name == a.getName():
            agent_id = a.getCurrCell().getID()

    previousID = self.whichBoundaryCellIndex(agent_id)

    if previousID == len(self.BoundaryCells) - 1:
        new_cell = self.BoundaryCells[previousID - 1]
    else:
        new_ID_index = random.choice([-1, 1])
        new_cell = self.BoundaryCells[previousID + new_ID_index]

    new_cell.printID()

    for agent in self.Agents:
        if agent.getName() == agent_name:
            agent.setCurrCell(new_cell)

def isConnected(self, dataframe):
    if sum(list(dataframe.sum())) == dataframe.size:
        return True
    else:
        return False