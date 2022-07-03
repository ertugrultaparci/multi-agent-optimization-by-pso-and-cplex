import math
import random
import numpy as np

from model.model_stages.Stage2 import cellCenterListXLoc
from model.model_stages.Stage4 import sigmaMatrix, findScannedCells, changeLoc


def PSOforScanning(instance, MaxIt, sigma_matrix, a_matrix, cost_func):
    # cost functions:
    cost_function = instance.cost_func_for_stage4_pso

    nVar = len(instance.Agents)  # number of unknown variables

    # PARAMETERS OF PSO
    MaxIt = MaxIt  # maximum number of Iterations
    nPop = 1  # swarm size (population size)

    # the following coefficient is stated as such in general PSO implementation:
    w = 1  # inertia coefficient
    wDamp = 0.99  # damping ratio of inertia coefficient

    c1 = 2  # personal acceleration coefficient
    c2 = 2 # social (global) coefficient

    # INITIALIZATION
    class Sol:
        def __init__(self, BestPosition, BestCost):
            self.BestPosition = BestPosition
            self.BestCost = BestCost

    # Initialize Global Best:
    glob = Sol
    glob.BestCost = float('inf')
    glob.BestPosition = None

    # Array to Hold Best Cost Value on Each Iteration:
    BestCosts = [0] * MaxIt

    class Particle(Sol):
        def __init__(self, BestPosition, BestCost):
            self.Velocity = []
            self.Position = []
            self.Cost = []
            self.p_list = []
            self.cost_list = []
            Sol.__init__(self, BestPosition, BestCost)

        def initializationOfParticle(self):
            for i in range(nPop):

                p = Particle([], [])

                # initialize initial cells as initial position:
                position_list = []

                for agent in instance.Agents:
                    position_list.append(agent.getCurrCell().getCenter())

                p.Position = position_list
                self.p_list.append(p)

            # initialize velocity
            for particle in self.p_list:
                particle.Velocity = [(0, 0) for i in range(nVar)]

            # evaluation
            for particle in self.p_list:
                particle.Cost = cost_function()
                self.cost_list.append(particle.Cost)

                # update the personal best:
                particle.BestPosition = particle.Position
                particle.BestCost = particle.Cost

                # update global best:
                if particle.BestCost < glob.BestCost:
                    cell = [agent.getCurrCell().getID() for agent in instance.Agents]
                    glob_cell = [a + 1 for a in cell]
                    glob.BestPosition = [instance.findCellFromId(cell).getCenter() for cell in glob_cell]

                    for k in range(len(instance.Agents)):
                        instance.Agents[k].setCurrCell(instance.findCellFromId(glob_cell[k]))

                    glob.BestCost = cost_function()

                    for k in range(len(instance.Agents)):
                        instance.Agents[k].setCurrCell(instance.findCellFromId(cell[k]))

        def getPList(self):
            c = []
            for a in self.p_list:
                c.append(a)
            return c

    p = Particle([], [])
    p.initializationOfParticle()

    # MAIN LOOP of PSO

    for i in range(MaxIt):
        for particle in p.getPList():

            # update velocity:
            list_rx = [(random.uniform(0, 1), random.uniform(0, 1)) for i in range(nVar)]
            c1_list = [(c1, c1) for i in range(nVar)]
            w_list = [(w, w) for i in range(nVar)]
            v1 = calculateOperation(particle.Velocity, w_list, "*")

            v2_3 = calculateOperation(particle.BestPosition, particle.Position, '-')
            v2_2 = calculateOperation(list_rx, v2_3, '*')
            v2 = calculateOperation(c1_list, v2_2, '*')

            list_rx = [(random.uniform(0, 1), random.uniform(0, 1)) for i in range(nVar)]
            c2_list = [(c2, c2) for i in range(nVar)]
            v3_3 = calculateOperation(glob.BestPosition, particle.Position, "-")
            v3_2 = calculateOperation(list_rx, v3_3, '*')
            v3 = calculateOperation(c2_list, v3_2, '*')

            particle.Velocity = calculateOperation(calculateOperation(v1, v2, "+"), v3, "+")

            # hold previous position:
            previous_position = findAgentCells(instance)

            # decrease coverage requirement of initial cells:
            # check coverage:
            a = findScannedCells(instance, a_matrix)
            print('which cells scanned in this iteration:', a)
            print('which cells completed:', instance.coveredCells)

            t=0
            # find visited cells to use in 'Airsim Simulation'
            findRoute(instance, t=0)

            # update position
            particle.Position = calculateOperation(particle.Position, particle.Velocity, "+")

            # find reachable sets:
            N_set = findNSet(instance, sigma_matrix=sigma_matrix)
            print('Nset', N_set)

            print('initial particle position', particle.Position)
            findNewCells(instance, particle.Position, N_set)

            print('final particle position', particle.Position)
            print(findCellid(instance, particle.Position))

            for agent in instance.Agents:
                print(f'agent type: {agent.getType()}, indis: {agent.getIndis()}, cell: {agent.getCurrCell().getID()}')

            print(instance.connectivity_matrix())

            # check coverage:
            a = findScannedCells(instance, a_matrix)
            print('which cells scanned in this iteration:', a)
            print('which cells completed:', instance.coveredCells)

            # find visited cells to use in 'Airsim Simulation'
            findRoute(instance, t=0)

            particle.Position = findPositon(instance)

            final_position = findAgentCells(instance)
            print('final pos:', final_position)

            if previous_position == final_position and not all(x >= instance.Cells[0].coverageRequirement for x in instance.dictOfCoverage.values()):
                changeLoc(instance)

                cellCenterListXLoc(instance)

                # find visited cells to use in 'Airsim Simulation'
                findRoute(instance, t)

                # check coverage:
                a = findScannedCells(instance, a_matrix)
                print('which cells scanned in this iteration:', a)
                print('which cells completed:', instance.coveredCells)

                particle.Position = findPositon(instance)
                print(findAgentCells(instance))

                connectedness = math.pow(len(instance.Agents), 2) - instance.connectivity_matrix().values.sum()
                print('connectedness:', connectedness)

                print(instance.connectivity_matrix())

            # for k in range(len(ins.Agents)):
            # ins.Agents[k].setCurrCell(ins.findCellFromId(particle.Position[k]))

            # evaluation
            particle.Cost = cost_function()

            print('particle cost:', particle.Cost)

            if particle.Cost < particle.BestCost:
                # update personal best:
                particle.BestPosition = particle.Position
                particle.BestCost = particle.Cost

                # update global best:
                if particle.BestCost < glob.BestCost:
                    glob.BestCost = particle.BestCost
                    glob.BestPosition = particle.BestPosition

        # store the best cost value:
        BestCosts[i] = glob.BestCost

        # display iteration information:
        print('Iteration: ', i, 'Best Cost = ', BestCosts[i])

        # damping inertia coefficient
        w = w * wDamp

        if glob.BestCost == 0:
            return


class Parameters:
    def __init__(self):
        self.MaxIt = None
        self.nPop = None

        self.w = None
        self.wDamp = None

        self.c1 = None
        self.c2 = None


def closest_value(input_list, input_value):
    arr = np.asarray(input_list)

    i = (np.abs(arr - input_value)).argmin()

    return arr[i]


def calculateOperation(a_list, b_list, ope):
    if ope == '+':
        v1 = [c + d for c, d in zip([a[0] for a in [x for x, y in zip(a_list, b_list)]],
                                    [b[0] for b in [y for x, y in zip(a_list, b_list)]])]
        v2 = [c + d for c, d in zip([a[1] for a in [x for x, y in zip(a_list, b_list)]],
                                    [b[1] for b in [y for x, y in zip(a_list, b_list)]])]
        return list(zip(v1, v2))
    elif ope == '*':
        v1 = [c * d for c, d in zip([a[0] for a in [x for x, y in zip(a_list, b_list)]],
                                    [b[0] for b in [y for x, y in zip(a_list, b_list)]])]
        v2 = [c * d for c, d in zip([a[1] for a in [x for x, y in zip(a_list, b_list)]],
                                    [b[1] for b in [y for x, y in zip(a_list, b_list)]])]
        return list(zip(v1, v2))
    elif ope == '-':
        v1 = [c - d for c, d in zip([a[0] for a in [x for x, y in zip(a_list, b_list)]],
                                    [b[0] for b in [y for x, y in zip(a_list, b_list)]])]
        v2 = [c - d for c, d in zip([a[1] for a in [x for x, y in zip(a_list, b_list)]],
                                    [b[1] for b in [y for x, y in zip(a_list, b_list)]])]
        return list(zip(v1, v2))


def moveToMinCoveredCell(instance, N_set):
    min = 100
    id = None
    for k, v in instance.dictOfCoverage.items():
        if v < min:
            min = v

    for k, v in instance.dictOfCoverage.items():
        if v == min:
            id = k
            break

    cell = instance.findCellCloserToACell(id, N_set)

    return cell


def calculateDist_to_cell(instance, x, y, N_set):
    dist = {}
    for n in N_set:
        center = instance.cellCenter[n]
        dist[n] = math.sqrt(math.pow(center[0] - x, 2) + math.pow(center[1] - y, 2))

    return min(dist, key=dist.get)


def findCloserCellWithCoordinates(instance, x_coord, y_coord, N_set):
    cell_id = calculateDist_to_cell(instance, x_coord, y_coord, N_set)

    cell = instance.findCellFromId(cell_id)
    return cell


def findNewCells(instance, coord_list, N_set):
    pos = []
    connectedness = math.pow(len(instance.Agents),2) - instance.connectivity_matrix().values.sum()
    print(connectedness)
    while not connectedness == 0:
        for i in range(len(instance.Agents)):
            coords_tuple = coord_list[i]
            cell = findCloserCellWithCoordinates(instance, coords_tuple[0], coords_tuple[1], N_set)
            pos.append(cell.getCenter())
            instance.Agents[i].setCurrCell(cell)
            instance.cellCenter.pop(cell.getID())
            N_set.remove(cell.getID())

        cellCenterListXLoc(instance)
        connectedness = math.pow(len(instance.Agents), 2) - instance.connectivity_matrix().sum()



def findCellid(instance, pos):
    newCell = []
    for p in pos:
        cell = list(p)
        newCell.append(instance.inWhichCellWithCenterInfo(cell[0], cell[1]))
    return newCell

def findAgentCells(instance):
    cell_list = []

    for agent in instance.Agents:
        cell_list.append(agent.getCurrCell().getID())

    return cell_list


def findPositon(instance):
    pos = []
    for agent in instance.Agents:
        center = agent.getCurrCell().getCenter()
        pos.append(center)

    return pos


def findRoute(instance, t):
    a_list = []
    for agent in instance.Agents:
        a_list.append(agent.getCurrCell().getID())

    instance.visitedCells[str(t)] = a_list


def findNSet(instance, sigma_matrix):
    R_l = instance.TypeAgents
    R_i = instance.AgentIndexSet
    R_j = instance.cellIDList()
    R_k = range(0, len(instance.Cells))

    # Define IDs:
    # idt = [t for t in R_t]
    idx = []
    for l in R_l:
        for i in range(len(R_i)):
            if l == i:
                for a in R_i[i]:
                    idx.append((l, a))
    idj = [j for j in R_j]
    idk = [j for j in R_k]

    N_set = []
    for x, y in idx:
        for j in idj:
            N_set.append(j) if j not in N_set else N_set
            for k in idk:
                if j != k:
                    a = sigma_matrix.loc[[x], [(j, k)]]
                    sigma_value = a[(j, k)].iloc[0]
                    if sigma_value == 1:
                        N_set.append(k) if k not in N_set else N_set

    N_set.sort()
    return N_set
