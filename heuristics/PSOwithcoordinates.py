import math
import random
import numpy as np

from model.model_stages.Stage2 import boundaryCellCenterListXLoc


def PSOwithcoords(ins, MaxIt, nPop, w, wDamp, c1, c2):
    # cost functions:
    cost_function = ins.CostFunctionForPID

    nVar = len(ins.Agents)  # number of unknown variables

    # PARAMETERS OF PSO
    MaxIt = MaxIt  # maximum number of Iterations
    nPop = nPop  # swarm size (population size)

    # the following coefficient is stated as such in general PSO implementation:
    w = w  # inertia coefficient
    wDamp = wDamp  # damping ratio of inertia coefficient

    c1 = c1  # personal acceleration coefficient
    c2 = c2  # social (global) coefficient

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

                for agent in ins.Agents:
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
                    cell = [agent.getCurrCell().getID() for agent in ins.Agents]
                    glob_cell = [a - 1 for a in cell]
                    glob.BestPosition = [ins.findCellFromId(cell).getCenter() for cell in glob_cell]

                    for k in range(len(ins.Agents)):
                        ins.Agents[k].setCurrCell(ins.findCellFromId(glob_cell[k]))

                    glob.BestCost = cost_function()

                    for k in range(len(ins.Agents)):
                        ins.Agents[k].setCurrCell(ins.findCellFromId(cell[k]))

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

            # update position
            particle.Position = calculateOperation(particle.Position, particle.Velocity, "+")

            print('initial particle position', particle.Position)

            # ins.createUniqueCellId(particle.Position)

            # for w in range(len(ins.Agents)):
            # particle.Position[w] = ins.chosenCell[w]

            findNewCells(ins, particle.Position)

            boundaryCellCenterListXLoc(ins)

            particle.Position = findPositon(ins)

            print('final particle position', particle.Position)
            print(findCellid(ins, particle.Position))

            for agent in ins.Agents:
                print(f'agent type: {agent.getType()}, indis: {agent.getIndis()}, cell: {agent.getCurrCell().getID()}')

            # for k in range(len(ins.Agents)):
            # ins.Agents[k].setCurrCell(ins.findCellFromId(particle.Position[k]))

            # evaluation
            particle.Cost = cost_function()

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


def calculateDist_to_cell(instance, x, y):
    dist = {}
    for k, v in instance.boundaryCellCenter.items():
        dist[k] = math.sqrt(math.pow(v[0] - x, 2) + math.pow(v[1] - y, 2))

    return min(dist, key=dist.get)


def findCloserCellWithCoordinates(instance, x_coord, y_coord):
    cell_id = calculateDist_to_cell(instance, x_coord, y_coord)

    cell = instance.findCellFromId(cell_id)
    return cell


def findNewCells(instance, coord_list):
    pos = []
    for i in range(len(instance.Agents)):
        coords_tuple = coord_list[i]
        cell = findCloserCellWithCoordinates(instance, coords_tuple[0], coords_tuple[1])
        pos.append(cell.getCenter())
        instance.Agents[i].setCurrCell(cell)
        instance.boundaryCellCenter.pop(cell.getID())


def findCellid(instance, pos):
    newCell = []
    for p in pos:
        cell = list(p)
        newCell.append(instance.inWhichCellWithCenterInfo(cell[0], cell[1]))
    return newCell

def findPositon(instance):
    pos = []
    for agent in instance.Agents:
        center = agent.getCurrCell().getCenter()
        pos.append(center)

    return pos