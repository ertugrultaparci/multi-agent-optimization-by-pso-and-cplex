import random
import numpy as np


def PSO(ins, MaxIt, nPop, w, wDamp, c1, c2):
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
                    position_list.append(agent.getCurrCell().getID())

                p.Position = position_list
                self.p_list.append(p)

            # initialize velocity
            for particle in self.p_list:
                particle.Velocity = [0] * nVar

            # evaluation
            for particle in self.p_list:
                particle.Cost = cost_function()
                self.cost_list.append(particle.Cost)

                # update the personal best:
                particle.BestPosition = particle.Position
                particle.BestCost = particle.Cost

                # update global best:
                if particle.BestCost < glob.BestCost:
                    glob.BestPosition = [a-1 for a in particle.BestPosition]

                    for k in range(len(ins.Agents)):
                        ins.Agents[k].setCurrCell(ins.findCellFromId(glob.BestPosition[k]))

                    glob.BestCost = cost_function()

                    for k in range(len(ins.Agents)):
                        ins.Agents[k].setCurrCell(ins.findCellFromId(particle.BestPosition[k]))

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
            list_rx = [random.uniform(0, 1) for i in range(nVar)]
            v1 = [w * a for a in particle.Velocity]

            v2_3 = [a - b for a, b in zip(particle.BestPosition, particle.Position)]
            v2_2 = [a * b for a, b in zip(list_rx, v2_3)]
            v2 = list(a * c1 for a in v2_2)

            list_rx = [random.uniform(0, 1) for i in range(nVar)]
            v3_3 = [a - b for a, b in zip(glob.BestPosition, particle.Position)]
            v3_2 = [a * b for a, b in zip(list_rx, v3_3)]
            v3 = list(a * c2 for a in v3_2)

            particle.Velocity = [sum(item) for item in zip(v1, v2, v3)]

            # update position
            particle.Position = [closest_value(ins.idListOfBoundaryCells(), int(sum(item))) for item in zip(particle.Position, particle.Velocity)]

            print('initial particle position', particle.Position)

            ins.createUniqueCellId(particle.Position)

            for w in range(len(ins.Agents)):
                particle.Position[w] = ins.chosenCell[w]

            print('final particle position', particle.Position)

            for k in range(len(ins.Agents)):
                ins.Agents[k].setCurrCell(ins.findCellFromId(particle.Position[k]))

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