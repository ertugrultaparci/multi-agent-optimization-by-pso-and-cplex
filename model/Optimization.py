import math
import os

from docplex.mp.model import Model

from heuristics.Pso import PSO
from model.model_stages.Stage2 import *
from model.model_stages.Stage3 import calculateRatio


class Optimization:
    def __init__(self):
        self.L = None
        self.filename_swarmModel1 = "C:/Users/ertug/OneDrive/Masaüstü/multi-agent-optimization-by-pso-and-cplex/model/model_output/swarmModel1.lp"
        self.filename_stage1solution = "C:/Users/ertug/OneDrive/Masaüstü/multi-agent-optimization-by-pso-and-cplex/model/model_output/stage1solution.json"
        self.filename_swarmModel2 = "C:/Users/ertug/OneDrive/Masaüstü/multi-agent-optimization-by-pso-and-cplex/model/model_output/swarmModel2.lp"
        self.filename_stage2solution = "C:/Users/ertug/OneDrive/Masaüstü/multi-agent-optimization-by-pso-and-cplex/model/model_output/stage2solution.json"
        self.filename_swarmModel4 = "C:/Users/ertug/OneDrive/Masaüstü/multi-agent-optimization-by-pso-and-cplex/model/model_output/swarmModel4.lp"
        self.filename_stage4solution = "C:/Users/ertug/OneDrive/Masaüstü/multi-agent-optimization-by-pso-and-cplex/model/model_output/stage4solution.json"

    def Stage1(self, instance):

        swarmModel1 = Model('stage1')

        # Define ranges:
        R_l = instance.TypeAgents
        R_i = instance.AgentIndexSet

        # Define IDs:
        idx = []
        for l in R_l:
            for i in range(len(R_i)):
                if l == i:
                    for a in R_i[i]:
                        idx.append((l, a))

        # Define decision variables:

        x = swarmModel1.binary_var_dict(idx, name="x")

        # Define budget constraint:

        budgetCons = swarmModel1.linear_expr()

        for l in R_l:
            for i in range(len(R_i)):
                if l == i:
                    for a in R_i[i]:
                        budgetCons += swarmModel1.sum(instance.AgentsType[l].cost * x[l, a])

        swarmModel1.add_constraint(budgetCons <= instance.Budget)

        # Define obj func:

        term_1 = 0

        for l in R_l:
            for i in range(len(R_i)):
                if l == i:
                    for a in R_i[i]:
                        term_1 += math.pi * math.pow(instance.AgentsType[l].rSense, 2) * x[l, a]

        term_2 = 0

        for l in R_l:
            for i in range(len(R_i)):
                if l == i:
                    for a in R_i[i]:
                        term_2 += instance.AgentsType[l].maxLifetime * x[l, a]

        term_3 = 0

        for l in R_l:
            for i in range(len(R_i)):
                if l == i:
                    for a in R_i[i]:
                        term_3 += (instance.AgentsType[l].cost * (
                                instance.planningHorizon * instance.periodLength / 3600)) * x[l, a]

        term_4 = 0

        for l in R_l:
            for i in range(len(R_i)):
                if l == i:
                    for a in R_i[i]:
                        term_4 += instance.AgentsType[l].minDistance * x[l, a]

        a1 = 0.1
        a2 = 0.01
        a3 = 1
        a4 = 0.1

        obj_func_value = a1 * term_1 + a2 * term_2 - a3 * term_3 - a4 * term_4

        # Objective Function:
        swarmModel1.maximize(obj_func_value)
        # print(smartFarmModel.export_to_string())
        # swarmModel1.print_information()

        solution = swarmModel1.solve(log_output=False)
        # assert solution, "solve failed"
        swarmModel1.report()
        solution.display()

        swarmModel1.export_as_lp(self.filename_swarmModel1)
        solution.export(self.filename_stage1solution)

    def Stage2(self, instance):
        # Initialize instances for drones which was chosen in Stage1:
        filename = self.filename_stage1solution
        initializeInstanceForChosenDrones(instance, filename)

        # Edge Length Determination:
        edgeLengthDetermination(instance)

        # initialize Cells and Boundary Cells with Edge Length Information:
        initializeCells(instance)
        initializeBoundaryCells(instance)

        # get cell center info:
        cellCenterListXLoc(instance)

        # decrease some cells from the boundary cells due to denied zone:
        # setting denied zone:
        deniedCells = [19, 29, 30, 59, 60, 79, 90]
        instance.setDeniedCells(deniedCells)

        # Let's calculate Stage 2:
        swarmModel2 = Model('stage2')

        # Define ranges:
        R_t = [0]
        R_l = instance.TypeAgents
        R_i = instance.AgentIndexSet
        R_j = range(0, len(instance.BoundaryCells))

        # Define IDs:
        idt = [t for t in R_t]
        idx = []
        for l in R_l:
            for i in range(len(R_i)):
                if l == i:
                    for a in R_i[i]:
                        idx.append((l, a))
        idj = [j for j in R_j]
        idz = [(x, y, j, t) for x, y in idx for j in R_j for t in R_t]

        # Define decision variables:
        z = swarmModel2.binary_var_dict(idz, name="z")

        for (l, i, j, t) in idz:
            for x, y in idx:
                if (l, i) == (x, y):
                    swarmModel2.add_constraint(z[l, i, j, t] <= 1, ctname='assignment_1')

        for x, y in idx:
            assignment_2 = swarmModel2.linear_expr()
            for (l, i, j, t) in idz:
                if (l, i) == (x, y):
                    assignment_2 += swarmModel2.sum(z[l, i, j, t])
            swarmModel2.add_constraint(assignment_2 == 1, ctname='assignment_2')

        for j in idj:
            assignment_3 = swarmModel2.linear_expr()
            for x, y in idx:
                assignment_3 += swarmModel2.sum(z[x, y, j, 0])
            swarmModel2.add_constraint(assignment_3 <= 1, ctname='assignment_3')

        # Alpha Connectedness:
        b_matrix = matrixWithinCommRangeCellJAgentTypeL(instance)

        # lambda function below is used to find the corresponding beta value:
        betaValue = lambda a, b, c, d: b_matrix.loc[[(a, b)], [(c, d)]][(c, d)].iloc[0]

        for l, i, j, t in idz:
            alpha_connectedness = swarmModel2.linear_expr()
            for l_, i_, j_, t_ in idz:
                if (l, i) != (l_, i_) and j != j_:
                    beta = betaValue(l, l_, j, j_)
                    alpha_connectedness += swarmModel2.sum(beta * z[l_, i_, j_, t_])
            swarmModel2.add_constraint(alpha_connectedness >= instance.alpha * z[l, i, j, t],
                                       ctname='alpha_connectedness')

        # Define obj func:
        distance_matrix = distanceBtwBaseLocationAndBoundaryCells(instance)

        obj_func_value = 0
        d = 0
        for x, y in idx:
            for (l, i, j, t) in idz:
                if (l, i) == (x, y):
                    obj_func_value += swarmModel2.sum(z[l, i, j, t] * distance_matrix[d][j])
            d += 1

        # Objective Function:
        swarmModel2.minimize(obj_func_value)
        # print(swarmModel2.export_to_string())
        # swarmModel2.print_information()

        solution2 = swarmModel2.solve(log_output=False)
        # assert solution, "solve failed"
        # swarmModel2.report()
        solution2.display()

        swarmModel2.export_as_lp(self.filename_swarmModel2)
        solution2.export(self.filename_stage2solution)

        # Initialize the location after obtain the result of unbalanced assignment problem.
        print('Stage 2 final solution:')
        initialLocation(instance, self.filename_stage2solution)

        # apply Particle Swarm Optimization to obtain fully connected swarm:
        #PSO(instance, MaxIt=100, nPop=1, w=1, wDamp=0.99, c1=2, c2=2)

        #initial_location = [19, 39, 1, 40]
        #initial_location = [1, 20, 39, 40]
        #initial_location = [72, 52, 93, 38, 47, 54, 55]
        initial_location = [70, 94, 93, 49]
        for i in range(len(instance.Agents)):
            instance.Agents[i].setCurrCell(instance.findCellFromId(initial_location[i]))

    def Stage3(self, instance):

        r_dict = calculateRatio(instance)
        max_r = max([r for d, r in r_dict.items()])
        leader_agent = list(r_dict.keys())[list(r_dict.values()).index(max_r)]
        instance.findAgentFromName(leader_agent).makeLeader()


        print('Stage 3 solution:')
        instance.findAgentFromName(leader_agent).printAgent()

    def Stage3_part2(self, instance):
        comm_matrix = instance.connectivity_matrix()
        r_dict = calculateRatio(instance)
        temp_leader_dict = {}
        for agent in instance.Agents:
            temp_leader_dict[agent.getName()] = {agent.getName(): r_dict[agent.getName()]}

        def check_negotiation(temp):
            drone_names = []
            for d, v in temp.items():
                for k, l in v.items():
                    drone_names.append(k.replace('Drone', ''))
            if (len(set(drone_names))) == 1:
                return True
        while not check_negotiation(temp_leader_dict):
            for agent in instance.Agents:
                agent_ID = agent.getName().replace('Drone', '')

                ratio_dict = {
                    list(temp_leader_dict[agent.getName()].keys())[0]: list(temp_leader_dict[agent.getName()].values())[
                        0]}
                for agent2 in instance.Agents:
                    agent2_ID = agent2.getName().replace('Drone', '')
                    if agent_ID != agent2_ID and comm_matrix.loc[agent2_ID, agent_ID] == 1:
                        ratio_dict[list(temp_leader_dict[agent2.getName()].keys())[0]] = list(temp_leader_dict[agent2.getName()].values())[0]
                max_r = max([r for d, r in ratio_dict.items()])
                leader_agent = list(ratio_dict.keys())[list(ratio_dict.values()).index(max_r)]
                temp_leader_dict[agent.getName()] = {leader_agent: max_r}
            print(temp_leader_dict)
            print(not check_negotiation(temp_leader_dict))
        print('START')
        print(temp_leader_dict)
        print('END')



        #Leader Selection During the Mission:
        """
        for r in range(len(instance.r_dict)):
        
            for agent in instance.Agents:
        """

    def Stage4(self, instance, t, sigma_matrix):

        # Let's calculate Stage 4:
        swarmModel4 = Model('stage4')

        # Define ranges:
        R_t = [t]
        R_l = instance.TypeAgents
        R_i = instance.AgentIndexSet
        R_j = instance.cellIDList()
        R_k = range(0, len(instance.Cells))

        # Define IDs:
        idt = [t for t in R_t]
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
        print('NSET:', N_set)
        idj = N_set
        idz = [(x, y, j, t) for x, y in idx for j in idj for t in R_t]

        # Define decision variables:
        z = swarmModel4.binary_var_dict(idz, name="z")

        for (l, i, j, t) in idz:
            for (k, r, m, n) in idz:
                if j != m:
                    const_1 = swarmModel4.linear_expr()
                    a = sigma_matrix.loc[[l], [(j, m)]]
                    sigma_value = a[(j, m)].iloc[0]
                    const_1 += swarmModel4.sum(z[l, i, j, t] + z[k, r, m, n])
                    swarmModel4.add_constraint(const_1 <= sigma_value + 1, ctname='new_pos_determination_1')

        for x, y in idx:
            const_2 = swarmModel4.linear_expr()
            for l, i, j, t in idz:
                if (x, y) == (l, i):
                    if j in instance.DeniedCells:
                        const_2 += swarmModel4.sum(z[l, i, j, t])
            swarmModel4.add_constraint(const_2 == 0, ctname='new_pos_determination_2')

        for x, y in idx:
            const_3 = swarmModel4.linear_expr()
            for l, i, j, t in idz:
                if (x, y) == (l, i):
                    const_3 += swarmModel4.sum(z[l, i, j, t])
            swarmModel4.add_constraint(const_3 <= 1, ctname='new_pos_determination_3')

        for n in N_set:
            const_4 = swarmModel4.linear_expr()
            for l, i, j, t in idz:
                if n == j:
                    const_4 += swarmModel4.sum(z[l, i, j, t])
            swarmModel4.add_constraint(const_4 <= 1, ctname='new_pos_determination_4')

        # f_value = 2

        """
        # Cell Coverage:
        a_matrix = instance.aMatrix()
        a_matrix.to_csv(r'C:/Users/ertug/OneDrive/Masaüstü/swarm2022 - Copy/model/model_output/my_data.csv', index=False)
        
        for c in idj:
            cell_coverage = swarmModel4.linear_expr()
            for (l, i, j, t) in idz:
                if c != j:
                    index = a_matrix.loc[[l], [(c, j)]]
                    a_value = index[(c, j)].iloc[0]
                    cell_coverage += swarmModel4.sum(a_value * z[l, i, j, t])
                    # f_j = f_value - instance.coverageArray[j][1]
            swarmModel4.add_constraint(cell_coverage >= f_value - instance.coverageArray[c][1], ctname='cell_coverage')
        """

        """
        # Collision Avoidance:
        
        for x, y in idx:
            for a, b in idx:
                for l, i, j, t in idz:
                    for k, r, m, n in idz:
                        if (x, y) != (a, b) and (x, y) == (l, i) and (a, b) == (k, r) and j != m and t == n:
                            const_5 = swarmModel4.linear_expr()
                            const_5 += swarmModel4.sum((z[l, i, j, t] + z[k, r, m, n] - 1) * (
                                    instance.getCrDistOfGivenType(l) + instance.getCrDistOfGivenType(k)))
                            swarmModel4.add_constraint(
                                const_5 <= instance.distanceBtwTwoCells(j, m),
                                ctname='coll_avoidance')
        """

        const_6 = swarmModel4.linear_expr()
        for n in idj:
            for l, i, j, t in idz:
                if n == j:
                    const_6 += swarmModel4.sum(z[l, i, j, t])
        swarmModel4.add_constraint(const_6 == len(instance.Agents), ctname='new_pos_determination_5')

        # objective function:
        obj_func = swarmModel4.linear_expr()
        for l, i, j, t in idz:
            f_j = instance.findCellFromId(j).coverageRequirement - instance.dictOfCoverage[j]
            if f_j <= 0:
                f_j = 100
                obj_func += swarmModel4.sum(f_j * z[l, i, j, t])
            else:
                obj_func += swarmModel4.sum((1 / f_j) * z[l, i, j, t])

        # Objective Function:
        swarmModel4.minimize(obj_func)
        # print(swarmModel4.export_to_string())
        # swarmModel4.print_information()
        swarmModel4.export_as_lp(self.filename_swarmModel4)

        solution = swarmModel4.solve(log_output=False)
        # assert solution, "solve failed"
        # swarmModel4.report()
        solution.display()
        solution.export(self.filename_stage4solution)
