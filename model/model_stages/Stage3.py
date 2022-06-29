
def calculateRatio(instance):
    comm_matrix = instance.connectivity_matrix()
    dist_matrix = instance.distanceMatrixOfAgents(instance.communicationGraph())
    r_list = {}

    for agent in instance.Agents:
        agent_ID = agent.getName().replace('Drone', '')
        total = 0
        r = 0
        for agent2 in instance.Agents:
            agent2_ID = agent2.getName().replace('Drone', '')
            if agent.getName() != agent2.getName() and comm_matrix.loc[agent_ID, agent2_ID] == 1:
                total += dist_matrix.loc[agent_ID, agent2_ID]

        r = (agent.getRComm() * agent.getRemEnergy()) / (total * agent.getCost())
        agent.setLeaderRatio(r)
        r_list[agent.getName()] = r

    return r_list




"""
   inflow_matrix = [[0 for col in range(len(instance.currAgents))] for row in range(len(instance.currAgents))]

    for agent in self.Agents:
        index = agent.getName().replace('Drone', '')
        my_list.append(index)
"""

"""
graph = {
  '5' : ['3','7'],
  '3' : ['2', '4'],
  '7' : ['8'],
  '2' : [],
  '4' : ['8'],
  '8' : []
}
"""
"""
def dfs(visited, graph, node):
    if node not in visited:
        print (node)
        visited.add(node)
        for neighbour in graph[node]:
            dfs(visited, graph, neighbour)
            
print("Following is the Depth-First Search")
dfs(visited, graph, '5')
"""
"""
def dfs(visited, graph, agent):
    if agent not in visited:
        print (agent)
        visited.add(agent)
        for neighbour in graph[agent]:
            dfs(visited, graph, neighbour)
"""
