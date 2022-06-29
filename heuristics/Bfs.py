def isReachable(graph, s, d):
    # Breadth-First Search Algorithm:

    visited = []  # List for visited nodes.
    queue = []  # Initialize a queue

    visited.append(s)
    queue.append(s)

    while queue:  # Creating loop to visit each node
        m = queue.pop(0)

        if m == d:
            return True

        for neighbour in graph[m]:
            if neighbour not in visited:
                visited.append(neighbour)
                queue.append(neighbour)
    return False


def graphForBFS(graph):

    bfs_dict = [dict(zip([y], [x])) for y, x, k in graph]

    a_list = []
    for k in bfs_dict:
        x = list(k.keys())[0]
        val = list(k.values())[0]
        a_list.append((x.replace('Drone', ''), val.replace('Drone', '')))
    bfs_dict = {}
    number_list = []
    for k in range(0, len(a_list)):
        to_list = []
        for c in range(0, len(a_list)):
            if a_list[k][0] == a_list[c][0]:
                to_list.append(a_list[c][1])
            number_list.append(a_list[c][0]) if a_list[c][0] not in number_list else number_list
            number_list.append(a_list[c][1]) if a_list[c][1] not in number_list else number_list
        bfs_dict[a_list[k][0]] = to_list

    for n in number_list:
        if n not in bfs_dict.keys():
            bfs_dict[n] = []

    return bfs_dict
