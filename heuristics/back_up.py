def calculateDist_to_cell(instance, x, y, N_set):
    dist = {}
    for n in N_set:
        center = instance.cellCenter[n]
        dist[n] = math.sqrt(math.pow(center[0] - x, 2) + math.pow(center[1] - y, 2))

    coverage_requirement = []

    for k, v in dist.items():
        cover_req = instance.dictOfCoverage[k]
        pair = [k, cover_req]
        coverage_requirement.append(pair)

    print(coverage_requirement)

    min_value = min([v for k, v in coverage_requirement])

    counter = 0
    candidate_cell = []

    while len(candidate_cell) < len(instance.Agents):
        for k, v in coverage_requirement:
            if v == min_value and counter < len(instance.Agents):
                candidate_cell.append(k)
                counter += 1
        min_value += 1

    dist_list = []
    for candidate in candidate_cell:
        pair = [candidate, dist[candidate]]
        dist_list.append(pair)

    min_dist = min([v for k, v in dist_list])

    for k, v in dist_list:
        if v == min_dist:
            return k