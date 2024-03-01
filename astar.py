from heapq import heappop, heappush

def parse_input(file_path):
    graph = {}
    heuristics = {}
    start = None
    end = None

    with open(file_path, 'r') as file:
        for line in file:
            line = line.strip()
            if line.startswith("ponto_inicial"):
                start = line.split("(")[1][0]
            elif line.startswith("ponto_final"):
                end = line.split("(")[1][0]
            elif line.startswith("pode_ir"):
                data = line.split(",")
                if len(data) == 4:
                    _, src, dest, weight = data
                    src = src.strip()
                    dest = dest.strip()
                    weight = int(weight.split(".")[0])
                    if src not in graph:
                        graph[src] = []
                    graph[src].append((dest, weight))
            elif line.startswith("h"):
                data = line.split(",")
                if len(data) == 4:
                    _, node, _, h_value = data
                    node = node.strip()
                    h_value = int(h_value.split(".")[0])
                    heuristics[node] = h_value

    return graph, start, end, heuristics

def astar(graph, start, end, heuristics):
    open_set = [(0, start)]
    came_from = {}
    g_score = {node: float('inf') for node in graph}
    g_score[start] = 0
    f_score = {node: float('inf') for node in graph}
    f_score[start] = heuristics[start]

    while open_set:
        current_f, current_node = heappop(open_set)

        if current_node == end:
            path = []
            while current_node in came_from:
                path.append(current_node)
                current_node = came_from[current_node]
            path.append(start)
            path.reverse()
            return path

        for neighbor, weight in graph[current_node]:
            tentative_g_score = g_score[current_node] + weight
            if tentative_g_score < g_score[neighbor]:
                came_from[neighbor] = current_node
                g_score[neighbor] = tentative_g_score
                f_score[neighbor] = g_score[neighbor] + heuristics[neighbor]
                heappush(open_set, (f_score[neighbor], neighbor))

    return None

file_path = "grafo.txt"

graph, start, end, heuristics = parse_input(file_path)

path = astar(graph, start, end, heuristics)
if path:
    print("Caminho encontrado:", path)
else:
    print("Não foi possível encontrar um caminho.")
