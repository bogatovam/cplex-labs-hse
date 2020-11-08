import math


def order_vertex(_n: int, _adjacency_lists: list):
    vertex_degree = [len(_adjacency_lists[i]) for i in range(0, _n)]
    ordered_vertex = []
    for i in range(0, _n):
        min_degree = math.inf
        min_degree_index = math.inf
        for j in range(0, _n):
            if vertex_degree[j] < min_degree and j not in ordered_vertex:
                min_degree = vertex_degree[j]
                min_degree_index = j

        ordered_vertex.append(min_degree_index)
        for neighbor in _adjacency_lists[min_degree_index]:
            vertex_degree[neighbor] -= 1

    return ordered_vertex

def find_min_color(neighbors_vector: list, colored_vertexes: list):
    min_color = 0
    neighbors_colors = []

    for neighbor in neighbors_vector:
        if colored_vertexes[neighbor] != math.inf:
            neighbors_colors.append(colored_vertexes[neighbor])

    neighbors_colors = set(neighbors_colors)

    while min_color in neighbors_colors:
        min_color += 1

    return min_color


def color_vertex(_n: int, _m: int, _adjacency_lists: list):
    vertices = order_vertex(_n, _adjacency_lists)
    colored_vertexes = [math.inf] * _n

    for vertex_index in vertices:
        min_neighbors_color = find_min_color(_adjacency_lists[vertex_index], colored_vertexes)
        colored_vertexes[vertex_index] = min_neighbors_color

    print(colored_vertexes)
    return colored_vertexes


def _find_max_colored_vertex(vertices: set, _colored_vertices: list):
    max_color = -math.inf
    max_color_vertex = -math.inf
    for v in vertices:
        if _colored_vertices[v] > max_color:
            max_color_vertex = v
            max_color = _colored_vertices[v]
    return max_color_vertex


def build_heuristic_clique(_n: int, _m: int, _adjacency_lists: list):
    _colored_vertices = color_vertex(_n, _m, _adjacency_lists)
    clique = set()
    candidates = set([i for i in range(0, _n)])

    for i in range(0, _n):
        max_colored_vertex = _find_max_colored_vertex(candidates, _colored_vertices)
        clique.add(max_colored_vertex)
        new_candidates = candidates.intersection(_adjacency_lists[max_colored_vertex])
        new_candidates = new_candidates.difference(clique)
        if len(new_candidates) == 0 or len(new_candidates.intersection(candidates)) == 0:
            break
        candidates = new_candidates

    print(clique)
    print(len(clique))
