import urllib.request
import math
from time import time
import cplex
import pandas as pd
import heapq

log: dict = {
    'Graph name': [],
    'Heuristic time (sec)': [],
    'Heuristic max clique': [],
    'Heuristic max clique vertices': [],
    'B&B time  (sec)': [],
    'B&B max clique': [],
    'B&B max clique vertices': [],
    'Timeout': []
}
is_timeout = False
start_time = time()

import matplotlib.pyplot as plt
from matplotlib import pylab
import networkx as nx

BASE_PATH: str = "https://github.com/bogatovam/cplex-labs-hse/raw/master/graphs/"
import sys

sys.setrecursionlimit(3000)
# sorted by time from smallest to largest
GRAPHS_NAMES: list = [
    'hamming6-2.clq', 'MANN_a9.clq', 'san200_0.9_1.clq', 'san200_0.9_2.clq', 'hamming6-4.clq', 'gen200_p0.9_55.clq',
    'san200_0.9_3.clq', 'san200_0.7_1.clq', 'C125.9.clq', 'san200_0.7_2.clq', 'gen200_p0.9_44.clq', 'c-fat200-5.clq',
    'keller4.clq', 'c-fat200-2.clq', 'c-fat200-1.clq', 'c-fat500-10.clq', 'brock200_2.clq', 'c-fat500-5.clq',
    'c-fat500-1.clq', 'p_hat300-1.clq', 'brock200_3.clq', 'brock200_4.clq', 'c-fat500-2.clq', 'p_hat300-2.clq',
    'sanr200_0.7.clq', 'brock200_1.clq'
]
TIMEOUT_SEC = 3600
EPS = 0.0001


def save_graph(graph, file_name):
    # initialze Figure
    plt.figure(num=None, figsize=(20, 20), dpi=80)
    plt.axis('off')
    fig = plt.figure(1)
    pos = nx.spring_layout(graph)
    nx.draw_networkx_nodes(graph, pos)
    nx.draw_networkx_edges(graph, pos)
    nx.draw_networkx_labels(graph, pos)

    cut = 1.00
    xmax = cut * max(xx for xx, yy in pos.values())
    ymax = cut * max(yy for xx, yy in pos.values())
    plt.xlim(0, xmax)
    plt.ylim(0, ymax)

    plt.savefig(file_name, bbox_inches="tight")
    pylab.close()
    del fig


class Vertex:
    def __init__(self, index, degree, support) -> None:
        self.index = index
        self.degree = degree
        self.support = support

    def __eq__(self, o) -> bool:
        return self.index == o.index

    def __hash__(self) -> int:
        return hash(self.degree)

    def __lt__(self, other):
        if self.degree == other.degree:
            if self.support == other.support:
                return self.index < other.index
            else:
                return self.support < other.support
        else:
            return self.degree < other.degree


def support_vertex(vertex, _adjacency_lists):
    support = 0
    for neighbor in _adjacency_lists[vertex]:
        support += len(_adjacency_lists[neighbor])
    return support


def order_vertex(_n: int, _adjacency_lists: list):
    vertex_degree = [Vertex(i, len(_adjacency_lists[i]), support_vertex(i, _adjacency_lists)) for i in range(0, _n)]
    heapq.heapify(vertex_degree)
    ordered_vertex = []
    for i in range(0, _n):
        # work better(and slower) than immersion (specific for heapq)
        # but only this way we can improve heuristic
        heapq.heapify(vertex_degree)

        min_vertex = heapq.heappop(vertex_degree)
        ordered_vertex.append(min_vertex.index)
        for neighbor in _adjacency_lists[min_vertex.index]:
            if neighbor not in ordered_vertex:
                neighbor_index = vertex_degree.index(Vertex(neighbor, None, None))
                vertex_degree[neighbor_index].degree -= 1
                vertex_degree[neighbor_index].support -= min_vertex.degree

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

    return colored_vertexes


def _find_max_colored_vertex(vertices: set, _colored_vertices: list):
    max_color = -math.inf
    max_color_vertex = -math.inf
    for v in vertices:
        if _colored_vertices[v] > max_color:
            max_color_vertex = v
            max_color = _colored_vertices[v]
    return max_color_vertex


def build_heuristic_clique(_n: int, _m: int, _adjacency_lists: list, log: dict):
    start_time = time()

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

    log["Heuristic time (sec)"].append(time() - start_time)
    log["Heuristic max clique"].append(len(clique))
    log["Heuristic max clique vertices"].append(clique)

    return len(clique), clique


def is_close_to_integer(number: float):
    up = math.ceil(number)
    down = int(number)

    if number + EPS >= up:   return True
    if number - EPS <= down: return True
    return False


def round_with_eps(number: float):
    if number.is_integer():
        return number

    up = math.ceil(number)
    down = int(number)

    if number + EPS >= up:
        return up
    else:
        return down


def is_result_improved(current, updated):
    return updated > current


def is_result_integer(result: list):
    is_all_integer: bool = True

    for val in result:
        is_all_integer &= (val.is_integer() or is_close_to_integer(val))
    return is_all_integer


def branching(variables: list):
    min_diff = math.inf
    branching_var = -1

    for i in reversed(range(0, len(variables))):
        if not variables[i].is_integer():
            diff = min(math.fabs(variables[i]), math.fabs(1 - variables[i]))
            if diff < min_diff:
                min_diff = diff
                branching_var = i

    return branching_var


def build_new_constrains(branching_var):
    up = {
        "lin_expr": [[["y" + str(branching_var)], [1.0]]],
        "rhs": [1.0],
        "names": ["constraint_" + str(branching_var) + "_" + "up"],
        "senses": ['E']
    }
    down = {
        "lin_expr": [[["y" + str(branching_var)], [1.0]]],
        "rhs": [0.0],
        "names": ["constraint_" + str(branching_var) + "_" + "down"],
        "senses": ['E']
    }
    return up, down


def add_constraint(constraint: dict, model: cplex.Cplex):
    model.linear_constraints.add(
        lin_expr=constraint['lin_expr'],
        rhs=constraint['rhs'],
        names=constraint['names'],
        senses=constraint['senses']
    )


def delete_constraint(constraint_name, model: cplex.Cplex):
    model.linear_constraints.delete(constraint_name)


def check_timeout():
    global is_timeout
    return is_timeout or time() - start_time > TIMEOUT_SEC


def branch_and_bound(model: cplex.Cplex, optimal_objective_value, optimal_values):
    if check_timeout():
        global is_timeout
        is_timeout = True
        return optimal_objective_value, optimal_values

    model.solve()

    new_result = model.solution.get_objective_value()
    new_variables = model.solution.get_values()

    integer_new_result = round_with_eps(new_result)

    if not is_result_improved(optimal_objective_value, integer_new_result):
        # print("BOUND:\tSolution became worse from:\t{}\tto\t{}".format(optimal_objective_value, new_result))
        return optimal_objective_value, optimal_values
    if is_result_integer(new_variables):
        print("\nFound better integer solution:\tmax clique size:\t{}\n".format(new_result))
        return integer_new_result, new_variables

    branching_var = branching(new_variables)
    # print("BRANCHING:\t\tmax clique float size:\t{}\t\t\t\t\t\t{}".format(new_result, branching_var))

    up, down = build_new_constrains(branching_var)

    add_constraint(up, model)
    optimal_objective_value, optimal_values = branch_and_bound(model,
                                                               optimal_objective_value,
                                                               optimal_values)
    delete_constraint(up['names'][0], model)

    add_constraint(down, model)
    optimal_objective_value, optimal_values = branch_and_bound(model,
                                                               optimal_objective_value,
                                                               optimal_values)
    delete_constraint(down['names'][0], model)

    return optimal_objective_value, optimal_values


def transform_to_vertices(optimal_values):
    optimal_values = list(optimal_values)
    vertices = []
    for i in range(0, len(optimal_values)):
        if optimal_values[i] != 0.0: vertices.append(i)
    return vertices


def set_global_timeout(val: bool):
    global start_time
    global is_timeout

    is_timeout = val
    start_time = time()


def reduce_graph(_n: int, _m: int, _adjacency_lists: list, optimal_values: list):
    new_list = []
    for i in range(0, )


def run_branch_and_bound(_n: int, _m: int, _adjacency_lists: list):
    optimal_objective_value, optimal_values = build_heuristic_clique(n, m, adjacency_lists, log)

    print("HEURISTIC: Found heuristic solution:\t\tmax clique size:\t{}\n".format(optimal_objective_value))
    set_global_timeout(False)
    initial_model = init_cplex_system(_n, _m, _adjacency_lists)

    optimal_objective_value, optimal_values = branch_and_bound(initial_model,
                                                               optimal_objective_value,
                                                               optimal_values)

    optimal_values = transform_to_vertices(optimal_values)
    print("\nFINISH:\tmax clique size:\t{}\tmax clique:\t{}\n".format(optimal_objective_value, optimal_values))
    log["B&B time  (sec)"].append(time() - start_time)
    log["B&B max clique"].append(optimal_objective_value)
    log["B&B max clique vertices"].append(optimal_values)
    log["Timeout"].append(is_timeout)


def init_cplex_system(_n: int, _m: int, _adjacency_lists: list):
    max_clique_model = cplex.Cplex()

    max_clique_model.variables.add(names=["y" + str(i) for i in range(_n)],
                                   types=[max_clique_model.variables.type.continuous for i in range(n)])
    for i in range(_n):
        max_clique_model.variables.set_lower_bounds(i, 0.0)
        max_clique_model.variables.set_upper_bounds(i, 1.0)

    constrains = []
    constrains_names = []
    constrains_types = []
    constrains_right_parts = []

    for i in range(0, n):
        for j in range(i + 1, n):
            if j not in _adjacency_lists[i]:
                constrains.append([["y" + str(i), "y" + str(j)], [1.0, 1.0]])
                constrains_names.append("constraint_" + str(i) + "_" + str(j))
                constrains_types.append('L')
                constrains_right_parts.append(1.0)

    max_clique_model.linear_constraints.add(
        lin_expr=constrains,
        rhs=constrains_right_parts,
        names=constrains_names,
        senses=constrains_types
    )

    for i in range(_n):
        max_clique_model.objective.set_linear("y" + str(i), 1)
    max_clique_model.objective.set_sense(max_clique_model.objective.sense.maximize)

    max_clique_model.set_log_stream(None)
    max_clique_model.set_warning_stream(None)
    max_clique_model.set_results_stream(None)

    print("CPLEX was successfully initialized:\t\t{}\tconstrains\n".format(len(constrains)))
    return max_clique_model


def read_graph(file_path):
    _n = _m = -1
    data = urllib.request.urlopen(file_path)
    for line in data:
        line = line.decode('ascii')
        line = line.strip('\n')
        if line.startswith('p'):
            items = [int(s) for s in line.split() if s.isdigit()]
            _n = int(items[0])
            _m = int(items[1])
            break
    _adjacency_lists = [[] for i in range(0, _n)]
    for line in data:
        line = line.decode('ascii')
        line = line.strip('\n')
        if line.startswith('e'):
            i = int(line.split(' ')[1]) - 1
            j = int(line.split(' ')[2]) - 1
            _adjacency_lists[i].append(j)
            _adjacency_lists[j].append(i)
    return _n, _m, _adjacency_lists


def fake_graph():
    _n = 7
    _m = 11

    _adjacency_lists = [
        [1, 2, 6],
        [0, 2, 3, 6],
        [0, 1, 3, 5, 6],
        [1, 2, 4, 6],
        [3, 5],
        [2, 4],
        [0, 3, 2, 1]
    ]

    return _n, _m, _adjacency_lists


if __name__ == '__main__':
    for graph in GRAPHS_NAMES:
        print("\nSTART PROCESSING GRAPH\t\t\t{}\t\n".format(graph))
        full_path = BASE_PATH + graph
        n, m, adjacency_lists = read_graph(full_path)
        print("READING GRAPH\t{}\t\tvertices: {}, edges: {}\n".format(graph, n, m))

        log["Graph name"].append(graph)
        run_branch_and_bound(n, m, adjacency_lists)
        df = pd.DataFrame(log)
        df.to_excel("output.xlsx")
