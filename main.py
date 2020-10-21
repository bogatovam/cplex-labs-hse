import cplex
import math
import logging as log
import networkx as nx
import matplotlib.pyplot as plt

import heuristic
import utils

if __name__ == '__main__':
    path = "http://iridia.ulb.ac.be/~fmascia/files/DIMACS/hamming6-4.clq"
    # path = "http://iridia.ulb.ac.be/~fmascia/files/DIMACS/C125.9.clq"
    # path = "http://iridia.ulb.ac.be/~fmascia/files/DIMACS/keller4.clq"
    n, m, adjacency_lists = utils.read_graph(path)
    # n, m, adjacency_lists = utils.fake_graph()

    heuristic.build_heuristic_clique(n, m, adjacency_lists)
    #
    # graph = nx.from_numpy_matrix(utils.to_confusion_matrix(adjacency_lists))
    # nx.draw(graph, pos=nx.circular_layout(graph), node_color=colored_vertexes, font_color="r", with_labels=True,
    #         font_size=12)
    # plt.show()
