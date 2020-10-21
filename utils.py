import urllib.request
import numpy as np


def read_graph(file_path):
    _n = _m = -1
    data = urllib.request.urlopen(file_path)
    for line in data:
        line = line.decode('ascii')
        line = line.strip('\n')
        if line.startswith('p'):
            _n = int(line.split(' ')[2])
            _m = int(line.split(' ')[3])
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


def to_confusion_matrix(_adjacency_lists: list):
    confusion_matrix = np.zeros((len(_adjacency_lists), len(_adjacency_lists)))
    for i in range(0, len(_adjacency_lists)):
        for elem in _adjacency_lists[i]:
            confusion_matrix[i][elem] = 1
    return confusion_matrix
