import networkx as nx
import numpy as np
import read_files
from networkx import DiGraph, from_numpy_matrix, relabel_nodes, set_node_attributes

DISTANCES = read_files.D[100]

A = np.array(DISTANCES, dtype=[("cost", int)])
G = from_numpy_matrix(A, create_using=nx.DiGraph())
G.remove_edges_from(read_files.no_route_truck)

print(G)