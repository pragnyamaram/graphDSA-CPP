# Graph Data Structures Detailed Implementation in C++

This repository GraphDSA-CPP is a collection of C++ implementations of common graph data structures and algorithms. Its purpose is educational: each file illustrates a different graph algorithm or operation, using clear C++ code that a beginner can study and run. The repository structure and contents are:
implementingGraph.cpp: Demonstrates basic graph construction (unweighted and weighted) using adjacency lists, and graph traversals (Breadth-First Search) plus cycle detection in an undirected graph. It contains Graph and weightedGraph classes with addEdge and printing methods, and shows how to detect cycles during BFS.

cycleDetectionUndirected.cpp: Implements cycle detection for both undirected and directed graphs. It defines a Graph class with separate adjacency lists for undirected and directed edges, and uses DFS or BFS to detect if a cycle exists in the graph.

belmanFordAlgo.cpp: Implements the Bellman–Ford algorithm for single-source shortest paths in a weighted graph that may contain negative edges. It uses a graph class with a weighted adjacency list, relaxes edges |V|–1 times, and detects negative cycles.

dijkstrasAlgo.cpp: Implements Dijkstra’s algorithm for single-source shortest paths with non-negative weights. It uses a graph class with weighted adjacency list and a std::set (as a min-heap) to repeatedly pick the closest unvisited vertex, updating distances in O(E log V) time
geeksforgeeks.org


flaydWarshalAlgo.cpp (Floyd–Warshall): Implements the Floyd–Warshall algorithm for all-pairs shortest paths. It initializes a distance matrix from the adjacency list, then uses a triple loop to update distances via intermediate vertices. The result is the shortest distance between every pair of nodes
geeksforgeeks.org


shortestPathDistance.cpp: Combines multiple shortest-path routines in one Graph class. It includes:


Unweighted shortest path: Finds the shortest path between two nodes in an unweighted graph via BFS.


Cycle detection: A DFS-based check for cycles in an undirected graph.


Topological sort: DFS to produce a topological order of a directed acyclic graph (DAG).


Shortest path in DAG: Uses the topological order to compute shortest paths in a weighted DAG efficiently.


kosarajuAlgo.cpp: Implements Kosaraju’s algorithm to find strongly connected components (SCCs) of a directed graph. It does a DFS to produce a finishing-time stack, reverses all edges, and then does DFS in stack order on the transposed graph. Each DFS from the stack root identifies one SCC
en.wikipedia.org
en.wikipedia.org


.tarjanAlgo.cpp: Implements Tarjan’s algorithm to detect bridges in an undirected graph. A bridge is an edge whose removal increases the number of connected components. This code uses DFS with discovery times and low-link values: when exploring an edge (u, v), if low[v] > disc[u] after recursion, the edge (u, v) is a bridge
geeksforgeeks.org
