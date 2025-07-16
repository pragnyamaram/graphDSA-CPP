# Graph Data Structures Detailed Implementation in C++
This repository GraphDSA-CPP is a collection of C++ implementations of common graph data structures and algorithms. Its purpose is educational: each file illustrates a different graph algorithm or operation, using clear C++ code that a beginner can study and run. The repository structure and contents are:
implementingGraph.cpp: Demonstrates basic graph construction (unweighted and weighted) using adjacency lists, and graph traversals (Breadth-First Search) plus cycle detection in an undirected graph. It contains Graph and weightedGraph classes with addEdge and printing methods, and shows how to detect cycles during BFS.
cycleDetectionUndirected.cpp: Implements cycle detection for both undirected and directed graphs. It defines a Graph class with separate adjacency lists for undirected and directed edges, and uses DFS or BFS to detect if a cycle exists in the graph.
belmanFordAlgo.cpp: Implements the Bellman–Ford algorithm for single-source shortest paths in a weighted graph that may contain negative edges. It uses a graph class with a weighted adjacency list, relaxes edges |V|–1 times, and detects negative cycles.
dijkstrasAlgo.cpp: Implements Dijkstra’s algorithm for single-source shortest paths with non-negative weights. It uses a graph class with weighted adjacency list and a std::set (as a min-heap) to repeatedly pick the closest unvisited vertex, updating distances in O(E log V) time
geeksforgeeks.org
.
flaydWarshalAlgo.cpp (Floyd–Warshall): Implements the Floyd–Warshall algorithm for all-pairs shortest paths. It initializes a distance matrix from the adjacency list, then uses a triple loop to update distances via intermediate vertices. The result is the shortest distance between every pair of nodes
geeksforgeeks.org
.
shortestPathDistance.cpp: Combines multiple shortest-path routines in one Graph class. It includes:
Unweighted shortest path: Finds the shortest path between two nodes in an unweighted graph via BFS.
Cycle detection: A DFS-based check for cycles in an undirected graph.
Topological sort: DFS to produce a topological order of a directed acyclic graph (DAG).
Shortest path in DAG: Uses the topological order to compute shortest paths in a weighted DAG efficiently.
kosarajuAlgo.cpp: Implements Kosaraju’s algorithm to find strongly connected components (SCCs) of a directed graph. It does a DFS to produce a finishing-time stack, reverses all edges, and then does DFS in stack order on the transposed graph. Each DFS from the stack root identifies one SCC
en.wikipedia.org
en.wikipedia.org
.
tarjanAlgo.cpp: Implements Tarjan’s algorithm to detect bridges in an undirected graph. A bridge is an edge whose removal increases the number of connected components. This code uses DFS with discovery times and low-link values: when exploring an edge (u, v), if low[v] > disc[u] after recursion, the edge (u, v) is a bridge
geeksforgeeks.org
.
The repository also contains:
A .gitignore file (to ignore compiled binaries and workspace files).
An output folder (possibly containing example outputs or data, if any).
A .vscode folder (IDE settings).
To use these algorithms, each .cpp file can be compiled (for example, with g++) and run. Each file contains its own main() function demonstrating the algorithm with example graphs. The code uses C++ STL containers like vector, unordered_map, and set for clarity and simplicity.
File: implementingGraph.cpp
This file shows how to build basic graph structures and perform common operations:
Graph construction:
The Graph class uses an unordered_map<int, vector<int>> adjList to represent an adjacency list for an unweighted graph.
The addEdge(int u, int v, bool direction) method inserts an edge u→v; if direction == 0 it adds v→u as well (undirected), otherwise only u→v (directed).
The printAdjList(int n) method loops from node 0 to n-1 and prints each adjacency list entry, for example printing 0 -> 1,2, if node 0 connects to 1 and 2. This helps visualize the graph.
Weighted graph variant:
A weightedGraph class is defined similarly, but its adjacency list is unordered_map<int, vector<pair<int,int>>> where each entry {v, weight} stores both neighbor and edge weight.
It also has addEdge(u, v, wt, direction) and printAdjList to display (neighbor,weight) pairs.
Breadth-First Search (BFS):
The code shows a BFS traversal from a source src. It uses a queue<int> q and an unordered_map<int,bool> visited. Initially, src is enqueued and marked visited.
While the queue isn’t empty, it dequeues frontNode, prints it, and enqueues any adjacent neighbor not yet visited (marking them visited). This prints all reachable nodes in BFS order.
Cycle detection (undirected):
The Graph class includes a method cycleDetectedUndirectedGraph(int src, unordered_map<int,bool> &visited) (the main uses g.cycleDetectedUndirectedGraph(i, visited) for each unvisited node).
In BFS-style cycle detection, it uses a queue plus a parent map to track each node’s parent in the BFS tree. For each neighbor nbr of the current node:
If nbr is not visited, it enqueues it and records parent[nbr] = currentNode.
If nbr is visited and nbr != parent[currentNode], then an undirected cycle is found (a back-edge not coming from the parent) and the function returns true.
In effect, if we reach a visited node that is not the immediate parent, we must have encountered a loop.
The main function uses this on all connected components; if any call returns true, it prints “CYCLE FOUND”.
Overall, implementingGraph.cpp teaches how to represent graphs and perform BFS and simple cycle checks. It serves as an introductory example before moving to more advanced algorithms.
File: cycleDetectionUndirected.cpp
This file focuses specifically on detecting cycles in graphs:
Graph class (undirected and directed lists):
The class contains two adjacency maps:
adjListUndirected for undirected edges.
adjListDirected for directed edges.
The methods addEdgeUndirected(u, v) and addEdgeDirected(u, v) add entries to these maps.
Printing the graph:
Two methods likely exist: printGraphUndirected(n) and printGraphDirected(n) (the raw snippet shows loops). They iterate nodes 0 to n-1, retrieve adjListUndirected[i] or adjListDirected[i], and print each list. This lets the user see the structure of both graphs.
Cycle detection (undirected):
Typically, an undirected-cycle check uses DFS (Depth-First Search) or BFS with a parent pointer. While the exact code isn’t fully visible, the common approach is:
Maintain a visited array/map.
For each unvisited node, start a recursive DFS (hasCycle(u, parent)).
In hasCycle(u, parent), mark u visited, then for each neighbor v of u:
If v is not visited, recurse hasCycle(v, u).
If v is visited and v != parent, return true (cycle found).
If no cycles are found, return false.
This works because in an undirected graph, encountering a visited neighbor that is not the one we came from means we closed a loop.
Cycle detection (directed):
The presence of adjListDirected suggests they also consider directed cycles. Detecting cycles in a directed graph is often done with DFS and a recursion stack:
Use two markers: visited[u] and inRecursionStack[u].
DFS from each unvisited u: mark visited[u]=true, inRecursionStack[u]=true.
For each neighbor v of u:
If not visited, recurse on v; if that finds a cycle, propagate true.
Else if inRecursionStack[v] is true, we found a back-edge (directed cycle).
After exploring all neighbors, pop u from the recursion stack (inRecursionStack[u]=false).
If any DFS call returns true, a directed cycle exists.
While the exact implementation details aren’t shown, the goal of cycleDetectionUndirected.cpp is to illustrate these cycle-finding techniques. The main function would typically add edges, call a cycle check on the undirected graph and possibly the directed graph, and print if a cycle is present.
File: belmanFordAlgo.cpp
This file implements the Bellman–Ford algorithm for single-source shortest paths in a weighted graph, even if edges can be negative (but with no negative cycles reachable from the source). Key points:
Graph representation:
Uses a class (named graph) with an adjacency map unordered_map<int, vector<pair<int,int>>> adjWt. Each entry adjWt[u] is a list of (v, weight) pairs, i.e. all directed edges and weights out of u.
Adding edges:
The method addEdgeWeight(u, v, wt, direction) adds a weighted edge. If direction == 0 (undirected), it also adds (u,wt) to adjWt[v]; if direction == 1, it only adds (v,wt) to adjWt[u].
Bellman–Ford algorithm:
Initialization: Create a distance array (e.g. a vector<int> distance(n, INT_MAX)) for all n nodes, and set the source distance to 0. All other distances are effectively “∞” (here INT_MAX).
Edge Relaxation: Repeat |V|–1 times: for each edge (u→v) with weight w, if distance[u] + w < distance[v], update distance[v] = distance[u] + w. This is relaxation. Relaxing all edges V-1 times ensures that the shortest paths (which can use at most V-1 edges) are found
geeksforgeeks.org
.
Negative cycle check (optional): After V-1 iterations, one more pass through all edges can detect negative cycles: if any distance can still be lowered, a negative cycle exists (not handled in all code variants).
Output: After relaxation, the distance array holds the shortest distances from the source to each node (or “∞”/INT_MAX if unreachable). The code may then print these distances, one per node.
Complexity: Bellman-Ford runs in O(V·E) time
geeksforgeeks.org
, since we relax all edges V-1 times.
Example (illustrative): Suppose we have edges (0→1 weight 5), (1→2 weight 3), and source = 0. Initially dist = [0, INF, INF]. After one relaxation, dist = [0, 5, INF]. After second (since V-1 = 2 for 3 nodes), relaxing (1→2) gives dist = [0, 5, 8]. No more updates after that. The final output is 0, 5, 8. Bellman–Ford is powerful because it handles negative edges (unlike Dijkstra)
geeksforgeeks.org
, but it’s slower. The code in belmanFordAlgo.cpp follows this approach step-by-step.
File: dijkstrasAlgo.cpp
This file contains an implementation of Dijkstra’s shortest-path algorithm, which finds the shortest distances from a source to all other nodes in a graph with non-negative edge weights. Highlights:
Graph structure:
It uses a graph class with a weighted adjacency map unordered_map<int, vector<pair<int,int>>> adjWt, similar to Bellman-Ford.
Adding edges:
The method addEdgeWeight(u, v, wt, direction) adds edges just like in Bellman-Ford (with optional undirected edges).
Printing adjacency list:
A helper method prints each node’s adjacency list of (neighbor, weight) pairs for verification.
Dijkstra’s algorithm (with a set):
Initialization: A vector distance of size n+1 is set to INT_MAX for all nodes, then distance[src] = 0.
Min-heap/set: Use a set<pair<int,int>> st or a priority queue to always extract the node with the smallest tentative distance (the snippet uses a set<pair<int,int>>, which orders by distance).
Main loop: Insert (0, src) into st. While st is not empty:
Extract the pair top = *st.begin(), which has the minimal distance. Let topNodeValue = top.first, topNode = top.second. Remove it from st.
For each neighbor (nbr.first, nbr.second) of topNode:
If topNodeValue + nbr.second < distance[nbr.first], we have found a shorter path to nbr.first.
If the neighbor was already in the set, erase its old entry. Update distance[nbr.first] = topNodeValue + nbr.second. Then insert the new pair {distance[nbr.first], nbr.first} into st.
This is exactly Dijkstra’s method: always “settle” the closest node, then relax its outgoing edges. As [60†L118-L124] describes, we repeatedly pick the minimum-distance node from a min-heap (or set) and update its neighbors.
Result printing: After the loop, the array distance[] holds the shortest distances from the source to every node. The code prints these values.
Complexity: Using a set or min-heap makes Dijkstra run in roughly O(E·log V) time
geeksforgeeks.org
. It cannot handle negative weights (unlike Bellman-Ford) but is faster on nonnegative graphs.
Example (concept): With edges 0–1 (4), 0–2 (8), 1–4 (6), etc., starting at source 0:
Initialize dist = [0, ∞, ∞, …].
Set contains (0,0). Pop it. Relax neighbors of 0: set dist[1]=4, dist[2]=8.
Set now has (4,1), (8,2). Pop (4,1). Relax neighbors of 1, etc.
Continue until all nodes settled.
Dijkstra’s greedy choice (pick nearest unvisited) is described in [60†L114-L124].
File: flaydWarshalAlgo.cpp
This file (named Floyd–Warshal algorithm) implements the Floyd–Warshall algorithm to compute shortest paths between every pair of vertices in a weighted graph.
Graph representation:
The code uses an adjacency list unordered_map<int, vector<pair<int,int>>> adjWt, but the Floyd–Warshall logic works on a matrix.
Floyd–Warshall function:
The key part is a function (e.g. vector<vector<int>> floydWarshal(int n)) that:
Initialize distance matrix: Creates an n × n matrix dist where dist[i][j] is set to infinity (often a large number like INT_MAX/2) except dist[i][i]=0. For each edge (u→v) with weight w, set dist[u][v] = w (and if undirected, dist[v][u] = w).
Triple loop (main algorithm): For each intermediate vertex k from 0 to n-1, for each pair (i, j), update:
if dist[i][k] + dist[k][j] < dist[i][j] then
    dist[i][j] = dist[i][k] + dist[k][j]
This checks if going from i to j via k is shorter than the current path. Over all k, this accounts for all possible intermediate nodes
geeksforgeeks.org
.
Result: After the loops, dist[i][j] holds the shortest distance from i to j for every pair.
Key idea: As [66†L179-L184] explains, Floyd–Warshall “treats each vertex k as an intermediate node, updating the shortest paths that pass through it.” Initially only direct edges are known; then considering one intermediate node at a time gradually yields final shortest paths
geeksforgeeks.org
.
Handling no edge / infinity: If there is no direct edge i→j, dist[i][j] was set to a large value (INF). If the graph is disconnected, those entries stay large.
Directed/Undirected: Floyd–Warshall works for both; the code just needs to populate the matrix correctly based on direction.
Complexity: O(n³), which is fine for small graphs but can be slow if n is large. It’s a classic algorithm for all-pairs shortest paths with dynamic programming.
Example (very small): Suppose n=3 and edges: 0→1 (5), 1→2 (2). Initialize:
dist = [[0, 5, ∞],
        [∞, 0, 2],
        [∞, ∞, 0]].
With k=0, nothing changes because 0 has no outgoing edges to 2 directly.
With k=1, check dist[0][2] > dist[0][1]+dist[1][2]? Yes (∞ > 5+2), so update dist[0][2] = 7.
With k=2, no further improvements.
Result: dist = [[0,5,7],[∞,0,2],[∞,∞,0]], showing that the shortest path 0→2 is 7 via 1.
The flaydWarshalAlgo.cpp code follows this pattern step-by-step (the partial snippet shows initializing dist, then comments about “step 1: diagonal 0, step 2: [loops]…”).
File: shortestPathDistance.cpp
This file combines several related routines in one Graph class. It includes both unweighted and weighted shortest path examples and a DAG shortest-path algorithm:
Unweighted shortest path (BFS):
Method shortestPath(int src, int destination): Uses BFS to find the shortest path (minimum edges) between src and destination in an unweighted graph.
It creates a visited map and a parent map. Enqueue src and mark visited/parent. Then standard BFS: for each dequeued frontNode, mark neighbors unvisited as visited and set parent[nbr] = frontNode.
Once BFS completes or when destination is reached, it reconstructs the path by following parent pointers backward from destination to src, storing nodes in a vector path. Then it reverses path so it goes from src to destination.
Finally, it prints the “SHORTEST PATH” and the sequence of nodes. This demonstrates how BFS finds shortest paths (in terms of number of edges) in O(V+E) time.
Cycle detection (undirected, DFS):
The code defines a function cycleDetectedUndirectedGraph(int src, visited, parent) using DFS. (Unlike BFS, DFS is natural for cycle check here.)
In DFS, mark src visited. For each neighbor nbr of src:
If nbr is not visited, recursively call the function.
Else if nbr != parent, then we have found a cycle (since we found a back-edge).
A global flag ans (or some mechanism) is set to true when a cycle is found.
The main code then checks all components: for each node i not yet visited, call cycleDetectedUndirectedGraph(i, parent=-1). If ans becomes true, it prints “CYCLE FOUND”. This illustrates DFS-based cycle detection (the classic “visited neighbor not equal to parent indicates a loop”).
Topological sort (DFS):
The code includes a recursive function topoSortDfs(int src, visited, stack). It marks src visited, then for each neighbor not yet visited it recurses. After exploring all neighbors, it pushes src onto a stack.
When applied to all nodes (in a loop), this produces a stack that contains the graph’s vertices in reverse topological order. Popping this stack gives one valid topological order of the DAG.
Shortest path in Directed Acyclic Graph (DAG):
Method shortestPathDirected(int src): Computes shortest paths in a weighted DAG (no cycles) in linear time using topological order. The steps:
a. Perform a topological sort of the DAG (using topoSortDfs on the original graph), obtaining a stack topoOrder.
b. Determine the number of vertices n by scanning all keys in the weighted adjacency list (adjWt).
c. Initialize a distance array dist of size n+1 to INT_MAX, set dist[src]=0.
d. Pop vertices from topoOrder in topological order. For each node u in that order: if dist[u] is not INT_MAX, then for each edge u→v of weight w, update dist[v] = min(dist[v], dist[u]+w).
e. After all nodes are processed, dist[] holds the shortest distances from src to every other node in the DAG. The code then prints this distance array.
This approach is standard: by processing nodes in topological order, each node’s distance is finalized by the time we see it (since DAG has no cycles)
geeksforgeeks.org
. It runs in O(V+E) time, faster than Dijkstra because we exploit acyclicity.
Summary of shortestPathDistance.cpp: It provides examples of BFS-based shortest path, DFS-based cycle detection, and a DAG shortest-path algorithm. Each is implemented clearly with parent maps or stacks.
File: kosarajuAlgo.cpp
This file implements Kosaraju’s algorithm for finding Strongly Connected Components (SCCs) in a directed graph:
Graph representation:
The Graph class uses unordered_map<int, vector<int>> adjWt to store a directed graph’s adjacency list. (The direction flag in addEdge indicates if the edge is directed or undirected, but for SCC we consider it directed.)
Adding edges:
addEdge(int u, int v, bool direction) adds u→v. If direction == 1, it adds only one directed edge. If direction == 0, it adds both u→v and v→u (effectively making the graph undirected).
Printing adjacency:
printAdjList(int n) prints each node’s neighbors for debugging.
Topological (finishing order) DFS:
topoSortDFS(stack<int> &st, int node, unordered_map<int,bool> &vis) does a DFS from node. It marks node visited, then recursively calls itself on any unvisited neighbors. After visiting all neighbors, it push(node) onto the stack. This ensures that nodes finish (are pushed) in post-order.
In the main() function, we loop over all vertices i=0..n-1, and if not yet visited, call topoSortDFS(st, i, visited). The resulting stack has vertices in descending order of finish times (largest finish time on top).
Transpose graph:
After the first DFS pass, we build the transpose of the graph (adjNew). For every original edge u→v, we add v→u in adjNew. This reverses all edges.
Second DFS pass (on transposed graph):
We then process vertices in the order of the stack st. Pop the top element src. If src is not yet visited in the new pass (visited2[src]), we start a DFS from src in the transposed graph using dfs(int src, visited2, adjNew). This DFS visits exactly one strongly connected component: all nodes reachable from src in the transpose (which correspond to nodes that could reach src in the original). We print each node as we visit (cout<<src) preceded by "SCC:". We then increment a component count.
According to Kosaraju’s algorithm, after doing this for all stack elements, we have identified and printed every SCC
en.wikipedia.org
en.wikipedia.org
.
Complexity: Each DFS pass takes O(V+E), so overall O(V+E). The use of a stack to record finishing order and then reversing edges is essential to this method.
Key idea: The first pass (filling stack) ensures that we process SCCs in correct order. The stack order means we always start a DFS from the “last finished” vertex of the original graph, which must be a source of some component in the transpose. By the time we pop any node from the stack, all nodes in “later” components have been removed, ensuring we find each component exactly once. This matches the explanation on Wikipedia
en.wikipedia.org
en.wikipedia.org
. Example: In a graph with two SCCs A and B where A can reach B but B cannot reach A, the finishing time of A’s nodes will be higher, so nodes of A are processed first in the second pass. The kosarajuAlgo.cpp code is a direct translation of this 2-pass method, printing each strongly connected component.
File: tarjanAlgo.cpp
This file uses Tarjan’s algorithm (for bridge-finding) on an undirected graph. It identifies bridges—edges which, if removed, increase the number of connected components.
Graph representation:
A graph class with unordered_map<int, vector<int>> adj for undirected adjacency. The addEdge(u, v, direction) method adds edges: if direction==1, it adds one directed edge; if direction==0, it adds both u→v and v→u. For bridge-finding we use undirected mode.
Bridge-finding algorithm:
The core is the recursive function countBridge(int src, int parent, unordered_map<int,int>& tin, unordered_map<int,int>& low, unordered_map<int,bool>& vis, int &timer). This is a DFS that computes:
tin[u]: the discovery time (the order in which node u was first visited).
low[u]: the lowest discovery time reachable from u by a DFS path that may use a back-edge.
The steps are:
Upon entering u, increment a global timer, set vis[u]=true, and tin[u] = low[u] = timer.
For each neighbor v of u:
If v == parent, skip it (we came from there).
Else if v is not visited, recurse countBridge(v, u, tin, low, vis, timer). After recursion, update low[u] = min(low[u], low[v]). Then bridge check: if low[v] > tin[u], it means the only way to reach v (or its descendants) from the root is via u. In other words, removing edge u–v disconnects the graph. So print "bridge exists between u and v".
Else (if v is visited and not the parent), it's a back-edge in the DFS tree; update low[u] = min(low[u], tin[v]).
The initial call is typically countBridge(0, -1, tin, low, visited, timer) for each connected component.
As [71†L37-L41] highlights, the key condition is exactly if (low[v] > tin[u]), then edge (u, v) is a bridge. This matches our algorithm (and the GFG explanation
geeksforgeeks.org
).
Overall usage: In main(), one would build the graph by adding edges, set up tin and low maps (initialized to 0), a visited map, and a timer = 0. Then for each vertex (or starting from 0 if connected), call countBridge(u, -1, tin, low, vis, timer). The function prints each bridge it finds.
Why it works: Intuitively, low[v] captures the earliest discovery time that v or its descendants can reach, excluding the direct parent link. If low[v] > tin[u], it means there is no back-edge from v’s subtree back to u or ancestors of u. Therefore, u–v is a critical link.
The tarjanAlgo.cpp code is a standard implementation of this Tarjan’s Bridge algorithm
geeksforgeeks.org
. It shows step-by-step DFS with time-stamping to identify bridges.
Each code file in GraphDSA-CPP is accompanied by comments and structured code that explain every step. Together, they form a tutorial-like collection covering key graph topics: building graphs, BFS/DFS, shortest paths (BFS, Dijkstra, Bellman-Ford, Floyd–Warshall, DAG algorithms), cycle detection, and connected components (both SCC and bridges). References: The algorithms above follow well-known descriptions (e.g., Dijkstra’s and Bellman-Ford as in GeeksforGeeks
geeksforgeeks.org
geeksforgeeks.org
, Kosaraju’s SCC algorithm
en.wikipedia.org
, Floyd–Warshall
geeksforgeeks.org
, and Tarjan’s bridges
geeksforgeeks.org
). These references provide additional details and illustrations of the principles behind the code.
Citations

Dijkstra's Algorithm to find Shortest Paths from a Source to all - GeeksforGeeks

https://www.geeksforgeeks.org/dsa/dijkstras-shortest-path-algorithm-greedy-algo-7/

Floyd Warshall Algorithm - GeeksforGeeks

https://www.geeksforgeeks.org/dsa/floyd-warshall-algorithm-dp-16/

Kosaraju's algorithm - Wikipedia

https://en.wikipedia.org/wiki/Kosaraju%27s_algorithm

Kosaraju's algorithm - Wikipedia

https://en.wikipedia.org/wiki/Kosaraju%27s_algorithm

Bridges in a graph - GeeksforGeeks

https://www.geeksforgeeks.org/dsa/bridge-in-a-graph/

Bellman–Ford Algorithm - GeeksforGeeks

https://www.geeksforgeeks.org/dsa/bellman-ford-algorithm-dp-23/

Bellman–Ford Algorithm - GeeksforGeeks

https://www.geeksforgeeks.org/dsa/bellman-ford-algorithm-dp-23/
