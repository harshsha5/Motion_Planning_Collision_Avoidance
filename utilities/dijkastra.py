import pdb
from collections import defaultdict,deque


class Graph: 
    #NOTE: Reference stated in the documentation of the program
    # Constructor 
    def __init__(self): 
  
        # default dictionary to store graph 
        self.graph = defaultdict(list) 
  
    # function to add an edge to graph 
    def addEdge(self,u,v): 
        self.graph[u].append(v)  

    def addVertex(self,u):
        self.graph[u] = []

    def show_graph(self):
        print(self.graph.items())

def dijkstra(g,source, dest):
    #NOTE: Reference stated in the documentation of the program (dev.to)
    inf = float("inf")
    distance = {}
    previous_vertex = {}
    unvisited_vertices = []
    for key,val in g.graph.items():
        distance[key] = inf
        previous_vertex[key] = None
        unvisited_vertices.append(key)

    distance[source] = 0

    while unvisited_vertices:

        current_vertex = min(unvisited_vertices, key=lambda vertex: distance[vertex])

        if distance[current_vertex] == inf:
            print("No path Found")
            return

        for neighbor,cost in g.graph[current_vertex]:
            new_possible_route = distance[current_vertex] + cost

            if new_possible_route < distance[neighbor]:
                distance[neighbor] = new_possible_route
                previous_vertex[neighbor] = current_vertex
        unvisited_vertices.remove(current_vertex)

    path, current_vertex = deque(), dest
    while previous_vertex[current_vertex] is not None:
        path.appendleft(current_vertex)
        current_vertex = previous_vertex[current_vertex]
    if path:
        path.appendleft(current_vertex)
    return path


if __name__ == "__main__":
    g = Graph() 
    g.addVertex("a")
    g.addVertex("b")
    g.addVertex("c")
    g.addVertex("d")
    g.addVertex("e")
    g.addVertex("f")
    g.addEdge("a",["b",7])
    g.addEdge("a",["c",9])
    g.addEdge("a",["f",14])
    g.addEdge("b",["c",10])
    g.addEdge("b",["d",15])
    g.addEdge("c",["d",11])
    g.addEdge("c",["f",2])
    g.addEdge("d",["e",6])
    g.addEdge("e",["f",9])
    path = dijkstra(g,"a","e")
    print(path)
