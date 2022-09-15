# Classes for graphs

class Node:

    def __init__(self, name: str, coords: tuple):
        self._name = name
        self._x = coords[0]
        self._y = coords[1]

    def name(self):
        return self._name

    def x(self):
        return self._x

    def y(self):
        return self._y

    def toString(self):
        return str(self.name()) # + "[" + str(self.x()) + "," + str(self.y()) + "]"

    def __str__(self):
        return self.toString()

    def __repr__(self):
        return self.toString()

class Edge:
    
    def __init__(self, start: Node, end: Node, cost: int):
        self._start = start
        self._end   = end
        self._cost  = cost

    def start(self):
        return self._start

    def end(self):
        return self._end

    def cost(self):
        return self._cost

    def toString(self):
        return str(self.start()) + "->" + str(self.end()) + "(" + str(self.cost()) + ")"

    def __str__(self):
        return self.toString()

    def __repr__(self):
        return self.toString()

class Graph:

    def __init__(self, data):
        self._valueToNode   = { v: Node(v[0], v_data["coords"]) for v, v_data in data.items() }
        self._valueToEdge   = { (start_v, end_v): Edge(self._valueToNode[start_v], self._valueToNode[end_v], end_cost) for start_v, start_data in data.items() for end_v, end_cost in start_data["neighbors"] }

        self._nodes         = list(self._valueToNode.values())
        self._edges         = list(self._valueToEdge.values())

    def getNode(self, v):
        return self._valueToNode[v]

    def getEdge(self, v):   
        return self._valueToEdge[v]

    def segments(self, edge: Edge):
        return list(filter(lambda seg: seg.start() == edge, self._edges))

    def toString(self):
        return "Graph:" + "\n" + "> Nodes :" + str(self._nodes) + "\n" + "> Edges :" + str(self._edges)

    def __str__(self):
        return self.toString()

    def __repr__(self):
        return self.toString()

# Classes for queues

class Queue:

    def __init__(self, data : list = []):
        self._elements = list(data);

    def empty(self):
        return len(self._elements) == 0

    def get(self):
        el = self._elements[0]
        self._elements.pop(0)
        return el;

    def put(self, v):
        self._elements.insert(0, v)

    def toString(self):
        return "Queue: " + str(self._elements)

    def __str__(self):
        return self.toString()

    def __repr__(self):
        return self.toString()

class PriorityQueue(Queue):
    
    def __init__(self, data : list = []):
        super().__init__(data)
        self.reorder()

    def put(self, v):
        self._elements.append(v)
        self.reorder()

    def reorder(self):
        self._elements = sorted(self._elements, key=lambda v: v[1])

    def toString(self):
        return "PriorityQueue: " + str(self._elements)

# Pathfinding functions

def getPath(start: Edge, end: Edge, came_from: dict):

    # follow the arrows backwards from the goal to the start
    current = end 
    path = []
    while current != start: 
        path.append(current)
        if not(current in came_from.keys()):
            return None
        current = came_from[current]

    path.append(start) # optional
    path.reverse() # optional

    return path

# Simple version, that not take in account the cost (v1)
def BreadthFirstSearch(graph: Graph, start: Edge, end: Edge):

    frontier  = Queue([ start ])
    came_from = dict({ start: None })

    # expansion process
    while not frontier.empty():
        current = frontier.get()
        
        # early exit
        if current == end:
            break    

        for segment in graph.segments(current):
            
            if segment.end() not in came_from:
                frontier.put(segment.end())
                came_from[segment.end()] = segment.start()

    return getPath(start, end, came_from)

# Dijkstra’s Algorithm version, that take in account the cost (v2)
def UniformCostSearch(graph: Graph, start: Edge, end: Edge):

    frontier = PriorityQueue([ (start, 0) ])
    came_from = dict({ start: None})
    cost_so_far = dict({ start: 0})

    while not frontier.empty():
        current_edge, current_priority = frontier.get()

        if current_edge == end:
            break

        for segment in graph.segments(current_edge):
            new_cost = cost_so_far[current_edge] + segment.cost()

            if segment.end() not in cost_so_far or new_cost < cost_so_far[segment.end()]:

                cost_so_far[segment.end()] = new_cost
                frontier.put((segment.end(), new_cost))
                came_from[segment.end()] = segment.start()

    path = getPath(start, end, came_from)
    return (path, cost_so_far[end]) if path else None

def heuristic(a: Node, b: Node):
    # Manhattan distance on a square grid
   return abs(a.x() - b.x()) + abs(a.y() - b.y())

# A* Algorithm version, that take in account the cost and it optimized for 1 path start -> end (v3)
def GreedyBestFirstSearch(graph: Graph, start: Edge, end: Edge):

    frontier = PriorityQueue([ (start, 0) ])
    came_from = dict({ start: None})
    cost_so_far = dict({ start: 0})

    while not frontier.empty():
        current_edge, current_priority = frontier.get()

        if current_edge == end:
            break

        for segment in graph.segments(current_edge):
            new_cost = cost_so_far[current_edge] + segment.cost()

            if segment.end() not in cost_so_far or new_cost < cost_so_far[segment.end()]:

                cost_so_far[segment.end()] = new_cost
                frontier.put((segment.end(), new_cost + heuristic(end, segment.end())))
                came_from[segment.end()] = segment.start()

    path = getPath(start, end, came_from)
    return (path, cost_so_far[end]) if path else None

if __name__ == '__main__':

    graph = Graph({
        "a": { "coords": (0, 2), "neighbors": [ ("b", 1) ] }, 
        "b": { "coords": (1, 2), "neighbors": [ ("e", 2) ] }, 
        "c": { "coords": (2, 2), "neighbors": [ ("f", 1) ] }, 
        "d": { "coords": (0, 1), "neighbors": [ ("b", 1), ("f", 15), ("g", 2) ] }, 
        "e": { "coords": (1, 1), "neighbors": [ ("d", 1) ] },
        "f": { "coords": (2, 1), "neighbors": [ ("e", 1), ("c", 3) ] },
        "g": { "coords": (0, 0), "neighbors": [ ("h", 1) ] }, 
        "h": { "coords": (1, 0), "neighbors": [ ("i", 1) ] },
        "i": { "coords": (2, 0), "neighbors": [ ("f", 1), ("c", 2) , ("e", 7)] }
    })

    print("2D Pathfinding in Python")

    print()
    print(graph)
    print()

    start = graph.getNode("b")
    end   = graph.getNode("f")

    print("> Breadth first search:")
    path = BreadthFirstSearch(graph, start, end)
    print("Path:", path)
    print()

    print("> Uniform cost search:")    
    path, cost = UniformCostSearch(graph, start, end)
    print("Path:", path)
    print("Cost:", cost)
    print()
    
    print("> Greedy best first search:")
    path, cost = GreedyBestFirstSearch(graph, start, end)
    print("Path:", path)
    print("Cost:", cost)
    print()

