import numpy as np
from .graph import Cell
from .utils import trace_path

"""
General graph search instructions:

First, define the correct data type to keep track of your visited cells
and add the start cell to it. If you need to initialize any properties
of the start cell, do that too.

Next, implement the graph search function. When you find a path, use the
trace_path() function to return a path given the goal cell and the graph. You
must have kept track of the parent of each node correctly and have implemented
the graph.get_parent() function for this to work. If you do not find a path,
return an empty list.

To visualize which cells are visited in the navigation webapp, save each
visited cell in the list in the graph class as follows:
     graph.visited_cells.append(Cell(cell_i, cell_j))
where cell_i and cell_j are the cell indices of the visited cell you want to
visualize.
"""


def depth_first_search(graph, start, goal):
    """Depth First Search (DFS) algorithm. This algorithm is optional for P3.
    Args:
        graph: The graph class.
        start: Start cell as a Cell object.
        goal: Goal cell as a Cell object.
    """
    graph.init_graph()  # Make sure all the node values are reset.

    """TODO (P3): Implement DFS (optional)."""
            

    # If no path was found, return an empty list.
    return []


def breadth_first_search(graph, start, goal):
    """Breadth First Search (BFS) algorithm.
    Args:
        graph: The graph class.
        start: Start cell as a Cell object.
        goal: Goal cell as a Cell object.
    """
    graph.init_graph()  # Make sure all the node values are reset.

    """TODO (P3): Implement BFS."""
    from collections import deque
    
    # initialize the queue with the start cell
    queue = deque([start])
    visited = {start}
    graph.visited_cells.append(start)  
    
    while queue:
        current = queue.popleft()
        
        # check if we reached the goal
        if current == goal:
            return trace_path(goal, graph)
        
        # explore neighbors
        for neighbor in graph.find_neighbors(current.i, current.j):
            if neighbor not in visited:
                visited.add(neighbor)
                graph.visited_cells.append(neighbor)
                graph.parent[neighbor] = current # set parent for path tracing
                queue.append(neighbor)

    # If no path was found, return an empty list.
    return []


def a_star_search(graph, start, goal):
    """A* Search (BFS) algorithm.
    Args:
        graph: The graph class.
        start: Start cell as a Cell object.
        goal: Goal cell as a Cell object.
    """
    graph.init_graph()  # Make sure all the node values are reset.

    """TODO (P3): Implement A*."""
    from collections import deque
    import heapq
    
    # initialize the open set with the start cell
    # counter allows consistent ordering in the heap
    
    counter = 0
    pq = [(0, counter, start)]
    counter += 1
    
    # initialize g_score and f_score dictionaries
    graph.g_score[start] = 0
    graph.f_score[start] = heuristic(start, goal)
    
    visited = set()
    graph.visited_cells.append(start)
    
    while pq:
        _, _, current = heapq.heappop(pq)
        
        # skip if already processed
        if current in visited:
            continue
        
        visited.add(current)
        
        # check if goal reached
        if current == goal:
            return trace_path(goal, graph)

        # explore neighbors
        for neighbor in graph.find_neighbors(current.i, current.j):
            if neighbor in visited:
                continue
                
            # calculate tentative score
            tentative_g = graph.g_score[current] + 1
            
            # if this path to neighbor is better than previous
            if neighbor not in graph.g_score or tentative_g < graph.g_score[neighbor]:
                graph.parent[neighbor] = current
                graph.g_score[neighbor] = tentative_g
                graph.f_score[neighbor] = tentative_g + heuristic(neighbor, goal)

                if neighbor not in visited:
                    graph.visited_cells.append(neighbor)
                    heapq.heappush(pq, (graph.f_score[neighbor], counter, neighbor))
                    counter += 1
            

    # If no path was found, return an empty list.
    return []

def heuristic(cell1, cell2):
    # manhattan distance for astar
    return abs(cell1.i - cell2.i) + abs(cell1.j - cell2.j)
