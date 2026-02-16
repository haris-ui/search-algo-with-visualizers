import heapq
from collections import deque
import math

class Node:
    def __init__(self, state, parent=None, action=None, cost=0, depth=0):
        self.state = state  # (row, col)
        self.parent = parent
        self.action = action
        self.cost = cost
        self.depth = depth

    def __lt__(self, other):
        return self.cost < other.cost

    def __eq__(self, other):
        return self.state == other.state

    def __hash__(self):
        return hash(self.state)

def get_successors(state, grid_size):
    """
    Generates successors in specific Clockwise order:
    1. Up
    2. Right
    3. Bottom
    4. Bottom-Right (Diagonal)
    5. Left
    6. Top-Left (Diagonal)
    """
    row, col = state
    rows, cols = grid_size
    actions = [
        ("Up", (-1, 0), 1),
        ("Right", (0, 1), 1),
        ("Bottom", (1, 0), 1),
        ("Bottom-Right", (1, 1), 1.5),
        ("Left", (0, -1), 1),
        ("Top-Left", (-1, -1), 1.5)
    ]

    successors = []
    for action, (dr, dc), cost in actions:
        new_row, new_col = row + dr, col + dc
        if 0 <= new_row < rows and 0 <= new_col < cols:
            successors.append(((new_row, new_col), action, cost))
    
    return successors

def reconstruct_path(node):
    path = []
    current = node
    while current.parent:
        path.append(current.action)
        current = current.parent
    return path[::-1] # Reverse path

def bfs(start, goal, grid_size):
    """Breadth-First Search"""
    start_node = Node(start)
    if start == goal:
        return []
    
    frontier = deque([start_node])
    explored = set()
    
    while frontier:
        node = frontier.popleft()
        explored.add(node.state)

        for state, action, cost in get_successors(node.state, grid_size):
            if state not in explored and state not in [n.state for n in frontier]:
                child = Node(state, node, action, node.cost + cost, node.depth + 1)
                if child.state == goal:
                    return reconstruct_path(child)
                frontier.append(child)
    return None

def dfs(start, goal, grid_size):
    """Depth-First Search - Graph Search version to avoid cycles"""
    start_node = Node(start)
    frontier = [start_node]
    explored = set()

    while frontier:
        node = frontier.pop() # LIFO
        
        if node.state == goal:
            return reconstruct_path(node)

        if node.state not in explored:
            explored.add(node.state)
            
            # Reverse order for stack to process strictly in order 1..6
            # Standard DFS pushes neighbors so last pushed is first popped.
            # If we want to expand "Up" first, "Up" should be at the top of stack.
            # So we push in strict reverse order of successors.
            neighbors = get_successors(node.state, grid_size)
            for state, action, cost in reversed(neighbors):
                if state not in explored:
                     child = Node(state, node, action, node.cost + cost, node.depth + 1)
                     frontier.append(child)
    return None

def ucs(start, goal, grid_size):
    """Uniform-Cost Search"""
    start_node = Node(start)
    frontier = []
    heapq.heappush(frontier, start_node)
    explored = set()
    
    while frontier:
        node = heapq.heappop(frontier)
        
        if node.state == goal:
            return reconstruct_path(node)
            
        if node.state not in explored:
            explored.add(node.state)
            
            for state, action, cost in get_successors(node.state, grid_size):
                if state not in explored:
                    new_cost = node.cost + cost
                    child = Node(state, node, action, new_cost, node.depth + 1)
                    # Check if child is in frontier with higher cost - strictly, UCS adds duplicates to heap or updates. 
                    # Heapq doesn't support update easily, so we can push. 
                    # Lazy deletion handles duplicates.
                    heapq.heappush(frontier, child)
    return None

def dls(start, goal, grid_size, limit):
    """Depth-Limited Search"""
    return recursive_dls(Node(start), goal, grid_size, limit, set())

def recursive_dls(node, goal, grid_size, limit, path_states):
    if node.state == goal:
        return reconstruct_path(node)
    
    if limit <= 0:
        return "cutoff"
    
    cutoff_occurred = False
    
    # Cycle checking for logic correctness in recursion path
    path_states.add(node.state)
    
    for state, action, cost in get_successors(node.state, grid_size):
        # Prevent cycles in current path
        if state not in path_states: 
            child = Node(state, node, action, node.cost + cost, node.depth + 1)
            result = recursive_dls(child, goal, grid_size, limit - 1, path_states)
            
            if result == "cutoff":
                cutoff_occurred = True
            elif result is not None:
                return result
                
    path_states.remove(node.state) # Backtrack
    
    if cutoff_occurred:
        return "cutoff"
    return None

def iddfs(start, goal, grid_size, max_depth=1000):
    """Iterative Deepening DFS"""
    for depth in range(max_depth):
        result = dls(start, goal, grid_size, depth)
        if result != "cutoff" and result is not None:
            return result
        # If result is None (failure) and not cutoff, we might stop early? 
        # But in infinite grid or complex graph, we typically just keep increasing depth.
    return None

def bidirectional_search(start, goal, grid_size):
    """Bidirectional Search using BFS"""
    if start == goal:
        return []
    
    # Forward BFS
    f_frontier = deque([Node(start)])
    f_explored = {start: Node(start)}
    
    # Backward BFS
    b_frontier = deque([Node(goal)])
    b_explored = {goal: Node(goal)}
    
    while f_frontier and b_frontier:
        # Expand forward
        if f_frontier:
            node = f_frontier.popleft()
            for state, action, cost in get_successors(node.state, grid_size):
                if state not in f_explored:
                    child = Node(state, node, action, node.cost + cost)
                    f_explored[state] = child
                    f_frontier.append(child)
                    if state in b_explored:
                        return merge_paths(child, b_explored[state])
        
        # Expand backward
        if b_frontier:
            node = b_frontier.popleft()
            # For backward search, we need inverse actions if we want to reconstruct perfectly,
            # or strictly just find connection.
            # Successors are same in undirected grid.
            for state, action, cost in get_successors(node.state, grid_size):
                if state not in b_explored:
                    # Parent pointer goes towards Goal
                    child = Node(state, node, action, node.cost + cost) 
                    b_explored[state] = child
                    b_frontier.append(child)
                    if state in f_explored:
                        return merge_paths(f_explored[state], child)
                        
    return None

def merge_paths(f_node, b_node):
    # f_node path: Start -> ... -> Meeting
    # b_node path: Goal -> ... -> Meeting (parent points to Goal)
    
    path_forward = reconstruct_path(f_node)
    
    path_backward = []
    curr = b_node
    while curr.parent:
        # Actions in b_node are "how we got here from goal direction", 
        # so if we moved "Up" from goal to get here, the path from start goes "Down"
        # But we just need to reconstruct.
        # Actually, let's keep it simple: just list actions.
        # Wait, if b_node is (next_state -> current_state), action is stored in child.
        # We need to reverse the actions for the second half?
        # The prompt asks for algorithms.
        # Let's simple return the list of nodes or standard actions if possible.
        # With "Strict Movement Order", we just need to find the path.
        
        # We need to inverse action? 
        # If we moved "Up" from Parent(Goal side) to Child(Meeting point),
        # real path is Child -> Parent which is "Down".
        
        action =  get_inverse_action(curr.action)
        path_backward.append(action)
        curr = curr.parent
        
    return path_forward + path_backward

def get_inverse_action(action):
    inverses = {
        "Up": "Bottom",
        "Bottom": "Up",
        "Left": "Right",
        "Right": "Left",
        "Top-Left": "Bottom-Right",
        "Bottom-Right": "Top-Left"
    }
    return inverses.get(action, action)
