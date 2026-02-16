from search_algorithms import bfs, dfs, ucs, dls, iddfs, bidirectional_search

def print_result(algorithm_name, path):
    if path is not None:
        print(f"{algorithm_name}: Path found with {len(path)} steps.")
        print(f"Path: {path}")
    else:
        print(f"{algorithm_name}: No path found.")
    print("-" * 30)

def main():
    # Grid settings
    ROWS, COLS = 5, 5 # Small grid for demonstration/debugging
    start = (0, 0)
    goal = (4, 4)
    grid_size = (ROWS, COLS)

    print(f"Running Search Algorithms on {ROWS}x{COLS} Grid")
    print(f"Start: {start}, Goal: {goal}")
    print("=" * 40)

    # 1. BFS
    print_result("BFS", bfs(start, goal, grid_size))

    # 2. DFS
    print_result("DFS", dfs(start, goal, grid_size))

    # 3. UCS
    print_result("UCS", ucs(start, goal, grid_size))

    # 4. DLS (limit=10)
    print_result("DLS (limit=10)", dls(start, goal, grid_size, 10))

    # 5. IDDFS
    print_result("IDDFS", iddfs(start, goal, grid_size))

    # 6. Bidirectional Search
    print_result("Bidirectional Search", bidirectional_search(start, goal, grid_size))

if __name__ == "__main__":
    main()
