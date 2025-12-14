# ===========================================
#   Maze Graph Mapping Module for Webots
# ===========================================

SIZE = 12                     # Maze is 12 x 12
N = SIZE * SIZE               # 144 nodes total


# ----------- Direction Encoding --------------
# 0 = +Y (North), 1 = +X (East), 2 = -Y (South), 3 = -X (West)
dirs = {
    0: (0, 1),     # North
    1: (1, 0),     # East
    2: (0, -1),    # South
    3: (-1, 0)     # West
}


# ---------- Convert grid <-> node -------------
def rc_to_node(r, c):
    return r * SIZE + c + 1   # row-major, node 1..144


def node_to_rc(node):
    node -= 1
    return node // SIZE, node % SIZE


# ===========================================
# 1. Build initial graph (ALL VALID EDGES = 1)
# ===========================================
def build_initial_graph():
    graph = {i: {} for i in range(1, N+1)}

    for r in range(SIZE):
        for c in range(SIZE):
            node = rc_to_node(r, c)

            # Add edges only if inside maze boundaries
            if r > 0:                         # up
                graph[node][rc_to_node(r-1, c)] = 1

            if r < SIZE - 1:                  # down
                graph[node][rc_to_node(r+1, c)] = 1

            if c > 0:                         # left
                graph[node][rc_to_node(r, c-1)] = 1

            if c < SIZE - 1:                  # right
                graph[node][rc_to_node(r, c+1)] = 1

    return graph


# ===========================================
# 2. Remove edges (WALL FOUND → delete)
# ===========================================

def remove_edge(graph, a, b):

    if b in graph[a]:
        del graph[a][b]
    if a in graph[b]:
        del graph[b][a]


# ===========================================
# 3. Update graph using sensor wall detection
# ===========================================
def update_walls(graph, x, y, direction,
                 frontBlocked, leftBlocked, rightBlocked, backBlocked):

    currNode = rc_to_node(y, x)

    # FRONT
    dx, dy = dirs[direction]
    frontNode = rc_to_node(y + dy, x + dx)

    # LEFT
    dxl, dyl = dirs[(direction + 3) % 4]
    leftNode = rc_to_node(y + dyl, x + dxl)

    # RIGHT
    dxr, dyr = dirs[(direction + 1) % 4]
    rightNode = rc_to_node(y + dyr, x + dxr)

    # BACK
    dxb, dyb = dirs[(direction + 2) % 4]
    backNode = rc_to_node(y + dyb, x + dxb)

    # If blocked → remove that edge
    if frontBlocked:
        remove_edge(graph, currNode, frontNode)

    if leftBlocked:
        remove_edge(graph, currNode, leftNode)

    if rightBlocked:
        remove_edge(graph, currNode, rightNode)

    if backBlocked:
        remove_edge(graph, currNode, backNode)


# ===========================================
# 4. Move robot forward in the grid
# ===========================================
def move_forward(x, y, direction):
    dx, dy = dirs[direction]
    return x + dx, y + dy


# ===========================================
# 5. VISITED CELLS TRACKING
# ===========================================
visited = [False] * (N + 1)    # index 1..144 used


def mark_visited(x, y):
    """Mark the cell at (x, y) as visited."""
    node = rc_to_node(y, x)
    visited[node] = True


def is_visited(x, y):
    """Check if a cell has been visited."""
    node = rc_to_node(y, x)
    return visited[node]


def get_unvisited_nodes():
    """Return a list of nodes that are NOT visited at least once."""
    return [i for i in range(1, N+1) if not visited[i]]


def get_visited_nodes():
    """Return a list of nodes that WERE visited."""
    return [i for i in range(1, N+1) if visited[i]]


# ===========================================
# 6. Debug print
# ===========================================


def print_graph(graph):
    for node in range(1, N + 1):
        print(f"Node {node} -> {graph[node]}")
