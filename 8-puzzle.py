from math import sqrt
import time

#A NODE STATE
class State:
    def __init__(self, board, goal, depth=0, path=[], f=-1):
        self.board = board      # deployment of numbers on 3x3 board
        self.goal = goal        # [1,2,3,4,5,6,7,8,0]
        self.depth = depth      # depth
        self.path = path        # trace
        self.f = f              # A* f value

    def __lt__(self, other):
        return self.f < other.f
# Trace printer
def path_return(node):
    for i in range(len(node.path)):
        print('--PATH[',i,']--','f=',node.path[i].f)
        for j in range(n):
            print(" ",node.path[i].board[j*n:(j+1)*n])

# EXPAND CHILD function
def EXPAND(node, new_depth, path, n):
    pos_blank = node.board.index(0)  # Find blank position
    row, col = pos_blank // n, pos_blank % n
    child = []
    for i in (range(len(OPERATORS))):
        nextrow, nextcol = row + OPERATORS[i][0], col + OPERATORS[i][1]
        if 0 <= nextrow < n and 0 <= nextcol < n:
            pos_next = nextrow * n + nextcol
            board = node.board[:]
            board[pos_blank], board[pos_next] = board[pos_next], board[pos_blank]  # swap
            child.append(State(board, goal, new_depth, path))
    return child  # child: expanded nodes

# A* MISTILED. h(x) is the number of different tiles from the goal.
def ASTAR_Mistiled(nodes):
    def f(node):
        return h(node) + g(node)
    def h(node):
        return sum([1 if node.board[i] != node.goal[i] else 0 for i in range(len(node.board))])
    def g(node):
        return node.depth
    for i in range(len(nodes)-1,0,-1):
        nodes[i].f = f(nodes[i])    #Calculate f value
        if nodes[i] < nodes[0]:     #Less f node moves to nodes[0]
            nodes[0], nodes[i] = nodes[i], nodes[0]
    return nodes

# A* MISTILED. h(x) is sum of |(y-2)-(y-1)| + |(x-2)-(x-1)|, x: row, y: column
def ASTAR_Manhattan(nodes):
    def f(node):
        return h(node) + g(node)
    def h(node):
        distance = 0
        for i in range(1, N):  # LOOP N times, N = problem size, N is in the main function.
            #n_pos: node position of i(1~N), g_pos: goal position of i(1~N)
            n_pos, g_pos = node.board.index(i), node.goal.index(i)
            n_row, n_col = n_pos // n, n_pos % n
            g_row, g_col = g_pos // n, g_pos % n
            distance += abs(n_row - g_row) + abs(n_col - g_col)
        return distance
    def g(node):
        return node.depth
    for i in range(len(nodes)-1,0,-1):
        nodes[i].f = f(nodes[i])    #Calculate f value
        if nodes[i] < nodes[0]:     #Less f node moves to nodes[0]
            nodes[0], nodes[i] = nodes[i], nodes[0]
    return nodes

# Uniform cost function
def UNICOST(nodes):
    for i in range(len(nodes)):
        if nodes[0].depth > nodes[i].depth:
            tmp_node = nodes[0]
            nodes[0] = nodes[i]
            nodes[i] = tmp_node
    return nodes

if __name__ == '__main__':
#### Setting problems
    print("Welcome to 8-Puzzle solver\n")
    print("------------Puzzle selection------------\n", "1. 8-puzzle given problems\n 2. Custom problem.\n")
    puzzle_selection = 0
    problem = []
    while puzzle_selection not in [1, 2]:
        puzzle_selection = int(input())
    if (puzzle_selection == 1):
        print("------------Problem selection------------\n",
              "1. depth0 \n 2. depth2\n 3. depth4\n 4. depth8\n 5. depth12\n 6. depth16\n 7. depth 20\n 8. depth24")
        given_prob = []
        given_prob.append([1, 2, 3, 4, 5, 6, 7, 8, 0])
        given_prob.append([1, 2, 3, 4, 5, 6, 0, 7, 8])
        given_prob.append([1, 2, 3, 5, 0, 6, 4, 7, 8])
        given_prob.append([1, 3, 6, 5, 0, 2, 4, 7, 8])
        given_prob.append([1, 3, 6, 5, 0, 7, 4, 8, 2])
        given_prob.append([1, 6, 7, 5, 0, 3, 4, 8, 2])
        given_prob.append([7, 1, 2, 4, 8, 5, 6, 3, 0])
        given_prob.append([0, 7, 2, 4, 6, 1, 3, 5, 8])
        i = int(input())
        print("You choose", i, "\n")
        problem = given_prob[i - 1]
    if (puzzle_selection == 2):
        print("1. 8-puzzle\n2. 15-puzzle\n3. 24-puzzle\n")
        custom_size = int(input()) + 2
        custom_prob = []
        print("type numbers one by one and Enter")
        for i in range(0, pow(custom_size, 2)):
            print("Type numbers", pow(custom_size, 2) - i, "times more.\n")
            custom_prob.append(int(input()))
        problem = custom_prob
### Define variables
    N = len(problem)
    n = int(sqrt(N))
    print("------------Algorithm selection------------\n",
          "1. UNIFORM COST \n2. A* with Mistiled \n3. A* with Manhattan distance\n")
    mode = 0
    while mode not in [1, 2, 3]:  # MODE 1 = UNIFORM COST, MODE 2 = A* with Mistiled, MODE 3 = A* with Manhattan
        mode = int(input())

    goal = [i for i in range(1, N)] + [0]   # Initialize goal.
    nodes = []                              # 1D-Array, It will include 'State(node)' instances.
    current = State(problem, goal)          # Current node
    nodes.append(current)                   # Open node array
    visited = []                            # Visited node array
    depth = 0                               # For updating node's depth
    path= []                                # For updating node's traces
    max_expanded_node = 0                   # MAX_QUEUE_SIZE check
    OPERATORS = [(-1, 0), (1, 0), (0, -1), (0, 1)]  # up, down, left, right / to compute with (ROW, COLUMN)

    time_start = time.time()
#### LOOP start
    while (1):
        # FAILURE TEST
        if len(nodes) == 0:
            print("failure\n")
            break
        current = nodes.pop(0)          # remove FRONT-NODE of nodes.
        depth = current.depth + 1       # To update node information
        path = current.path + [current] # To update node information

        # GOAL TEST
        if current.board == goal:
            print("Success")
            current.path = current.path + [current]
            break
        # EXPAND CHILD
        # If a child is a new one, add a child into 'nodes'
        for state in EXPAND(current, depth, path, n):
            flag = False
# If the state is already visited, will not be appended.
            for i in range(0, len(visited)):
                if (state.board == visited[i].board):
                    if (state.f < visited[i].f):
                        visited[i] = state
                    flag = True
                    break
# If the state is already opened, will not be appended.
            if flag is False:
                for j in range(0, len(nodes)):
                    if (state.board == nodes[j].board) and (state.f is not -1):
                        if (state.f < nodes[j].f):
                            nodes[j] = state
                        flag = True
                        break
            if flag is False:
                nodes.append(state)

        if ((len(nodes)) > max_expanded_node):  # COUNT #OF MAX EXPANDED NODES.
            max_expanded_node = len(nodes)
        visited.append(current)                 # Check current node is visited
        # QUEUEING FUNCTION. NEXT NODE SHOULD BE nodes[0]
        if mode == 1:
            nodes = UNICOST(nodes)
        if mode == 2:
            nodes = ASTAR_Mistiled(nodes)
        if mode == 3:
            nodes = ASTAR_Manhattan(nodes)

    time_end = time.time()
#### PRINT RESULTS
    path_return(current) # return traces and f value
    print("\n")
    print(f"{time_end - time_start:.5f} sec")
    print("MAX EXPANDED NODES:", max_expanded_node)
    print("DEPTH:", current.depth)
