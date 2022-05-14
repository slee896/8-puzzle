from math import sqrt

#A NODE STATE
class State:
    def __init__(self, board, goal, depth=0, f=-1):
        self.board = board
        self.goal = goal
        self.depth = depth
        self.f = f

    def __lf__(self, other):
        return self.f < other.f

#EXPAND CHILD FUNCTION 
def EXPAND(node, new_depth):
    pos_blank = node.board.index(0)  # Find blank position
    row, col = pos_blank // n, pos_blank % n
    print("position row:", row, "position col:", col)
    child = []
    for i in (range(len(OPERATORS))):
        nextrow, nextcol = row + OPERATORS[i][0], col + OPERATORS[i][1]
        if 0 <= nextrow < n and 0 <= nextcol < n:
            pos_next = nextrow * n + nextcol
            board = node.board[:]
            board[pos_blank], board[pos_next] = board[pos_next], board[pos_blank]  # swap
            child.append(State(board, goal, new_depth))
    return child  # child: expanded nodes

#A* MISTILED. h(x) is the number of different tiles from the goal. 
def ASTAR_Mistiled(nodes):
    def f(node):
        return h(node) + g(node)

    def h(node):
        return sum([1 if node.board[i] != node.goal[i] else 0 for i in range(len(node.board))])

    def g(node):
        return node.depth

    tmp_node = nodes[0]

    for i in range(len(nodes)):
        if nodes[i].f == -1:
            nodes[i].f = f(nodes[i])
        if nodes[0].f > nodes[i].f:
            tmp_node = nodes[0]
            nodes[0] = nodes[i]
            nodes[i] = tmp_node
    return nodes

#A* MISTILED. h(x) is sum of |(y-2)-(y-1)| + |(x-2)-(x-1)|, x: row, y: column 
def ASTAR_Manhattan(nodes):
    def f(node):
        return h(node) + g(node)

    def h(node):
        distance = 0
        for i in range(1, N):  # LOOP N times, N = problem size, N is in the main function.
            n_pos, g_pos = node.board.index(i), node.goal.index(i)
            n_row, n_col = n_pos // n, n_pos % n
            g_row, g_col = g_pos // n, g_pos % n
            distance += abs(n_row - g_row) + abs(n_col - g_col)
        return distance

    def g(node):
        return node.depth

    tmp_node = nodes[0]
    for i in range(len(nodes)):
        if nodes[i].f == -1:
            nodes[i].f = f(nodes[i])
        if nodes[0].t > nodes[i].f:
            tmp_node = nodes[0]
            nodes[0] = nodes[i]
            nodes[i] = tmp_node
    return nodes

def UNICOST(nodes):  # place the minimum depth node to nodes[0]
    for i in range(len(nodes)):
        if nodes[0].depth > nodes[i].depth:
            tmp_node = nodes[0]
            nodes[0] = nodes[i]
            nodes[i] = tmp_node
    return nodes

if __name__ == '__main__':
    print("Welcome to 8-Puzzle solver\n")
    print("------------Puzzle selection------------\n", "1. 8-puzzle given problems\n 2. Custom problem.\n")
    puzzle_selection = 0
    problem = []
    while puzzle_selection not in [1, 2]:
        puzzle_selection = int(input())
    if (puzzle_selection == 1):
        print("------------Problem selection------------\n",
              "1. depth0 \n 2. depth2\n 3. depth4\n 4. depth8\n 5. depth12\n 6. depth=16\n 7. depth 20\n 8. depth24")
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
    N = len(problem)
    n = int(sqrt(N))
    print("------------Algorithm selection------------\n",
          "1. UNIFORM COST \n2. A* with Mistiled \n3. A* with Manhattan distance\n")
    mode = 0
    while mode not in [1, 2, 3]:  # MODE 1 = UNIFORM COST, MODE 2 = A* with Mistiled, MODE 3 = A* with Manhattan
        mode = int(input())

    goal = [i for i in range(1, N)] + [0]  # Initialize goal.
    nodes = []  # 1D-Array, It will include 'State(node)' instances.
    current = State(problem, goal)
    nodes.append(current)
    visited = []
    depth = 0
    # print("nodesize: ", len(nodes))#qsize())rr
    max_expanded_node = 0  # MAX_QUEUE_SIZE
    OPERATORS = [(-1, 0), (1, 0), (0, -1), (0, 1)]  # up, down, left, right / to compute with (ROW, COLUMN)
    while (1):  # LOOP UNTIL IT IS SUCCESS OR THERE IS NO CHILD
        print(len(nodes))
        if len(nodes) == 0:
            print("failure\n")
            break
        # print("current", current.board)
        current = nodes.pop(0)  # remove FRONT-NODE of nodes.
        # print("current after pop", current.board)
        if current.board == goal:  # GOAL-TEST
            print("Success")
            break
        depth = current.depth + 1
        # print("BEFORE EXPAND, current=",current.board)
        # EXPAND CHILD
        for state in EXPAND(current, depth):
            # print("EXPAND CHILD:",state)
            flag = False
            for i in range(0, len(visited)):
                if (state.board == visited[i].board):
                    flag = True
                    break
            if flag is False:
                for j in range(0, len(nodes)):
                    if (state.board == nodes[j].board):
                        flag = True
                        break
            if flag is False:
                nodes.append(state)
        # UPDATE VISITED NODES
        visited.append(current)
        print("Current board: ", current.board, "Current depth:", current.depth)

        print("closed queue:", len(visited), "\n")
        print("current node:", current.board, "\n")
        print("expanded queue size:", len(nodes), "\n")
        if ((len(nodes)) > max_expanded_node):  # COUNT #OF MAX EXPANDED NODES.
            max_expanded_node = len(nodes)

        # QUEUEING FUNCTION. NEXT NODE SHOULD BE nodes[0]
        if mode == 1:
            nodes = UNICOST(nodes)
        if mode == 2:
            nodes = ASTAR_Mistiled(nodes)
        if mode == 3:
            nodes = ASTAR_Manhattan(nodes)
        ######### LOOP AGAIN  ###############
        
    print("MAX EXPANDED NODES:", max_expanded_node, "\n")
    print("DEPTH:", current.depth, "\n")