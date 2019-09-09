import queue
import math


class Node:
    def __init__(self, x, y, value=None):
        self.x = x
        self.y = y
        self.value = value             # represents extra move points or to be a wall
        self.parent = None
        self.total_cost = 0


map = [[Node(0, 0, "-"), Node(0, 1, 0), Node(0, 2, 1), Node(0, 3, 0), Node(0, 4, 1)],
       [Node(1, 0, "-"), Node(1, 1, "-"), Node(1, 2, 2), Node(1, 3, 1), Node(1, 4, 2)],
       [Node(2, 0, 2), Node(2, 1, "-"), Node(2, 2, 3), Node(2, 3, 0), Node(2, 4, "-")],
       [Node(3, 0, 0), Node(3, 1, 2), Node(3, 2, 1), Node(3, 3, 1), Node(3, 4, "-")],
       [Node(4, 0, 1), Node(4, 1, 0), Node(4, 2, 3), Node(4, 3, 1), Node(4, 4, "-")]]


def validMove(map, x, y):
    numbOfRows = len(map)
    numbOfElementsInThatRow = len(map[0])

    if 0 <= x < numbOfRows and 0 <= y < numbOfElementsInThatRow:   # If the indexes are in between 0 and the number of rows
        if map[x][y].value != "-":                                 # and the value of the node at those indexes is not a wall
            return True                                            # it is a valid move
    return False


def printParentNodes(current):
    path = queue.LifoQueue()
    while True:
        path.put(current)                # Put the path into the stack until reaching the root node
        current = current.parent
        if current.parent == None:
            path.put(current)
            break
    while not path.empty():
        node = path.get()
        print("[", node.x, ",", node.y, "]", end="")


def add_to_frontier(frontier, node, parent, total_cost, count):
    found = False                                               # whether the node is already in the frontier or nor
    if len(frontier) == 0:
      node.parent = parent  # change the node's parent
      frontier.add((total_cost, count, node))

    else:
        for e in frontier:
          element = e[2]
          if element.x == node.x and element.y == node.y:      # if the node is already in the frontier set
            if element.total_cost <= total_cost:                # if the cost of the already added node is less than another
              found = True
            else:                                              # if the node's total cost is less than the already added one
              node.parent = parent                                  #change the node's parent
              found = True

    if not found:
      node.parent = parent  # change the node's parent
      frontier.add((total_cost, count, node))


def admissable_heuristic(current_node, target_node):
  heuristic = math.sqrt((current_node.x - target_node.x) ** 2 + (current_node.y - target_node.y) ** 2)
  return heuristic


def inadmissable_heuristic(current_node, target_node):
    if current_node.x == 0 and current_node.y == 0:
        heuristic = 150000
    else:
        heuristic = math.sqrt((current_node.x - target_node.x) ** 2 + (current_node.y - target_node.y) ** 2)
    return heuristic


def bfs(map, starting_x, starting_y, goal_x, goal_y):
    frontier = queue.Queue()                                    # Generates a FIFO Queue
    current = map[starting_x][starting_y]  # initializing current node as the starting node
    frontier.put(current)
    explored = []
    path_cost = 0

    found = False
    while True:
        if frontier.empty():  # there is no solution
            break

        current = frontier.get()
        explored.append(current)
        cX, cY = current.x, current.y

        print("Explored tile is ", "map[", cX, ",", cY, "]")

        if map[cX][cY].value != "-":
            path_cost = path_cost + map[cX][cY].value + 1   # Extra move points and the action cost of passing a node from another

        if cX == goal_x and cY == goal_y:
            found = True
            print("path cost is  ", path_cost)
            print("Solution is :")
            printParentNodes(current)
            print()
            break

        if found == False:  # Add Frontiers
            if validMove(map, cX, cY + 1) and (map[cX][cY + 1] not in explored):     # Right
                frontier.put(map[cX][cY + 1])
                map[cX][cY + 1].parent = current

            if validMove(map, cX + 1, cY) and (map[cX + 1][cY] not in explored):     # Down
                frontier.put(map[cX + 1][cY])
                map[cX + 1][cY].parent = current

            if validMove(map, cX, cY - 1) and (map[cX][cY - 1] not in explored):     # Left
                frontier.put(map[cX][cY - 1])
                map[cX][cY - 1].parent = current

            if validMove(map, cX - 1, cY) and (map[cX - 1][cY] not in explored):     # Up
                frontier.put(map[cX - 1][cY])
                map[cX - 1][cY].parent = current


def dfs(map, starting_x, starting_y, goal_x, goal_y):
    frontier = queue.LifoQueue()                              # Generates a LIFO Queue
    current = map[starting_x][starting_y]  # initializing current node as the starting node
    frontier.put(current)
    explored = []
    path_cost = 0

    found = False
    while True:
        if frontier.empty():  # there is no solution
            break

        current = frontier.get()
        explored.append(current)
        cX, cY = current.x, current.y

        print("Explored tile is ", "map[", cX, ",", cY, "]")

        if map[cX][cY].value != "-":
            path_cost = path_cost + map[cX][cY].value + 1  # Extra move points and the action cost of passing a node from another

        if cX == goal_x and cY == goal_y:
            found = True
            print("path cost is  ", path_cost)
            print("Solution is :")
            printParentNodes(current)
            print()
            break

        if found == False:  # Add Frontiers
            if validMove(map, cX, cY + 1) and (map[cX][cY + 1] not in explored):  # Right
                frontier.put(map[cX][cY + 1])
                map[cX][cY + 1].parent = current

            if validMove(map, cX + 1, cY) and (map[cX + 1][cY] not in explored):  # Down
                frontier.put(map[cX + 1][cY])
                map[cX + 1][cY].parent = current

            if validMove(map, cX, cY - 1) and (map[cX][cY - 1] not in explored):  # Left
                frontier.put(map[cX][cY - 1])
                map[cX][cY - 1].parent = current

            if validMove(map, cX - 1, cY) and (map[cX - 1][cY] not in explored):  # Up
                frontier.put(map[cX - 1][cY])
                map[cX - 1][cY].parent = current


def uniform_cost_with_ExtraPoints(map, starting_x, starting_y, goal_x, goal_y):
    explored = set()
    frontier = set()

    current = map[starting_x][starting_y]
    count = 0                               # Kept for in case of the priority equality to process the node earlier came
    frontier.add((0, count, current))       # We are adding nodes in tuples, first element is total cost
                                                # second one is the arrival order, and the third one is the node itself
    found = False
    while True:
        if len(frontier) == 0:
            break

        tupleOfMin = min(frontier)
        current = tupleOfMin[2]             # third element in the tuple is the node itself
        frontier.remove(tupleOfMin)
        explored.add(current)
        cX, cY = current.x, current.y

        print("Explored tile is ", "map[", cX, ",", cY, "]")

        if cX == goal_x and cY == goal_y:
            found = True
            print("Solution is :")
            printParentNodes(current)
            print()
            break

        if found == False:     # Add Frontiers
            if validMove(map, cX, cY + 1) and (map[cX][cY + 1] not in explored):
                count += 1
                cost = current.total_cost + map[cX][cY + 1].value + 1
                add_to_frontier(frontier, map[cX][cY + 1], current, cost, count)

            if validMove(map, cX + 1, cY) and (map[cX + 1][cY] not in explored):
                count += 1
                cost = current.total_cost + map[cX + 1][cY].value + 1
                add_to_frontier(frontier, map[cX + 1][cY], current, cost, count)

            if validMove(map, cX, cY - 1) and (map[cX][cY - 1] not in explored):
                count += 1
                cost = current.total_cost + map[cX][cY - 1].value + 1
                add_to_frontier(frontier, map[cX][cY - 1], current, cost, count)

            if validMove(map, cX - 1, cY) and (map[cX - 1][cY] not in explored):
                count += 1
                cost = current.total_cost + map[cX - 1][cY].value + 1
                add_to_frontier(frontier, map[cX - 1][cY], current, cost, count)


def uniform_cost_without_ExtraPoints(map, starting_x, starting_y, goal_x, goal_y):
    explored = set()
    frontier = set()

    current = map[starting_x][starting_y]
    count = 0  # Kept for in case of the priority equality to process the node earlier came
    frontier.add((0, count, current))  # We are adding nodes in tuples, first element is total cost
    # second one is the arrival order, and the third one is the node itself
    found = False
    while True:
        if len(frontier) == 0:
            break

        tupleOfMin = min(frontier)
        current = tupleOfMin[2]  # third element in the tuple is the node itself
        frontier.remove(tupleOfMin)
        explored.add(current)
        cX, cY = current.x, current.y

        print("Explored tile is ", "map[", cX, ",", cY, "]")

        if cX == goal_x and cY == goal_y:
            found = True
            print("Solution is :")
            printParentNodes(current)
            print()
            break
        if found == False:  # Add Frontiers
            if validMove(map, cX, cY + 1) and (map[cX][cY + 1] not in explored):
                count += 1
                cost = current.total_cost + 1
                add_to_frontier(frontier, map[cX][cY + 1], current, cost, count)

            if validMove(map, cX + 1, cY) and (map[cX + 1][cY] not in explored):
                count += 1
                cost = current.total_cost + 1
                add_to_frontier(frontier, map[cX + 1][cY], current, cost, count)

            if validMove(map, cX, cY - 1) and (map[cX][cY - 1] not in explored):
                count += 1
                cost = current.total_cost + 1
                add_to_frontier(frontier, map[cX][cY - 1], current, cost, count)

            if validMove(map, cX - 1, cY) and (map[cX - 1][cY] not in explored):
                count += 1
                cost = current.total_cost + 1
                add_to_frontier(frontier, map[cX - 1][cY], current, cost, count)


def A_star_with_AdmissableHeuristic(map, starting_x, starting_y, goal_x, goal_y):
    explored = set()
    frontier = set()

    current = map[starting_x][starting_y]
    count = 0
    frontier.add((0, count, current))

    found = False
    while True:
        if len(frontier) == 0:
            break

        tupleOfMin = min(frontier)
        current = tupleOfMin[2]
        frontier.remove(tupleOfMin)
        explored.add(current)
        cX, cY = current.x, current.y

        print("Explored tile is ", "map[", cX, ",", cY, "]")

        if cX == goal_x and cY == goal_y:
            found = True
            print("Solution is :")
            printParentNodes(current)
            print()
            break

        if found == False:  # Add Frontiers
            if validMove(map, cX, cY + 1) and (map[cX][cY + 1] not in explored):
                count += 1
                cost = current.total_cost + map[cX][cY + 1].value + 1 + admissable_heuristic(map[cX][cY + 1], map[goal_x][goal_y])
                add_to_frontier(frontier, map[cX][cY + 1], current, cost, count)

            if validMove(map, cX + 1, cY) and (map[cX + 1][cY] not in explored):
                count += 1
                cost = current.total_cost + map[cX + 1][cY].value + 1 + admissable_heuristic(map[cX + 1][cY], map[goal_x][goal_y])
                add_to_frontier(frontier, map[cX + 1][cY], current, cost, count)

            if validMove(map, cX, cY - 1) and (map[cX][cY - 1] not in explored):
                count += 1
                cost = current.total_cost + map[cX][cY - 1].value + 1 + admissable_heuristic(map[cX][cY - 1], map[goal_x][goal_y])
                add_to_frontier(frontier, map[cX][cY - 1], current, cost, count)

            if validMove(map, cX - 1, cY) and (map[cX - 1][cY] not in explored):
                count += 1
                cost = current.total_cost + map[cX - 1][cY].value + 1 + admissable_heuristic(map[cX - 1][cY], map[goal_x][goal_y])
                add_to_frontier(frontier, map[cX - 1][cY], current, cost, count)



def A_star_with_InadmissableHeuristic(map, starting_x, starting_y, goal_x, goal_y):
    explored = set()
    frontier = set()

    current = map[starting_x][starting_y]
    count = 0
    frontier.add((0, count, current))

    found = False
    while True:
        if len(frontier) == 0:
            break

        tupleOfMin = min(frontier)
        current = tupleOfMin[2]
        frontier.remove(tupleOfMin)
        explored.add(current)
        cX, cY = current.x, current.y

        print("Explored tile is ", "map[", cX, ",", cY, "]")

        if cX == goal_x and cY == goal_y:
            found = True
            print("Solution is :")
            printParentNodes(current)
            print()
            break

        if found == False:  # Add Frontiers
            if validMove(map, cX, cY + 1) and (map[cX][cY + 1] not in explored):
                count += 1
                cost = current.total_cost + map[cX][cY + 1].value + 1 + inadmissable_heuristic(map[cX][cY + 1], map[goal_x][goal_y])
                add_to_frontier(frontier, map[cX][cY + 1], current, cost, count)

            if validMove(map, cX + 1, cY) and (map[cX + 1][cY] not in explored):
                count += 1
                cost = current.total_cost + map[cX + 1][cY].value + 1 + inadmissable_heuristic(map[cX + 1][cY], map[goal_x][goal_y])
                add_to_frontier(frontier, map[cX + 1][cY], current, cost, count)

            if validMove(map, cX, cY - 1) and (map[cX][cY - 1] not in explored):
                count += 1
                cost = current.total_cost + map[cX][cY - 1].value + 1 + inadmissable_heuristic(map[cX][cY - 1], map[goal_x][goal_y])
                add_to_frontier(frontier, map[cX][cY - 1], current, cost, count)

            if validMove(map, cX - 1, cY) and (map[cX - 1][cY] not in explored):
                count += 1
                cost = current.total_cost + map[cX - 1][cY].value + 1 + inadmissable_heuristic(map[cX - 1][cY], map[goal_x][goal_y])
                add_to_frontier(frontier, map[cX - 1][cY], current, cost, count)



bfs(map, 0, 0, 4, 2)
print()

dfs(map, 0, 0, 4, 2)
print()

uniform_cost_with_ExtraPoints(map, 0, 0, 4, 2)
print()

uniform_cost_without_ExtraPoints(map, 0, 0, 4, 2)
print()

A_star_with_AdmissableHeuristic(map, 0, 0, 4, 2)
print()
