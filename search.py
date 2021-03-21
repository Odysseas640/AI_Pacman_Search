# search.py
# ---------
# Licensing Information:  You are free to use or extend these projects for
# educational purposes provided that (1) you do not distribute or publish
# solutions, (2) you retain this notice, and (3) you provide clear
# attribution to UC Berkeley, including a link to http://ai.berkeley.edu.
# 
# Attribution Information: The Pacman AI projects were developed at UC Berkeley.
# The core projects and autograders were primarily created by John DeNero
# (denero@cs.berkeley.edu) and Dan Klein (klein@cs.berkeley.edu).
# Student side autograding was added by Brad Miller, Nick Hay, and
# Pieter Abbeel (pabbeel@cs.berkeley.edu).


"""
In search.py, you will implement generic search algorithms which are called by
Pacman agents (in searchAgents.py).
"""

import util

class SearchProblem:
    """
    This class outlines the structure of a search problem, but doesn't implement
    any of the methods (in object-oriented terminology: an abstract class).

    You do not need to change anything in this class, ever.
    """

    def getStartState(self):
        """
        Returns the start state for the search problem.
        """
        util.raiseNotDefined()

    def isGoalState(self, state):
        """
          state: Search state

        Returns True if and only if the state is a valid goal state.
        """
        util.raiseNotDefined()

    def getSuccessors(self, state):
        """
          state: Search state

        For a given state, this should return a list of triples, (successor,
        action, stepCost), where 'successor' is a successor to the current
        state, 'action' is the action required to get there, and 'stepCost' is
        the incremental cost of expanding to that successor.
        """
        util.raiseNotDefined()

    def getCostOfActions(self, actions):
        """
         actions: A list of actions to take

        This method returns the total cost of a particular sequence of actions.
        The sequence must be composed of legal moves.
        """
        util.raiseNotDefined()


def tinyMazeSearch(problem):
    """
    Returns a sequence of moves that solves tinyMaze.  For any other maze, the
    sequence of moves will be incorrect, so only use this for tinyMaze.
    """
    from game import Directions
    s = Directions.SOUTH
    w = Directions.WEST
    return  [s, s, w, s, w, w, s, w]

def depthFirstSearch(problem):
    """
    Search the deepest nodes in the search tree first.

    Your search algorithm needs to return a list of actions that reaches the
    goal. Make sure to implement a graph search algorithm.

    To get started, you might want to try some of these simple commands to
    understand the search problem that is being passed in:
    """
    # print "Start:", problem.getStartState()
    # print "Is the start a goal?", problem.isGoalState(problem.getStartState())
    # print "Start's successors:", problem.getSuccessors(problem.getStartState())
    #"""
    "*** YOUR CODE HERE ***"
    frontier = util.Stack()
    explored = set() # Initialize list for visited nodes
    node = problem.getStartState()
    push_to_frontier = [node]
    push_to_frontier.append([])
    push_to_frontier.append(0)
    util.Stack.push(frontier, push_to_frontier)
    while util.Stack.isEmpty(frontier) == False:
        node = util.Stack.pop(frontier)
        explored.add(node[0])
        if problem.isGoalState(node[0]) == True:
            return node[1]
        for each_successor in problem.getSuccessors(node[0]):
            if each_successor[0] in explored:
                continue
            successor_tile = each_successor[0]
            successor_direction = each_successor[1]
            successor_cost = each_successor[2]
            if successor_tile not in explored:
                direction_to_append = []
                direction_to_append = direction_to_append + node[1]
                direction_to_append.append(successor_direction)
                util.Stack.push(frontier, [successor_tile, direction_to_append, successor_cost + node[2]])

    util.raiseNotDefined()

def breadthFirstSearch(problem):
    """Search the shallowest nodes in the search tree first."""
    "*** YOUR CODE HERE ***"
    frontier = util.Queue()
    explored = [] # Initialize list for visited nodes
    node = problem.getStartState()
    push_to_frontier = [node]
    push_to_frontier.insert(1, [])
    push_to_frontier.append(0)
    util.Queue.push(frontier, push_to_frontier)
    while util.Queue.isEmpty(frontier) == False:
        node = util.Queue.pop(frontier)
        if node[0] in explored:
            continue
        explored.append(node[0])
        if problem.isGoalState(node[0]) == True:
            return node[1]
        for each_successor in problem.getSuccessors(node[0]):
            if each_successor[0] not in explored:
                successor_tile = each_successor[0] # Successor contains a tile, a list of moves and cost to get there
                successor_direction = each_successor[1]
                successor_cost = each_successor[2]
                direction_to_append = []
                direction_to_append = direction_to_append + node[1]
                direction_to_append.append(successor_direction)
                put_in_frontier = node[:]
                put_in_frontier[0] = successor_tile
                put_in_frontier[1] = direction_to_append
                put_in_frontier[2] = successor_cost + node[2]
                util.Queue.push(frontier, put_in_frontier)

def uniformCostSearch(problem):
    """Search the node of least total cost first."""
    "*** YOUR CODE HERE ***"
    frontier = util.PriorityQueue()
    explored = set() # Initialize list for visited nodes
    node = problem.getStartState()
    push_to_frontier = [node]
    push_to_frontier.append([])
    push_to_frontier.append(0)
    util.PriorityQueue.push(frontier, push_to_frontier, 0)
    while util.PriorityQueue.isEmpty(frontier) == False:
        node = util.PriorityQueue.pop(frontier)
        if node[0] in explored:
            continue
        explored.add(node[0])
        if problem.isGoalState(node[0]) == True:
            return node[1]
        for each_successor in problem.getSuccessors(node[0]):
            if each_successor[0] in explored:
                continue
            successor_tile = each_successor[0]
            successor_direction = each_successor[1]
            successor_cost = each_successor[2]
            if successor_tile not in explored:
                direction_to_append = []
                direction_to_append = direction_to_append + node[1]
                direction_to_append.append(successor_direction)
                util.PriorityQueue.update(frontier, [successor_tile, direction_to_append, successor_cost + node[2]], successor_cost + node[2])

    util.raiseNotDefined()

def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0

def aStarSearch(problem, heuristic=nullHeuristic):
    """Search the node that has the lowest combined cost and heuristic first."""
    "*** YOUR CODE HERE ***"
    frontier = util.PriorityQueue()
    explored = [] # Initialize list for visited nodes
    start_state = problem.getStartState()
    util.PriorityQueue.push(frontier, [start_state, [], 0], 1)
    while util.PriorityQueue.isEmpty(frontier) == False:
        node = util.PriorityQueue.pop(frontier)
        if node[0] in explored:
            continue
        explored.append(node[0])
        if problem.isGoalState(node[0]) == True:
            return node[1]
        for each_successor in problem.getSuccessors(node[0]):
            successor_tile = each_successor[0]
            successor_direction = each_successor[1]
            successor_cost = node[2] + each_successor[2] # fix this in other functions as well
            if successor_tile not in explored:
                direction_to_append = []
                direction_to_append = direction_to_append + node[1] # cost = len(node[1]) = length of moves
                direction_to_append.append(successor_direction)
                util.PriorityQueue.update(frontier, [successor_tile, direction_to_append, successor_cost], successor_cost + heuristic(successor_tile, problem))
    util.raiseNotDefined()


# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch