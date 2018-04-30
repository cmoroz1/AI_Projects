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

def index(lst,elt):
    for i in range(len(lst)):
        if lst[i] == elt:
            return i
    return -1

def general_search(problem, dataStruct):
    discovered = {} # The keys are the grid locations and the values are the 3-tuples of the node itself
    startLoc = problem.getStartState()
    startNode = (startLoc,None,None) #STORE IN PRIORITY QUEUE AS (location, action, previous node)
    dataStruct.push(startNode)
    while not dataStruct.isEmpty():
        node = dataStruct.pop()
        if(not discovered.has_key(node[0]) or problem.getCostOfActions(path(node)) < problem.getCostOfActions(path(discovered[node[0]]))):
            discovered[node[0]] = node
            if problem.isGoalState(node[0]):
                return path(node)
            for neighbor in problem.getSuccessors(node[0]):
                newNode = (neighbor[0], neighbor[1], node)
                if(not discovered.has_key(newNode[0]) or problem.getCostOfActions(path(node)) < problem.getCostOfActions(path(discovered[node[0]]))):
                    dataStruct.push(newNode)
    util.raiseNotDefined()

def depthFirstSearch(problem):
    """
    Search the deepest nodes in the search tree first.

    Your search algorithm needs to return a list of actions that reaches the
    goal. Make sure to implement a graph search algorithm.

    To get started, you might want to try some of these simple commands to
    understand the search problem that is being passed in:

    print "Start:", problem.getStartState()
    print "Is the start a goal?", problem.isGoalState(problem.getStartState())
    print "Start's successors:", problem.getSuccessors(problem.getStartState())
    """
    stack = util.Stack()
    return general_search(problem,stack)
    # discovered = {}
    # #print "Start:", problem.getStartState()
    # #print "Start's successors:", problem.getSuccessors(problem.getStartState())
    # startLoc = problem.getStartState()
    # stack.push((startLoc,None,None)) #STORE IN STACK AS (location, action, previous node)
    # while not stack.isEmpty():
    #     node = stack.pop()
    #     if not discovered.has_key(node[0]):
    #         discovered[node[0]] = True
    #         if problem.isGoalState(node[0]):
    #             return path(node)
    #         for neighbor in problem.getSuccessors(node[0]):
    #             newNode = (neighbor[0], neighbor[1], node)
    #             if(not discovered.has_key(newNode[0])):
    #                 stack.push(newNode)
    # util.raiseNotDefined()

# node is a 3-tuple made of (location, action, previous node)
# the root node is (problem.getStartState(), None, None)
def path(node):
    actions = []
    current = node
    while current[2] != None:
        actions.append(current[1])
        current = current[2]
    actions.reverse()
    return actions


def breadthFirstSearch(problem):
    """Search the shallowest nodes in the search tree first."""
    queue = util.Queue()
    return general_search(problem,queue)
    # discovered = {}
    # startLoc = problem.getStartState()
    # queue.push((startLoc,None,None)) #STORE IN QUEUE AS (location, action, previous node)
    # while not queue.isEmpty():
    #     node = queue.pop()
    #     if not discovered.has_key(node[0]):
    #         discovered[node[0]] = True
    #         if problem.isGoalState(node[0]):
    #             return path(node)
    #         for neighbor in problem.getSuccessors(node[0]):
    #             newNode = (neighbor[0], neighbor[1], node)
    #             if(not discovered.has_key(newNode[0])):
    #                 queue.push(newNode)
    # util.raiseNotDefined()

def uniformCostSearch(problem):
    """Search the node of least total cost first."""
    pq = util.PriorityQueue()
    discovered = {}
    startLoc = problem.getStartState()
    startNode = (startLoc,None,None)
    discovered[startNode[0]] = startNode
    pq.push(startNode,0)
    while not pq.isEmpty():
        node = pq.pop()
        if problem.isGoalState(node[0]):
            return path(node)
        for neighbor in problem.getSuccessors(node[0]):
            newNode = (neighbor[0], neighbor[1], node)
            if(not discovered.has_key(newNode[0])):
                discovered[newNode[0]] = newNode
                pq.push(newNode,problem.getCostOfActions(path(newNode)))
            else:
            # newNode[0] has already been found previously, so check if this one is easier to get to
                if(problem.getCostOfActions(path(newNode)) < problem.getCostOfActions(path(discovered[newNode[0]]))):
                    discovered[newNode[0]] = newNode
                    pq.push(newNode,problem.getCostOfActions(path(newNode)))
    util.raiseNotDefined()

def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0

def aStarSearch(problem, heuristic=nullHeuristic):
    """Search the node that has the lowest combined cost and heuristic first."""
    pq = util.PriorityQueue()
    discovered = {} # The keys are the grid locations and the values are the 3-tuples of the node itself
    startLoc = problem.getStartState()
    startNode = (startLoc,None,None) #STORE IN PRIORITY QUEUE AS (location, action, previous node)
    pq.push(startNode,0)
    while not pq.isEmpty():
        node = pq.pop()
        new_cost = problem.getCostOfActions(path(node)) + heuristic(node[0],problem)
        if(discovered.has_key(node[0])):
            old_cost = problem.getCostOfActions(path(discovered[node[0]])) + heuristic(node[0],problem)
        else:
            old_cost = new_cost + 1
        if(not discovered.has_key(node[0]) or new_cost < old_cost):
            discovered[node[0]] = node
            if problem.isGoalState(node[0]):
                return path(node)
            for neighbor in problem.getSuccessors(node[0]):
                newNode = (neighbor[0], neighbor[1], node)
                new_cost = problem.getCostOfActions(path(newNode)) + heuristic(newNode[0],problem)
                if(discovered.has_key(newNode[0])):
                    old_cost = problem.getCostOfActions(path(discovered[newNode[0]])) + heuristic(newNode[0],problem)
                else:
                    old_cost = new_cost + 1
                if(not discovered.has_key(newNode[0]) or new_cost < old_cost):
                    pq.push(newNode,new_cost)
    util.raiseNotDefined()


# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
