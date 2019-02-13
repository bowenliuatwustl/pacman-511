# search.py
# ---------
# Licensing Information: Please do not distribute or publish solutions to this
# project. You are free to use and extend these projects for educational
# purposes. The Pacman AI projects were developed at UC Berkeley, primarily by
# John DeNero (denero@cs.berkeley.edu) and Dan Klein (klein@cs.berkeley.edu).
# For more info, see http://inst.eecs.berkeley.edu/~cs188/sp09/pacman.html

"""
In search.py, you will implement generic search algorithms which are called
by Pacman agents (in searchAgents.py).
"""

import util
#from ipdb import set_trace

class SearchProblem:
    """
    This class outlines the structure of a search problem, but doesn't implement
    any of the methods (in object-oriented terminology: an abstract class).

    You do not need to change anything in this class, ever.
    """

    def getStartState(self):
        """
        Returns the start state for the search problem
        """
        util.raiseNotDefined()

    def isGoalState(self, state):
        """
          state: Search state

        Returns True if and only if the state is a valid goal state
        """
        util.raiseNotDefined()

    def getSuccessors(self, state):
        """
          state: Search state

        For a given state, this should return a list of triples,
        (successor, action, stepCost), where 'successor' is a
        successor to the current state, 'action' is the action
        required to get there, and 'stepCost' is the incremental
        cost of expanding to that successor
        """
        util.raiseNotDefined()

    def getCostOfActions(self, actions):
        """
         actions: A list of actions to take

        This method returns the total cost of a particular sequence of actions.  The sequence must
        be composed of legal moves
        """
        util.raiseNotDefined()


def tinyMazeSearch(problem):
    """
    Returns a sequence of moves that solves tinyMaze.  For any other
    maze, the sequence of moves will be incorrect, so only use this for tinyMaze
    """
    '''from ipdb import set_trace
    set_trace()'''
    from game import Directions
    s = Directions.SOUTH
    w = Directions.WEST
    return  [s,s,w,s,w,w,s,w]

def depthFirstSearch(problem):
    """
    Search the deepest nodes in the search tree first
    [2nd Edition: p 75, 3rd Edition: p 87]

    Your search algorithm needs to return a list of actions that reaches
    the goal.  Make sure to implement a graph search algorithm
    [2nd Edition: Fig. 3.18, 3rd Edition: Fig 3.7].

    To get started, you might want to try some of these simple commands to
    understand the search problem that is being passed in:

    print "Start:", problem.getStartState()
    print "Is the start a goal?", problem.isGoalState(problem.getStartState())
    print "Start's successors:", problem.getSuccessors(problem.getStartState())
    """
    "*** YOUR CODE HERE ***"


    from game import Directions
    dir = {'East': Directions.EAST,
           'West': Directions.WEST,
           'South': Directions.SOUTH,
           'North': Directions.NORTH}
    frontier = util.Stack()
    visited = []
    solutions = util.Stack()
    frontier.push(problem.getStartState())
    visited.append(problem.getStartState())
    solutions.push([])
    while frontier:
        vertex = frontier.pop()
        solution = solutions.pop()

        if problem.isGoalState(vertex):
            return solutions
        for successor in problem.getSuccessors(vertex):
            successor_v = successor[0]
            successor_d = successor[1]

            if successor_v not in visited:
                if problem.isGoalState(successor_v):
                    return solution + [dir[successor_d]]
                else:
                    frontier.push(successor_v)
                    solutions.push(solution+[dir[successor_d]])
                    visited.append(successor_v)





def breadthFirstSearch(problem):
    """
    Search the shallowest nodes in the search tree first.
    [2nd Edition: p 73, 3rd Edition: p 82]
    """
    "*** YOUR CODE HERE ***"
    from game import Directions
    dir = {'East': Directions.EAST,
           'West': Directions.WEST,
           'South': Directions.SOUTH,
           'North': Directions.NORTH}
    frontier = util.Queue()
    visited = []
    solutions = util.Queue()
    frontier.push(problem.getStartState())
    visited.append(problem.getStartState())
    solutions.push([])
    while frontier:
        vertex = frontier.pop()
        solution = solutions.pop()


        for successor in problem.getSuccessors(vertex):
            successor_v = successor[0]
            successor_d = successor[1]

            if successor_v not in visited:
                if problem.isGoalState(successor_v):
                    #print solution+[dir[successor_d]]
                    return solution + [dir[successor_d]]
                frontier.push(successor_v)
                solutions.push(solution+[dir[successor_d]])
                #print solution+[dir[successor_d]],successor_v
                visited.append(successor_v)


def uniformCostSearch(problem):
    "Search the node of least total cost first. "
    "*** YOUR CODE HERE ***"
    # how to delete the longer path?

    from game import Directions
    dir = {'East': Directions.EAST,
           'West': Directions.WEST,
           'South': Directions.SOUTH,
           'North': Directions.NORTH}
    frontier = util.PriorityQueue()
    visited = []
    #set_trace()
    # save [vertex, direction, cost] and priority in priority queue
    frontier.push([problem.getStartState(), [], 0], 0 )

    #set_trace()
    while frontier:
        # pop min
        vertex, direction, cost = frontier.pop()
        visited.append(vertex)

        # in case the start state is the goal state
        if problem.isGoalState(vertex):
            return direction

        for successor in problem.getSuccessors(vertex):
            successor_v = successor[0]
            successor_d = successor[1]
            successor_c = successor[2]
            if successor_v not in visited:
                frontier.push([successor_v, direction + [dir[successor_d]], cost+successor_c], cost+successor_c)

def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0

def aStarSearch(problem, heuristic=nullHeuristic):
    "Search the node that has the lowest combined cost and heuristic first."
    "*** YOUR CODE HERE ***"
    # how to delete the longer path?

    from game import Directions
    dir = {'East': Directions.EAST,
           'West': Directions.WEST,
           'South': Directions.SOUTH,
           'North': Directions.NORTH}
    frontier = util.PriorityQueue()
    #from ipdb import set_trace
    #set_trace()
    visited = []
    #set_trace()
    # save [vertex, direction, cost] and priority in priority queue
    # calculate cost use manhaton distance
    frontier.push([problem.getStartState(), [], 0], 0 )

    #set_trace()
    while frontier:
        # pop min
        vertex, direction, cost = frontier.pop()
        visited.append(vertex)

        # in case the start state is the goal state
        if problem.isGoalState(vertex):
            print direction
            return direction

        for successor in problem.getSuccessors(vertex):
            successor_v = successor[0]
            successor_d = successor[1]
            successor_c = successor[2]

            #from ipdb import set_trace
            #set_trace()
            cost = heuristic(successor_v, problem) + successor_c+ problem.getCostOfActions(direction)
            if successor_v not in visited :
                frontier.push([successor_v, direction + [dir[successor_d]], cost], cost)






# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
