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

    print("Start:", problem.getStartState())
    print("Is the start a goal?", problem.isGoalState(problem.getStartState()))
    print("Start's successors:", problem.getSuccessors(problem.getStartState()))
    """
    "*** YOUR CODE HERE ***"
    # TODO: Find the representation for how the game state is recorded on the AI, find useful tools for solving
    # and getting the game state and representation.
    # Heuristic direction 1 - How are the states encoded? how can I use it
    # found the start state + how it works, how can I use it, what is the encoding? --> found out that it is a grid
    # full foudn the use of the obejct + knowledge of the interface.

    # TODO: Build an algorithm that prioritizes searching as deeply as possible finding the solution. - Done
    # - heuristic 1 - devise a way to find the goal node first w/o creating a path.

    # model - assume unvisited
    # check if its the goal if not then get the unvisited
    # for each node check if it is unvisited if not check the path.


    # define a set, to define all visited nodes, to ensure that no path is visted more than once.
    visited = set()
    prev = {}
    

    

    
    

    def find_finish_state(state):
        """ 
        Uses DFS to find the finish node, while iteratively computing the prev node to keep track of paths.         
        """

        visited.add(state)
        
        if problem.isGoalState(state): 
            return state
        

        # filler funciton to revrse succedssors for demonstration purposes of possible paths to explore
        # not a necessary component to the function only reorders the successors being explored DFS.
        successors = problem.getSuccessors(state)
        successors.reverse()

        for (x,y), direction, stepCost in successors: 
            if (x,y) in visited: 
                continue

            prev[(x,y)] = (state, direction, stepCost) # at this stage, we always "build" a bigger path that will be considered as part of the solution.
            finish_state = find_finish_state((x,y))

    
            if finish_state != -1: 
                

                return finish_state
            

        # nothing found in the other paths.
        return -1
    

    start = problem.getStartState() 
    finish_state = find_finish_state(start)


    # Reconstruct the ideal path by using prev array.
    move_list = []

    working_state = finish_state

    # will only add to list if finish_state is not -1 
    while working_state in prev:
                    # insert the move list at 0 to get a working dictionary
                    move_list.insert(0, prev[working_state][1])

                    # Get the previous working key
                    working_state = prev[working_state][0]

    return move_list            

            
    print(f"finish statee: {finish_state}")






    from game import Directions
    return [Directions.WEST]


    util.raiseNotDefined()

def breadthFirstSearch(problem):
    """Search the shallowest nodes in the search tree first."""
    "*** YOUR CODE HERE ***"


    

    def reconstruct_list(prev, finish_state):
        """
        Creates the move path from the tuple
        """
        move_list = []

        working_state = finish_state

        # will only add to list if finish_state is not -1 
        while working_state in prev:
                        # insert the move list at 0 to get a working dictionary
                        move_list.insert(0, prev[working_state][1])

                        # Get the previous working key
                        working_state = prev[working_state][0]

        print(f"move_list: {move_list} len: {len(move_list)}")
        return move_list            



    prev = {}
    visited = set()
    Q = []


    


    start = problem.getStartState()


    Q.append(start)
    

    

    while len(Q) != 0:
        
        working_node = Q.pop(0)
        

        print(f"Working Node: {working_node}")
        visited.add(working_node)
        if problem.isGoalState(working_node):
            
            return reconstruct_list(prev, working_node)
        
        for (x,y), direction, stepCost in problem.getSuccessors(working_node): 
            if (x,y) in visited: 
                continue
            # heuristic - I add visited here to allow algorithm to later on not add to the Q
            # visited = already "seen" and considered into the solution will be traversed on the Q. 
            # visited === 'noted'


            #visited.add((x,y))
            prev[(x,y)] = (working_node, direction, stepCost)

            visited.add((x,y))
            Q.append((x,y))
         
    return []



    util.raiseNotDefined()


from util import PriorityQueue
def uniformCostSearch(problem):
    """Search the node of least total cost first."""
    "*** YOUR CODE HERE ***"


    def reconstruct_list(prev, finish_state):
        """
        Creates the move path from the tuple
        """
        move_list = []

        working_state = finish_state

        # will only add to list if finish_state is not -1 
        while working_state in prev:
                        # insert the move list at 0 to get a working dictionary
                        move_list.insert(0, prev[working_state][1])

                        # Get the previous working key
                        working_state = prev[working_state][0]

        return move_list            



    prev = {}
    visited = set()
    Q = PriorityQueue()


    


    start = problem.getStartState()


    Q.push(start, 1) # priority does not matter
    visited.add(start)

    

    while not Q.isEmpty():

        working_node = Q.pop()
        
        if problem.isGoalState(working_node):
            
            return reconstruct_list(prev, working_node)
        
        for (x,y), direction, stepCost in problem.getSuccessors(working_node): 
            if (x,y) in visited: 
                continue
            # heuristic - I add visited here to allow algorithm to later on not add to the Q
            # visited = already "seen" and considered into the solution will be traversed on the Q. 
            # visited === 'noted'

            prev[(x,y)] = (working_node, direction, stepCost)

            visited.add((x,y))
            Q.push((x,y), stepCost)
         
    return []
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


    # when visiting nodes, rewrite my UCS function to incorporate greedy information. 


    def reconstruct_list(prev, finish_state):
        """
        Creates the move path from the tuple
        """
        move_list = []

        working_state = finish_state

        # will only add to list if finish_state is not -1 
        while working_state in prev:
                        # insert the move list at 0 to get a working dictionary
                        move_list.insert(0, prev[working_state][1])

                        # Get the previous working key
                        working_state = prev[working_state][0]

        return move_list            



    prev = {}
    visited = set()
    Q = PriorityQueue()


    


    start = problem.getStartState()


    Q.push(start, 1) # priority does not matter
    visited.add(start)

    

    while not Q.isEmpty():

        working_node = Q.pop()
        
        if problem.isGoalState(working_node):
            
            return reconstruct_list(prev, working_node)
        
        for (x,y), direction, stepCost in problem.getSuccessors(working_node): 
            if (x,y) in visited: 
                continue
            # heuristic - I add visited here to allow algorithm to later on not add to the Q
            # visited = already "seen" and considered into the solution will be traversed on the Q. 
            # visited === 'noted'

            prev[(x,y)] = (working_node, direction, stepCost)

            visited.add((x,y))

            # add the cost + the heuristical estimate between the goal the position.

            estimated_distance = stepCost + heuristic((x,y), problem)    
            Q.push((x,y), estimated_distance)
         
    return []
    util.raiseNotDefined()


# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
