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

    
    

        # idea 1 - I expected the successor to always mean
        # changed the output to always mean the identifier. 
        for successor, direction, stepCost in successors: 
            if successor in visited: 
                continue

            prev[successor] = (state, direction, stepCost) # at this stage, we always "build" a bigger path that will be considered as part of the solution.
            finish_state = find_finish_state(successor)

    
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

        #print(f"move_list: {move_list} len: {len(move_list)}")
        return move_list            



    prev = {}
    visited = set()
    Q = []


    


    start = problem.getStartState()


    Q.append(start)
    

    

    while len(Q) != 0:
        
        working_node = Q.pop(0)
        

        #print(f"Working Node: {working_node}")
        visited.add(working_node)
        if problem.isGoalState(working_node):
            
            return reconstruct_list(prev, working_node)
        
        for successor, direction, stepCost in problem.getSuccessors(working_node): 
            if successor in visited: 
                continue
            # heuristic - I add visited here to allow algorithm to later on not add to the Q
            # visited = already "seen" and considered into the solution will be traversed on the Q. 
            # visited === 'noted'


            #visited.add((x,y))
            prev[successor] = (working_node, direction, stepCost)

            visited.add(successor)
            Q.append(successor)
         
    return []



    util.raiseNotDefined()


from util import PriorityQueue
def uniformCostSearch(problem):
    """Search the node of least total cost first."""
    "*** YOUR CODE HERE ***"

    # Misleading 


    def reconstruct_list(prev, finish_state):
        """
        Creates the move path from the tuple
        """

        # I suspect that thing wrong is made in the new thing 
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

    min_distance = {}

    # Error - I was not using the cumulative cost. for alll paths I want to explore by examining up to a certain point
    # what paths I have explored is. 
    # previously, I was just computing / dequeueing what was the lowest cost edge as a model, but dequqing on lowest cost edge 
    # attatched to a minimum node, on the queue made it ideal. 
    


    start = problem.getStartState()


    Q.push(start, 1) # priority does not matter
    visited.add(start)
    min_distance[start] = 0

    

    while not Q.isEmpty():
    
        working_node = Q.pop()

        #visited.add(working_node) # we define visited now to not mean 'noted' but that it has been dequeued and will not be touched, it has the min value.  
        # this one is added badly this error caused a node to be added multiple times.

        if problem.isGoalState(working_node):
            return reconstruct_list(prev, working_node)
        
        for successor, direction, stepCost in problem.getSuccessors(working_node): 
            #if successor in visited: # change visited to mean that I won't change anything about it. 
            #    continue

            # Check to create new discovered nodes as inf if not discovered
            if successor not in min_distance:
                min_distance[successor] = float('inf')

            # check if there is already a prev add the current node as the highest cost. 
            # I need to handle the special case for having the start as successor.
            if successor not in prev and successor != start: # discovered this as I realized in references that there was an infinite loop, leading to references.
                prev[successor] = (working_node, direction, stepCost)



            # heuristic - I add visited here to allow algorithm to later on not add to the Q
            # visited = already "seen" and considered into the solution will be traversed on the Q. 
            # visited === 'noted'

            # I don't want to immediately change this, on what condition should I add a prev? 
            # 1. if there is no prev -> add a prev. 
            # 2. if there is a prev -> 
            #prev[successor] = (working_node, direction, stepCost)

            assert working_node in min_distance, "Error working_node is not in min distance"

            # problem 1 address the changing prev. 
            # address the decisoin problem.

       

            new_min_distance = stepCost + min_distance[working_node] #--> if there was already a tentative difference, then we never hold tthat under condition 
            # repeating F problem.
            if new_min_distance < min_distance[successor]: 
                 min_distance[successor] = new_min_distance
                 prev[successor] = (working_node, direction, stepCost)
                

           

            # now visited here does not make sense as essentially I am boxing myself in and not allowing further exploration.
            if successor not in visited: 
                # visited == I have seen it and there is now a new value.
                # if I dequeue this "visited" node then I have found the min value.
        
                Q.push(successor, min_distance[successor])
                visited.add(successor)
         
    return []
    util.raiseNotDefined()

def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0


def aStarSearch(problem, heuristic=nullHeuristic):

    print("running:")
    # Figure out how to create thi squeeu.
    def reconstruct_list(prev, finish_state):
        """
        Creates the move path from the tuple
        """

        # I suspect that thing wrong is made in the new thing 
        move_list = []
        


        working_state = finish_state

        # will only add to list if finish_state is not -1 
        while working_state in prev:
                        # insert the move list at 0 to get a working dictionary
                        move_list.insert(0, prev[working_state][1])

                        # Get the previous working key
                        working_state = prev[working_state][0]

        print(move_list)
        return move_list            



    prev = {}
    visited = set()
    Q = PriorityQueue()

    min_distance = {}

    # Error - I was not using the cumulative cost. for alll paths I want to explore by examining up to a certain point
    # what paths I have explored is. 
    # previously, I was just computing / dequeueing what was the lowest cost edge as a model, but dequqing on lowest cost edge 
    # attatched to a minimum node, on the queue made it ideal. 
    


    start = problem.getStartState()


    Q.push(start, 1) # priority does not matter
    visited.add(start)
    min_distance[start] = 0

    

    while not Q.isEmpty():


        if len(min_distance) < 20:
            print(min_distance)
    
        working_node = Q.pop()

        #visited.add(working_node) # we define visited now to not mean 'noted' but that it has been dequeued and will not be touched, it has the min value.  
        # this one is added badly this error caused a node to be added multiple times.

        if problem.isGoalState(working_node):
            
            return reconstruct_list(prev, working_node)
        
        for successor, direction, stepCost in problem.getSuccessors(working_node): 
            #if successor in visited: # change visited to mean that I won't change anything about it. 
            #    continue

            # Check to create new discovered nodes as inf if not discovered
            if successor not in min_distance:
                min_distance[successor] = float('inf')

            # check if there is already a prev add the current node as the highest cost. 
            # I need to handle the special case for having the start as successor.
            if successor not in prev and successor != start: # discovered this as I realized in references that there was an infinite loop, leading to references.
                prev[successor] = (working_node, direction, stepCost)



            # heuristic - I add visited here to allow algorithm to later on not add to the Q
            # visited = already "seen" and considered into the solution will be traversed on the Q. 
            # visited === 'noted'

            # I don't want to immediately change this, on what condition should I add a prev? 
            # 1. if there is no prev -> add a prev. 
            # 2. if there is a prev -> 
            #prev[successor] = (working_node, direction, stepCost)

            assert working_node in min_distance, "Error working_node is not in min distance"

            # problem 1 address the changing prev. 
            # address the decisoin problem.

            
        
            # distance encodes the whole cumulative cost to queue the smallest difference
            new_min_distance = stepCost + min_distance[working_node] #+ heuristic(successor, problem) doesnt seem as convenient to calculate the min distance, true
            
            

            if successor == 'G' and new_min_distance == 10:
                #import pdb
                #pdb.set_trace()
                print(new_min_distance)
                print("successor")
                print(min_distance[successor])

            # repeating F problem.
            if new_min_distance < min_distance[successor]: 
                 min_distance[successor] = new_min_distance
                 prev[successor] = (working_node, direction, stepCost)

                 if successor in visited:
                    # recompute the cumulative cost and add the heuristic

                    guessed_distance = min_distance[successor] + heuristic(successor, problem)

                    Q.update(successor, guessed_distance)

                

           

            # now visited here does not make sense as essentially I am boxing myself in and not allowing further exploration.


            # issue related too visited - i only queue once when I visit with the heuristic. 
            if successor not in visited: 
                # visited == I have seen it and there is now a new value.
                # if I dequeue this "visited" node then I have found the min value.
                if len(min_distance) < 20: 
                    print(f"{min_distance} in loop")
                guessed_distance = min_distance[successor] + heuristic(successor, problem)
                if successor == "A": 
                    print(f"A: {guessed_distance}")
                if successor == "C": 
                     
                     print(f"C: {guessed_distance} heuristic")
                if successor == "D": 
                     print(f"D: {guessed_distance}")
                if successor == 'B': 
                     print(f"B: {guessed_distance}")
                if successor == 'G': 
                     print(min_distance)
                     print(f"{successor} {min_distance[successor]}")
                
                Q.push(successor, guessed_distance) # when G is pushed, it is pushed when the value is 10, but visited essentially prevents it from going again
                visited.add(successor)
         
    return []
    util.raiseNotDefined()


# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
