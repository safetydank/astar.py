"""A generic A* implementation"""

import heapq

UNINITIALIZED = 0
SUCCEEDED     = 2
FAILED        = 3
SEARCHING     = 4

class Node(object):
    """A search node"""

    def __init__(self, state, parent=None, g=0, h=0):
        self.parent = parent
        self.g, self.h = g, h
        self.state = state

    def __cmp__(self, other):
        """Sort by total cost, used to determine open node priority"""
        return cmp(self.f, other.f)

    @property
    def f(self):
        """Total cost is g + h"""
        return self.g + self.h

class AStar(object):
    """Performs A* search on a state-space"""

    def __init__(self):
        self._status = UNINITIALIZED
    
    def _reset(self):
        self._open = []
        self._closed = set()

        self._start = None
        self._goal = None

        self._status = UNINITIALIZED
        self.steps = 0

    def initialize(self, start_state, goal_state):
        """Initialize A* search with start and goal states.  States must 
        support the following interface:

        estimate_cost(goal_state)
            estimate the cost to the given goal_state (heuristic)

        __eq__
            compare two states for equality (probably good to implement __ne__
            as well)

        successors()
            return an iterable of successive states
        """

        self._reset()

        # Create search nodes
        self.start = Node(start_state)
        self.goal  = Node(goal_state)

        # Initialize start node
        self.start.g = 0
        self.start.h = start_state.estimate_cost(self.goal.state)
        self.start.parent = None
        
        heapq.heappush(self._open, self.start)

        self._status = SEARCHING

    def search(self):
        """Return True if search succeeded, False otherwise"""

        while self._status is SEARCHING:
            self.search_step()
            self.steps += 1

        return (self._status == SUCCEEDED)

    def get_solution(self):
        node = self.goal
        solution = [ node ]

        while node.parent is not None:
            solution.append(node.parent)
            node = node.parent

        solution.reverse()

        return solution

    def search_step(self):
        if not self._open:
            self._status = FAILED

        if self._status is not SEARCHING:
            return self._status
        
        node = heapq.heappop(self._open)

        if node.state == self.goal.state:
            self.goal.parent = node.parent
            self._status = SUCCEEDED
        else:
            next_nodes = [ Node(state, parent=node, g=node.g + cost) 
                           for (state, cost) in node.state.successors() ]

            for next in next_nodes:
                # continue if next is already in a queue (q) with less/equal 
                # cost (g)
                cheaper = lambda q: (n for n in q if next.state == n.state 
                                                  and n.g <= next.g)
                if any(cheaper(self._open)) or any(cheaper(self._closed)):
                    continue 

                # check if a worse node is present in open or closed lists
                for nodelist in (self._open, self._closed):
                    # Remove any worse nodes
                    worse = [ n for n in nodelist 
                                if next.state == n.state and n.g > next.g ]
                    for n in worse: 
                        nodelist.remove(n)

                # update order of open priority queue
                heapq.heapify(self._open)

                # update heuristic and add to open queue
                next.h = next.state.estimate_cost(self.goal.state)
                heapq.heappush(self._open, next)

            # push checked node onto closed list
            self._closed.add(node)

        return self._status

