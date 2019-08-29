# -*- coding: utf-8 -*-
"""
Created on Mon Apr  8 18:47:22 2019

@author: colin
"""
from utils import (
    is_in, argmin, argmax, argmax_random_tie, probability, weighted_sampler,
    memoize, print_table, open_data, Stack, FIFOQueue, PriorityQueue, name,
    distance
)

from collections import defaultdict
import math
import random
import sys
import bisect

infinity = float('inf')

class Problem(object):

    """The abstract class for a formal problem.  You should subclass
    this and implement the methods actions and result, and possibly
    __init__, goal_test, and path_cost. Then you will create instances
    of your subclass and solve them with the various search functions."""

    def __init__(self, initial, goal=None):
        """The constructor specifies the initial state, and possibly a goal
        state, if there is a unique goal.  Your subclass's constructor can add
        other arguments."""
        self.initial = initial
        self.goal = goal

    def actions(self, state):
        """Return the actions that can be executed in the given
        state. The result would typically be a list, but if there are
        many actions, consider yielding them one at a time in an
        iterator, rather than building them all at once."""
        raise NotImplementedError

    def result(self, state, action):
        """Return the state that results from executing the given
        action in the given state. The action must be one of
        self.actions(state)."""
        raise NotImplementedError

    def goal_test(self, state):
        """Return True if the state is a goal. The default method compares the
        state to self.goal or checks for state in self.goal if it is a
        list, as specified in the constructor. Override this method if
        checking against a single self.goal is not enough."""
        if isinstance(self.goal, list):
            return state in self.goal
        else:
            return state == self.goal

    def path_cost(self, c, state1, action, state2):
        """Return the cost of a solution path that arrives at state2 from
        state1 via action, assuming cost c to get up to state1. If the problem
        is such that the path doesn't matter, this function will only look at
        state2.  If the path does matter, it will consider c and maybe state1
        and action. The default method costs 1 for every step in the path."""
        return c + 1

    def value(self, state):
        """For optimization problems, each state has a value.  Hill-climbing
        and related algorithms try to maximize this value."""
        raise NotImplementedError
        
# ______________________________________________________________________________


class Node:

    """A node in a search tree. Contains a pointer to the parent (the node
    that this is a successor of) and to the actual state for this node. Note
    that if a state is arrived at by two paths, then there are two nodes with
    the same state.  Also includes the action that got us to this state, and
    the total path_cost (also known as g) to reach the node.  Other functions
    may add an f and h value; see best_first_graph_search and astar_search for
    an explanation of how the f and h values are handled. You will not need to
    subclass this class."""

    def __init__(self, state, parent=None, action=None, path_cost=0):
        """Create a search tree Node, derived from a parent by an action."""
        self.state = state
        self.parent = parent
        self.action = action
        self.num_expanded = 0
        self.num_explored = 0
        self.path_cost = path_cost
        self.depth = 0
        
        self.max_frontier = 0
        if parent:
            self.depth = parent.depth + 1
            
    def varied_expand(self, problem, ref):
        """List the nodes reachable in one step from this node.
        apende a lista das acoes as acoes possiveis neste node, 
        seguindo o ordem da prioridade ja establecido"""
        actions = []
        for action in ref:
            if action in problem.actions(self.state):
                actions.append(action)
            
        return [self.child_node(problem, action) for action in actions]

    def __repr__(self):
        return "<Node {}>".format(self.state)

    def __lt__(self, node):
        return self.state < node.state

    def expand(self, problem):
        """List the nodes reachable in one step from this node."""
        return [self.child_node(problem, action)
                for action in problem.actions(self.state)]
        


    def child_node(self, problem, action):
        """[Figure 3.10]"""
        next = problem.result(self.state, action)
        return Node(next, self, action,
                    problem.path_cost(self.path_cost, self.state,
                                      action, next))

    def solution(self):
        """Return the sequence of actions to go from the root to this node."""
        return [node.action for node in self.path()[1:]]

    def path(self):
        """Return a list of nodes forming the path from the root to this node."""
        node, path_back = self, []
        while node:
            path_back.append(node)
            node = node.parent
        return list(reversed(path_back))

    # We want for a queue of nodes in breadth_first_search or
    # astar_search to have no duplicated states, so we treat nodes
    # with the same state as equal. [Problem: this may not be what you
    # want in other contexts.]

    def __eq__(self, other):
        return isinstance(other, Node) and self.state == other.state

    def __hash__(self):
        return hash(self.state)
    

def graph_search(problem, frontier):
    """Search through the successors of a problem to find a goal.
    The argument frontier should be an empty queue.
    If two paths reach a state, only use the first one. [Figure 3.7]"""
    frontier.append(Node(problem.initial))
    explored = set()
    while frontier:
        node = frontier.pop()
        if problem.goal_test(node.state):
            return node
        explored.add(node.state)
        frontier.extend(child for child in node.expand(problem)
                        if child.state not in explored and
                        child not in frontier)
    return None

def graph_search_stats(problem, frontier):
    """Search through the successors of a problem to find a goal.
    The argument frontier should be an empty queue.
    If two paths reach a state, only use the first one. [Figure 3.7]"""
    max_frontier = 0
    num_explored = 1
    num_expanded= 0
    frontier.append(Node(problem.initial))
    explored = set()
    while frontier:
        if len(frontier) > max_frontier:
            max_frontier = len(frontier)
        node = frontier.pop()
        if problem.goal_test(node.state):
            node.max_frontier = max_frontier
            node.num_explored = num_explored
            node.num_expanded = len(explored) +1
            num_expanded = len(explored) +1
            return node
        explored.add(node.state)
        for child in node.expand(problem):
            num_explored = num_explored +1
            if child.state not in explored and child not in frontier:
                frontier.append(child)
    return [None, num_explored, num_expanded, max_frontier]

def graph_search_pedagogica(problem, frontier):
    """Search through the successors of a problem to find a goal.
    The argument frontier should be an empty queue.
    If two paths reach a state, only use the first one. [Figure 3.7]"""
    i = 0
    frontier.append(Node(problem.initial))
    explored = set()
    while frontier:
        node = frontier.pop()
        print("Passo:", i)
        print("Removido da frontiera:", "\n", node.state)
        print("Estado a expandir:", "\n", node.state)
        print("Successores: \n") 
        for child in node.expand(problem):
            print(child.state)
        for child in node.expand(problem):
            if child.state in explored:
                print("O nó filho que contém o estado", child.state, "não vai ser visitado porque o estado que o nó representa já foi explorado.")
            if child in frontier:
                print("O nó filho que contém o estado", child.state, "já é membro da frontiera.")
        if problem.goal_test(node.state):
            return node
        explored.add(node.state)
        for child in node.expand(problem):
            if child.state not in explored and child not in frontier:
                frontier.extend([child])
                print("Adicionado a fronteira:", child.state)
        print(" \n \n \n")
        i = i +1
    return None

def graph_search_varied_actions_stats(problem, frontier):
    """Search through the successors of a problem to find a goal.
    The argument frontier should be an empty queue.
    If two paths reach a state, only use the first one. [Figure 3.7]"""
    res = None
    actions_reference = ['N','S','E','W','NE','SE', 'NW','SW'] #establece um ordem da prioridade das acoes numa
    random.shuffle(actions_reference)                          #instancia da procura
    max_frontier = 0
    num_explored = 1
    frontier.append(Node(problem.initial))
    explored = set()
    while frontier:
        if len(frontier) > max_frontier:
            max_frontier = len(frontier)
        node = frontier.pop()
        if problem.goal_test(node.state):
            node.max_frontier = max_frontier
            node.num_explored = num_explored
            node.num_expanded = len(explored) +1
            num_expanded = len(explored) +1
            res = node
            return node
        explored.add(node.state)
        for child in node.varied_expand(problem, actions_reference):
            num_explored = num_explored +1
            if child.state not in explored and child not in frontier:
                frontier.append(child)
    return [res, num_explored, num_expanded, max_frontier]

def depth_first_graph_search_stats(problem):
    """Search the deepest nodes in the search tree first."""
    return graph_search_stats(problem, Stack())

def depth_first_graph_varied_actions_search_stats(problem):
    """Search the deepest nodes in the search tree first."""
    return graph_search_varied_actions_stats(problem, Stack())

def depth_first_graph_search(problem):
    """Search the deepest nodes in the search tree first."""
    return graph_search(problem, Stack())


def breadth_first_search_stats(problem):
    """[Figure 3.11]"""
    res = None 
    expanded = 0
    max_frontier = 0
    node = Node(problem.initial)
    explored = set()
    if problem.goal_test(node.state):
        node.num_expanded = expanded
        node.max_frontier = max_frontier
        node.num_explored = len(explored)
        num_explored = len(explored)
        res = node
        return node
    frontier = FIFOQueue()
    frontier.append(node)
    if len(frontier) > max_frontier:
        max_frontier = len(frontier)
    while frontier:
        node = frontier.pop()
        explored.add(node.state)
        for child in node.expand(problem):
            if child.state not in explored and child not in frontier:
                if problem.goal_test(child.state):
                    return child
                frontier.append(child)
        expanded = expanded + 1
    return [res, num_explored, expanded, max_frontier]

def breadth_first_search(problem):
    """[Figure 3.11]"""
    node = Node(problem.initial)
    if problem.goal_test(node.state):
        return node
    frontier = FIFOQueue()
    frontier.append(node)
    explored = set()
    while frontier:
        node = frontier.pop()
        explored.add(node.state)
        for child in node.expand(problem):
            if child.state not in explored and child not in frontier:
                if problem.goal_test(child.state):
                    return child
                frontier.append(child)
    return None


def best_first_graph_search(problem, f):
    """Search the nodes with the lowest f scores first.
    You specify the function f(node) that you want to minimize; for example,
    if f is a heuristic estimate to the goal, then we have greedy best
    first search; if f is node.depth then we have breadth-first search.
    There is a subtlety: the line "f = memoize(f, 'f')" means that the f
    values will be cached on the nodes as they are computed. So after doing
    a best first search you can examine the f values of the path returned."""
    f = memoize(f, 'f')
    node = Node(problem.initial)
    if problem.goal_test(node.state):
        return node
    frontier = PriorityQueue(min, f)
    frontier.append(node)
    explored = set()
    while frontier:
        node = frontier.pop()
        if problem.goal_test(node.state):
            return node
        explored.add(node.state)
        for child in node.expand(problem):
            if child.state not in explored and child not in frontier:
                frontier.append(child)
            elif child in frontier:
                incumbent = frontier[child]
                if f(child) < f(incumbent):
                    del frontier[incumbent]
                    frontier.append(child)
    return None

def best_first_graph_search_stats(problem, f):
    """Search the nodes with the lowest f scores first.
    You specify the function f(node) that you want to minimize; for example,
    if f is a heuristic estimate to the goal, then we have greedy best
    first search; if f is node.depth then we have breadth-first search.
    There is a subtlety: the line "f = memoize(f, 'f')" means that the f
    values will be cached on the nodes as they are computed. So after doing
    a best first search you can examine the f values of the path returned."""
    res = None
    f = memoize(f, 'f')
    num_expanded = 0
    num_explored = 1
    max_frontier = 0
    explored = set()
    frontier = PriorityQueue(min, f)
    node = Node(problem.initial)
    if problem.goal_test(node.state):
        node.max_frontier = max_frontier
        node.num_explored = num_explored
        node.num_expanded = len(explored) + 1
        res = node
        return node
    frontier.append(node)
    while frontier:
        if len(frontier) > max_frontier:
            max_frontier = len(frontier)
        node = frontier.pop()
        #print(node.state)
        if problem.goal_test(node.state):
            node.max_frontier = max_frontier
            node.num_explored = num_explored
            node.num_expanded = len(explored) + 1
            num_expanded = len(explored) + 1
            return node
        explored.add(node.state)
        for child in node.expand(problem):
            num_explored = num_explored +1
            if child.state not in explored and child not in frontier:
                frontier.append(child)
            elif child in frontier:
                incumbent = frontier[child]
                if f(child) < f(incumbent):
                    del frontier[incumbent]
                    frontier.append(child)
              
    
    return [res, num_explored, num_expanded, max_frontier]

def best_first_graph_search_pedagogica(problem, f):
    """Search the nodes with the lowest f scores first.
    You specify the function f(node) that you want to minimize; for example,
    if f is a heuristic estimate to the goal, then we have greedy best
    first search; if f is node.depth then we have breadth-first search.
    There is a subtlety: the line "f = memoize(f, 'f')" means that the f
    values will be cached on the nodes as they are computed. So after doing
    a best first search you can examine the f values of the path returned."""
    i = 0
    f = memoize(f, 'f')
    expanded = 0
    max_frontier = 0
    node = Node(problem.initial)
    if problem.goal_test(node.state):
        return node
    frontier = PriorityQueue(min, f)
    frontier.append(node)
    explored = set()
    while frontier:
        if len(frontier) > max_frontier:
            max_frontier = len(frontier)
        node = frontier.pop()
        print("Passo:", i)
        print("Removido da frontiera:", "\n", node.state)
        print("Estado a expandir:", "\n", node.state)
        print("Successores: \n") 
        for child in node.expand(problem):
            print(child.state)
        for child in node.expand(problem):
            if child.state in explored:
                print("O nó filho que contém o estado", child.state, "não vai ser visitado porque o estado que o nó representa já foi explorado.")
            if child in frontier:
                print("O nó filho que contém o estado", child.state, "já é membro da frontiera.")
                
        if problem.goal_test(node.state):
            print("O estado: \n", node.state, "é um estado goal.")
            node.max_frontier = max_frontier
            node.num_expanded = expanded
            node.num_explored = len(explored)
            print("Solução:", node.solution)
            print("Custo:", node.path_cost)
            print("Nós expandidos:", expanded)
            print("Nós visitados:", len(explored))
            return node
        explored.add(node.state)

        for child in node.expand(problem):
            if child.state not in explored and child not in frontier:
                frontier.append(child)
                print(child.state, "foi posto na fronteira")
            elif child in frontier:
                incumbent = frontier[child]
                if f(child) < f(incumbent):
                    del frontier[incumbent]
                    frontier.append(child)
                    print("Adicionado a fronteira:", child.state)
        print(" \n \n \n")
        i = i +1
        expanded = expanded +1           
    
    return None



def uniform_cost_search(problem):
    """[Figure 3.14]"""
    return best_first_graph_search(problem, lambda node: node.path_cost)

def uniform_cost_search_pedagogica(problem):
    """[Figure 3.14]"""
    return best_first_graph_search_pedagogica(problem, lambda node: node.path_cost)

def uniform_cost_search_stats(problem):
    """[Figure 3.14]"""
    return best_first_graph_search_stats(problem, lambda node: node.path_cost)


def depth_limited_search_stats(problem, limit=50):
    """[Figure 3.17]"""
    def recursive_dls_stats(node, problem, limit):
        num_explored = 1
        max_frontier = 0
        if problem.goal_test(node.state):
            node.num_explored = num_explored
            node.max_frontier = max_frontier
            return node
        elif limit == 0:
            return 'cutoff'
        else:
            cutoff_occurred = False
            if len(node.expand(problem)) >max_frontier:
                max_frontier = len(node.expand(problem))
            for child in node.expand(problem):
                num_explored = num_explored +1
                result = recursive_dls_stats(child, problem, limit - 1)
                if result == 'cutoff':
                    cutoff_occurred = True
                elif result is not None:
                    return result
            return 'cutoff' if cutoff_occurred else None

    # Body of depth_limited_search:
    return recursive_dls_stats(Node(problem.initial), problem, limit)


def iterative_deepening_search_stats(problem):
    """[Figure 3.18]"""
    for depth in range(sys.maxsize):
        result = depth_limited_search_stats(problem, depth)
        if result != 'cutoff':
            return result
        
def depth_limited_search(problem, limit=50):
    """[Figure 3.17]"""
    def recursive_dls(node, problem, limit):
        if problem.goal_test(node.state):
            return node
        elif limit == 0:
            return 'cutoff'
        else:
            cutoff_occurred = False
            for child in node.expand(problem):
                result = recursive_dls(child, problem, limit - 1)
                if result == 'cutoff':
                    cutoff_occurred = True
                elif result is not None:
                    return result
            return 'cutoff' if cutoff_occurred else None

    # Body of depth_limited_search:
    return recursive_dls(Node(problem.initial), problem, limit)


def iterative_deepening_search(problem):
    """[Figure 3.18]"""
    for depth in range(sys.maxsize):
        result = depth_limited_search(problem, depth)
        if result != 'cutoff':
            return result
        
        
greedy_best_first_graph_search = best_first_graph_search

best_first_graph_search = greedy_best_first_graph_search

greedy_best_first_graph_search_pedagogica = best_first_graph_search_pedagogica

greedy_best_first_graph_search_stats = best_first_graph_search_stats
# Greedy best-first search is accomplished by specifying f(n) = h(n).


def astar_search_stats(problem, h=None):
    """A* search is best-first graph search with f(n) = g(n)+h(n).
    You need to specify the h function when you call astar_search, or
    else in your Problem subclass."""
    h = memoize(h or problem.h, 'h')
    return best_first_graph_search_stats(problem, lambda n: n.path_cost + h(n))


def astar_search(problem, h=None):
    """A* search is best-first graph search with f(n) = g(n)+h(n).
    You need to specify the h function when you call astar_search, or
    else in your Problem subclass."""
    h = memoize(h or problem.h, 'h')
    return best_first_graph_search(problem, lambda n: n.path_cost + h(n))

def astar_search_pedagogica(problem, h=None):
    """A* search is best-first graph search with f(n) = g(n)+h(n).
    You need to specify the h function when you call astar_search, or
    else in your Problem subclass."""
    h = memoize(h or problem.h, 'h')
    return best_first_graph_search_pedagogica(problem, lambda n: n.path_cost + h(n))

def breadth_first_graph_search(problem):
    """Search the shallowest nodes in the search tree first."""
    return graph_search(problem, FIFOQueue())


def breadth_first_graph_search_stats(problem):
    """Search the shallowest nodes in the search tree first."""
    return graph_search_stats(problem, FIFOQueue())




# ______________________________________________________________________________



    
    
def h1(Node): 
    """uma heuristica que volta o numero de pastilhas encima
    de pacman. Como o custo de mover-se encima é maior do que o custo de
    mover-se abaixo, um estado com menos pastilhas encima de pacman é preferivel"""
    y = Node.state.pacmanpos[1]
    l=0
    for p in Node.state.pastilhas:
        if p.y > y:
            l = l+1
    
    return l
    
    
    
def h2(node):
    """uma heuristica que volta a distancia manhattan da pastilha mas perta
    a pacman."""
    distancia = node.state.lenx + node.state.leny
        
    for p in node.state.pastilhas:
        manhattan_distance_x = abs(node.state.pacmanpos[0] -p.x)
        manhattan_distance_y = abs(node.state.pacmanpos[1] - p.y)
        manhattan_distance1 = manhattan_distance_x + manhattan_distance_y
        if manhattan_distance1 < distancia:
            distancia = manhattan_distance1
                
    return distancia

def h(node):
    """volta o valor da heurstica do problema de grafo.
    foi necessario definir esta funcao aqui"""
    hD = {'I':7, 'A':2,'B':3,'C':1,'D':5,'F':0}
    return hD[node.state]

def h_romenia(node):
    hromenia = {'Arad':366, 'Bucharest':0,'Craiova':160,'Dobreta':242,'Eforie':161,'Fagaras':178,'Giurgiu':77,'Hirsova':151,'Iasi':226,'Lugoj':244,'Mehadia':241, 'Neamt':234,'Oradea':380,'Pitesti':98,'RimnicuVilcea':193,'Sibiu':253,'Timisoara':329,'Urziceni':80,'Vaslui':199,'Zerind':374}
    return hromenia[node.state]

def h_espelho(node):
    hespelho= {0:19.5, 1:18, 2:18, 3:17, 4:16, 5:16, 6:17, 7:15.7,8:15,9:14.8,10:14.2,11:14.2,12:14.7,13:15,14:15.7, 15:15.2, 16:14.8, 17:14.6, 18:14.2, 19:14, 20:13.6, 21:13.4, 22:13, 23:13, 24:13.4, 25:13.6, 26:14, 27:14.2, 28:14.6, 29:14.8, 30:15.2, 31:11.2, 32:11.2, 33:9.4, 34:8.4, 35:8.2, 36:7.2, 37:7,38:6.4,39:6.2,40:6, 41:6, 42:6.2, 43:6.4, 44:7, 45:7.2, 46:8.2, 47:8.4, 48:9.4, 49:8.5, 50:7, 51:6, 52:5.5 , 53:5.5, 54:6, 55:7, 56:8.5, 57:6.8,58:4.5,59:4.5, 60: 6.8, 61:120, 62:120, 63:0}
    return hespelho[int(node.state)]

def h_pistola(node):
    hpistola = {'A':8, 'B':9, 'C':10, 'D':6, 'E':9, 'F':8, 'G':5, 'H':5, 'I':3, 'J':0, 'K':0, 'L':1, 'M':1}
    return hpistola[node.state]