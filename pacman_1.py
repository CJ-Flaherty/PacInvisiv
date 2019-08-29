# -*- coding: utf-8 -*-
"""
Created on Sun Mar 31 11:29:25 2019

@author: colin



            
        
        
        
        
        
        
        
        
    
"""
from search_pacman import *

from random import randrange

from copy import deepcopy

from Graphos import *

import time

import csv

import numpy as np

random.seed(3)


def GerarEstado(dim, x = None, y = None, o = None, p = None):
    """Um metedo que gera estados aleatorios do problema Pacman. É necesario
    passar um argumento dim, que é a dimensão maximo da area. Podes especificar as dimensões,
    numero de obstaculos, e numero de pastilhas se quiser. Vai ser colocados em posições 
    aleatorias. Se quiser gerar um estado especifico, é necessario crear manualmente uma
    instancia do classe EstadoPacman"""
    
    if x == None:
        lenx = randrange(1,dim)
    else:
        lenx= x
        
    if y == None:
        leny = randrange(1,dim)
    else:
        leny= y
        
    if o == None:
         numobs = randrange(lenx*leny)//3
    else:
        numobs = o
    if p == None:
        numpas = randrange((lenx*leny)-(3*numobs))
        
    else:
        numpas = p
    

    pos = [randrange(1,lenx+1)-1, randrange(1,leny+1)-1] #colocar o pacman numa posição aleatoria
    
    
    L = [] #initializar a lista das casas que temos que anular
    
    


    i0 = [[pos[0] - 1, pos[1] - 1], True]
    L.append(i0)
    
    i1 = [[pos[0], pos[1] - 1], True]
    L.append(i1)
    
    i2 = [[pos[0] + 1, pos[1] - 1], True]
    L.append(i2)
    
    i3 = [[pos[0] - 1, pos[1]], True]
    L.append(i3)
    
    i4 = [[pos[0] + 1, pos[1]], True]
    L.append(i4)
    
    i5 = [[pos[0] - 1, pos[1] + 1], True]
    L.append(i5)
    
    i6 = [[pos[0], pos[1] + 1], True]
    L.append(i6)
    
    i7 = [[pos[0] + 1, pos[1] + 1], True]
    L.append(i7)
    
    i8 = [[pos[0], pos[1]], False]
    L.append(i8)
    


        
    e = EstadoPacman(lenx, leny, numobs, numpas, pos, L) #initializar o estado
    
    pastilhas = [] #pôr as pastilhas em posições aleatorias
    for i in range(numpas+1):
            coords = (randrange(lenx), randrange(leny))
            while( any((p.x,p.y) == coords for p in pastilhas) or coords == pos):
                coords = (randrange(lenx), randrange(leny))
            p = pastilha(coords[0], coords[1])
            pastilhas.append( p )
            
    e.pastilhas = pastilhas
    
    obs = [] #pôr os obstaculos em posições aleatorias
    for i in range(numobs+1):
            coords = (randrange(lenx+1), randrange(leny+1))
            while( coords in e.obstaculos or any((p.x,p.y) == coords for p in pastilhas) or coords == (pos[0],pos[1])):
                coords = (randrange(lenx), randrange(leny))
            obs.append( coords )
            
    e.obstaculos = obs
    
    return e
    
    
    
    
   

class pastilha():
    """Um objeto pastilha, que tem posicao e saude"""
    def __init__(self, x,y):
        self.x = x
        self.y = y
        self.pos = (x,y)
        self.health = 2
        
    def __str_(self):
        return self.x, self.y, self.health
        
    def __eq__(self, other):
        return self.pos == other.pos and self.health == other.health
        
    def health(self):
        return self.health
    


class EstadoPacman():
    """O classe do Estados, esta initializado so com numero de pastilhas e numero de obstaculos,
    para facilitar a generacao de estados aleatorios. pode colocar pastilhas, o pacman, e obstaculos
    numa posicao particular depois de initializar o estado se quiser"""
    def __init__(self, x, y, o, p,pos, L):
        self.lenx = x
        self.leny= y
        self.numobs= o
        self.numpas = p
        self.pastilhas = [] 
        self.obstaculos = []
        self.pacmanpos = pos
        self.L = L
        
    def display(self):
        pas = ''
        for p in self.pastilhas:
            pas = pas + str(p.x) +str(p.y)+ str(p.health) + "\n"
        print("lenx: " ,self.lenx)
        print("leny:", self.leny)
        print("obstaculos: ",str(self.obstaculos))
        print("pastilhas:" , pas)
        print("pacmanpos:", self.pacmanpos[0],self.pacmanpos[1])
        print("L:", str(self.L))
        
        
    def __eq__(self, state):
        return self.pastilhas == state.pastilhas and self.obstaculos == state.obstaculos and self.pacmanpos[0] == state.pacmanpos[0] and self.pacmanpos[1] == state.pacmanpos[1] and self.L == state.L 

            
    def __str__(self):
        s= ""
        for i in range(self.leny):
            for p in range(self.lenx):
                c = "-"
                if (p,i) in self.obstaculos:
                    c = "o"
                if [p,i] == self.pacmanpos:
                    c = "0"
                for j in self.pastilhas:
                    if j.x == p and j.y == i and j.health > 0:
                        c = "p"
                        if [p,i] == self.pacmanpos:
                            c = "Q"
                   
                s = s +c
            s = s + "\n"
            
        return s 
    
    def __hash__(self):
        return hash(str(self))
    
    def __lt__(self,state):
        return self.numpas < state.numpas
    
    def __gt__(self, state):
        return self.numpas> state.numpas
    
    
    

class PlayPacman(Problem):
    """O class do problem do pacman"""
    def __init__(self, initial):
        self.initial = initial
        super().__init__(initial) 
        
        
    def actions(self, estado):
        """volta as accoes possivies dado um estado. Verifica que uma accao
        nao coloca o pacman fora do jogo, ou num espaco com obstaculo"""
        accoes = []
        if ((estado.pacmanpos[0] +1, estado.pacmanpos[1]) not in estado.obstaculos 
        and estado.pacmanpos[0] +1 > 0 
        and estado.pacmanpos[0] +1  < estado.lenx ):
            accoes.append("E")
            
        if ((estado.pacmanpos[0] -1, estado.pacmanpos[1]) not in estado.obstaculos
        and estado.pacmanpos[0] -1 > 0 
        and estado.pacmanpos[0] -1 < estado.lenx ):
            accoes.append("W")
            
        if ((estado.pacmanpos[0], estado.pacmanpos[1]+1) not in estado.obstaculos
        and estado.pacmanpos[1] +1 > 0 
        and estado.pacmanpos[1]  +1 < estado.leny ):
            accoes.append("S")
            
        if ((estado.pacmanpos[0], estado.pacmanpos[1]-1) not in estado.obstaculos
        and estado.pacmanpos[1] -1 > 0 
        and estado.pacmanpos[1]  +1 < estado.leny ):
            accoes.append("N")
            
        if ((estado.pacmanpos[0] +1, estado.pacmanpos[1]+1) not in estado.obstaculos
        and estado.pacmanpos[0] +1 > 0 
        and estado.pacmanpos[0] +1 < estado.lenx 
        and estado.pacmanpos[1] +1 > 0 
        and estado.pacmanpos[1] +1 < estado.leny ):
            accoes.append("SE")
            
        if ((estado.pacmanpos[0] -1, estado.pacmanpos[1]+1) not in estado.obstaculos
        and estado.pacmanpos[0] -1 > 0 
        and estado.pacmanpos[0] -1 < estado.lenx 
        and estado.pacmanpos[1] +1 > 0 
        and estado.pacmanpos[1] +1 < estado.leny ):
            accoes.append("SW")
            
        if ((estado.pacmanpos[0] +1, estado.pacmanpos[1]-1) not in estado.obstaculos
        and estado.pacmanpos[0] +1 > 0 
        and estado.pacmanpos[0] +1 < estado.lenx 
        and estado.pacmanpos[1] -1 > 0 
        and estado.pacmanpos[1] -1 < estado.leny ):
            accoes.append("NE")
            
        if ((estado.pacmanpos[0] -1, estado.pacmanpos[1]-1) not in estado.obstaculos
        and estado.pacmanpos[0] -1 > 0 
        and estado.pacmanpos[0] -1 < estado.lenx +1
         and estado.pacmanpos[1] -1 > 0 and estado.pacmanpos[1] -1 < estado.leny):
            accoes.append("NW")
        
        check_list = []
        for l in estado.L:
            check_list.append(l[1])
            
        if check_list.count(True) > (len(estado.pastilhas) *2):
            accoes = []
        
        
        
            
        return accoes
    
    def result(self,state, action):
        """volta o estado resultante dado um estado e uma accao. Talvez o codigo nao e 
        eficiente, o podemos melhorar."""
        x = state.lenx
        y = state.leny
        numobs = state.numobs
        numpas = state.numpas
        pacmanpos = deepcopy(state.pacmanpos)
        
        
        obstaculos = deepcopy(state.obstaculos)
        
        pastilhas = deepcopy(state.pastilhas)
        
        tf0 = state.L[0][1]
        tf1 = state.L[1][1]
        tf2 = state.L[2][1]
        tf3 = state.L[3][1]
        tf4 = state.L[4][1]
        tf5 = state.L[5][1]
        tf6 = state.L[6][1]
        tf7 = state.L[7][1]
        tf8 = state.L[8][1]
        
        
        
        
        if action == "E":
            pacmanpos[0] = pacmanpos[0] +1
            
        if action == "W":
            pacmanpos[0] = pacmanpos[0] -1
            
            
        if action == "N":
            pacmanpos[1] = pacmanpos[1] -1
            
        if action == "S":
            pacmanpos[1] = pacmanpos[1] +1
            
        if action == "NW":
            pacmanpos[0] = pacmanpos[0] -1
            pacmanpos[1] = pacmanpos[1] -1
            
        if action == "SW":
            pacmanpos[0] = pacmanpos[0] -1
            pacmanpos[1] = pacmanpos[1] +1
            
        if action == "NE":
            pacmanpos[0] = pacmanpos[0] +1
            pacmanpos[1] = pacmanpos[1] -1
            
        if action == "SE":
            pacmanpos[0] = pacmanpos[0] +1
            pacmanpos[1] = pacmanpos[1] +1
            
            
        N = []
        
       


        i0 = [[pacmanpos[0] - 1, pacmanpos[1] - 1],tf0]
        N.append(i0)

        i1 = [[pacmanpos[0], pacmanpos[1] - 1],tf1]
        N.append(i1)
        
        i2 = [[pacmanpos[0] + 1, pacmanpos[1] - 1],tf2]
        N.append(i2)
        
        i3 = [[pacmanpos[0] - 1, pacmanpos[1]],tf3]
        N.append(i3)
        
        i4 = [[pacmanpos[0] + 1, pacmanpos[1]],tf4]
        N.append(i4)
        
        i5 = [[pacmanpos[0] - 1, pacmanpos[1] + 1],tf5]
        N.append(i5)
        
        i6 = [[pacmanpos[0], pacmanpos[1] + 1],tf6]
        N.append(i6)
        
        i7 = [[pacmanpos[0] + 1, pacmanpos[1] + 1],tf7]
        N.append(i7)
        
        i8 = [[pacmanpos[0], pacmanpos[1]],tf8]
        N.append(i8)    
        
                
        for n in N:
            for p in pastilhas:
                if (n[0][0],n[0][1]) == (p.x,p.y):
                    if p.health > 0:
                        n[1] = False
                    if p.health > -1:
                        p.health = p.health - 1
                    
        newL = N
                
        res = EstadoPacman(x,y,numobs,numpas,pacmanpos, newL)
        
        res.obstaculos = obstaculos
        
        res.pastilhas = pastilhas
        
        return res
        
    def path_cost(self, c, state1, action, state2):
        r = 0
        if action in ["NW", "N", "NE"]:
            r = 3
        if action in ["W", "E"]:
            r = 1
        if action in ["SW","S","SE"]:
            r = 0
        for l in state2.L:
            for p in state2.pastilhas:
                if (l[0][0],l[0][1]) == (p.x,p.y) and p.health ==0:
                    r = r +10 
        
        return c + r
    
     
    def goal_test(self, state):
        """Um estado e um estado goal se temos anulado as posicoes relativas
        de 7 de 8 das casas vecinas de pacman. ou seja, aunque as coordenadas absolutas mudam,
        um estado e estado goal se anulamos 7 de 8 das casas relativas, que sao NW, W, ... """
        check_list = []
        for l in state.L:
            check_list.append(l[1])
            
         
        return check_list.count(True) < 2 
    
            
        
        

           




def stats_test_pac(algo, prob, heuristica,depth):
    """dado um algoritmo e problema, volta os estatisticos pedidos no enuciado
    
    OJO- pasa uma lista de algortimos com o nome verdadero, por exemplo
    se queres os estatisticos de uniform cost search, pasa como argumento
    uniform_cost_search, NAO uniform_cost_search_stats. 
    
    """
    
    if algo == uniform_cost_search:
        start = time.time()
        x = uniform_cost_search_stats(prob)
        end = time.time()
        elapsed = end - start
        resultados = []
        if type(x) == list:
            resultados.append("Solução não encontrada")
            resultados.append("Solução não encontrada")
            resultados.append(x[1])
            resultados.append(x[2])
            resultados.append(x[3])
            resultados.append(elapsed)
            
        else:
            path_actions = []
            for n in x.path():
                path_actions.append(n.action)
     
     
            resultados.append(path_actions)
     
            resultados.append(x.path_cost)
     
            resultados.append(x[1])
     
            resultados.append(x[2])
     
            resultados.append(x[3])
     
            resultados.append(elapsed)
       
    if algo == greedy_best_first_graph_search:
        start = time.time()
        x = greedy_best_first_graph_search_stats(prob, lambda n : heuristica(n))
        end = time.time()
        resultados = []
        elapsed = end - start
        if type(x) == list:
            resultados.append("Solução não encontrada")
            resultados.append("Solução não encontrada")
            resultados.append(x[1])
            resultados.append(x[2])
            resultados.append(x[3])
            resultados.append(elapsed)
        else:
            path_actions = []
            for n in x.path():
                path_actions.append(n.action)
     
     
            resultados.append(path_actions)
     
            resultados.append(x.path_cost)
     
            resultados.append(x[1])
     
            resultados.append(x[2])
     
            resultados.append(x[3])
     
            resultados.append(elapsed)
         
    if algo == astar_search:
        start = time.time()
        x = astar_search_stats(prob, lambda n: heuristica(n) + n.path_cost)
        end = time.time()
        resultados = []
        elapsed = end - start
        if type(x) == list:
            resultados.append("Solução não encontrada")
            resultados.append("Solução não encontrada")
            resultados.append(x[1])
            resultados.append(x[2])
            resultados.append(x[3])
            resultados.append(elapsed)
        else:
            path_actions = []
            for n in x.path():
                path_actions.append(n.action)
     
     
            resultados.append(path_actions)
     
            resultados.append(x.path_cost)
     
            resultados.append(x[1])
     
            resultados.append(x[2])
     
            resultados.append(x[3])
     
            resultados.append(elapsed)
    if algo == iterative_deepening_search:
        start = time.time()
        x = iterative_deepening_search_stats(prob)
        end = time.time()
        if x == None:
            return "Solução não encontrada"
    
        elapsed = end - start
        resultados = []
     
     
        path_actions = []
        for n in x.path():
            path_actions.append(n.action)
     
     
        resultados.append(path_actions)
     
        resultados.append(x.path_cost)
     
        resultados.append(x.num_explored)
     
        resultados.append(x.num_expanded)
     
        resultados.append(x.max_frontier)
     
        resultados.append(elapsed)
     
    if algo == depth_first_graph_search:
        start = time.time()
        x = depth_first_graph_search_stats(prob)
        end = time.time()
        resultados = []
        elapsed = end - start
        if type(x) == list:
            resultados.append("Solução não encontrada")
            resultados.append("Solução não encontrada")
            resultados.append(x[1])
            resultados.append(x[2])
            resultados.append(x[3])
            resultados.append(elapsed)
     
        else:
            path_actions = []
            for n in x.path():
                path_actions.append(n.action)
     
     
            resultados.append(path_actions)
     
            resultados.append(x.path_cost)
     
            resultados.append(x[1])
     
            resultados.append(x[2])
     
            resultados.append(x[3])
     
            resultados.append(elapsed)
         
    if algo == breadth_first_graph_search:
        start = time.time()
        x = breadth_first_graph_search_stats(prob)
        end = time.time()
        resultados = []
        elapsed = end - start
        print(1,x)
        if type(x) == list:
            resultados.append("Solução não encontrada")
            resultados.append("Solução não encontrada")
            resultados.append(x[1])
            resultados.append(x[2])
            resultados.append(x[3])
            resultados.append(elapsed)
     
        elif type(x) == Node:
            path_actions = []
            for n in x.path():
                path_actions.append(n.action)
     
     
            resultados.append(path_actions)
     
            resultados.append(x.path_cost)
     
            resultados.append(x[1])
     
            resultados.append(x[2])
     
            resultados.append(x[3])
     
            resultados.append(elapsed)   
    return resultados

def stats_test_grapho(algo, prob, heuristica,depth):
    """dado um algoritmo e problema, volta numa lista os estatisticos pedidos no enuciado
    
    OJO- pasa uma lista de algortimos com o nome verdadero, por exemplo
    se queres os estatisticos de uniform cost search, pasa como argumento
    uniform_cost_search, NAO uniform_cost_search_stats. 
    
    """
    
    if algo == uniform_cost_search:
        start = time.time()
        x = uniform_cost_search_stats(prob)
        end = time.time()
        resultados = []
        if x == None:
            return "Solução não encontrada"
        path_actions = []
        elapsed = end - start
        
        for n in x.path():
            path_actions.append(n.action)
     
     
        resultados.append(path_actions)
     
        resultados.append(x.path_cost)
     
        resultados.append(x.num_explored)
     
        resultados.append(x.num_expanded)
        
        resultados.append(x.max_frontier)
     
        resultados.append(elapsed)
       
    if algo == greedy_best_first_graph_search:
        start = time.time()
        x = greedy_best_first_graph_search_stats(prob, lambda n : heuristica(n))
        end = time.time()
        if x == None:
            return "Solução não encontrada"
        path_actions = []
        elapsed = end - start
        resultados = []
        for n in x.path():
            path_actions.append(n.action)
     
     
        resultados.append(path_actions)
     
        resultados.append(x.path_cost)
     
        resultados.append(x.num_explored)
     
        resultados.append(x.num_expanded)
        
        resultados.append(x.max_frontier)
     
        resultados.append(elapsed)
         
    if algo == astar_search:
        start = time.time()
        x = astar_search_stats(prob, lambda n: heuristica(n) + n.path_cost)
        end = time.time()
        if x == None:
            return "Solução não encontrada"
        path_actions = []
        elapsed = end - start
        resultados = []
        for n in x.path():
             path_actions.append(n.action)
     
     
        resultados.append(path_actions)
     
        resultados.append(x.path_cost)
     
        resultados.append(x.num_explored)
     
        resultados.append(x.num_expanded)
     
        resultados.append(x.max_frontier)
     
        resultados.append(elapsed)  
    if algo == iterative_deepening_search:
        start = time.time()
        x = iterative_deepening_search_stats(prob)
        end = time.time()
        if x == None:
            return "Solução não encontrada"
    
        elapsed = end - start
        resultados = []
     
     
        path_actions = []
        for n in x.path():
            path_actions.append(n.action)
     
     
        resultados.append(path_actions)
     
        resultados.append(x.path_cost)
     
        resultados.append(x.num_explored)
     
        resultados.append(x.num_expanded)
     
        resultados.append(x.max_frontier)
     
        resultados.append(elapsed)
     
    if algo == depth_first_graph_search:
        start = time.time()
        x = depth_first_graph_search_stats(prob)
        end = time.time()
        if x == None:
            return "Solução não encontrada"
     
        elapsed = end - start
        
        resultados = []
     
     
        path_actions = []
        for n in x.path():
            path_actions.append(n.action)
     
     
        resultados.append(path_actions)
     
        resultados.append(x.path_cost)
     
        resultados.append(x.num_explored)
     
        resultados.append(x.num_expanded)
     
        resultados.append(x.max_frontier)
     
        resultados.append(elapsed)
         
    if algo == breadth_first_graph_search:
        start = time.time()
        x = breadth_first_graph_search_stats(prob)
        end = time.time()
        if x == None:
            return "Solução não encontrada"
     
        elapsed = end - start
        resultados = []
     
     
        path_actions = []
        for n in x.path():
            path_actions.append(n.action)
     
     
        resultados.append(path_actions)
     
        resultados.append(x.path_cost)
     
        resultados.append(x.num_explored)
     
        resultados.append(x.num_expanded)
     
        resultados.append(x.max_frontier)
     
        resultados.append(elapsed)
        
    return resultados

algos = [depth_first_graph_search, breadth_first_graph_search, uniform_cost_search, greedy_best_first_graph_search, astar_search] 


probgrafo = ProblemaGrafoAbs()



def collect_write_stats_Pacinvisiv(algolist, prob, file, heuristicas, limit):
     """Dado uma lista de algoritmos, uma problema do Pacinvisiv, um ficheiro, uma 
    heursitica, e uma limite, crea um fichero com o nome dado e escreve
     neste ficheiro os estatisticos pedidos. E necesario incluir uma heuristica e limite
     porque a lista dos algoritmos pode incluir uma procura que usa uma heuristica ou limite
     
     Foi necesario fazer metodos diferentes para problemas Pacinisiv e problemas de grafos
     abstratos porque com Pacinvisiv queremos comparar varias heuristicas, e nao e necesario 
     com um problema de grafo abstrato."""
     
     with open(file, mode='w') as file:
         stats_writer = csv.writer(file, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
 
         stats_writer.writerow(['Algoritmo','Heurstica (se usado)', 'Solucao', 'Custo do caminho', "estados explorados", "estados expandidos", "maior tamanho da fronteira", "tempo"])
         for algo in algolist:
            print(algo)
            if algo == greedy_best_first_graph_search or algo == astar_search:
                print("greedy, astar")
                for h in heuristicas:
                    results = stats_test_pac(algo, prob, h, limit)
                    stats_writer.writerow([algo.__name__,h.__name__, str(results[0]),str(results[1]),str(results[2]),str(results[3]),str(results[4]),str(results[5])])
                    

            elif algo == depth_first_graph_search:
                print("depth first")
                results_trials = []
                for i in range(30):
                    results = stats_test_pac(algo, prob, None, 50)
                    results_trials = results[1:]
                
                    if results[1] != "Solução não encontrada":
                        results = list(average(results_trials))
                        results.insert(0,"N/A")
                    
                    else:
                        
            
                        del results_trials[0]
                        
                        results = results_trials
                        results.insert(0,"N/A")
                        results.insert(1, "Solução não encontrada")        
                stats_writer.writerow([algo.__name__,"nao usado", str(results[0]),str(results[1]),str(results[2]),str(results[3]),str(results[4]),str(results[5])])
                    

            else:
                 print("other")
                 results = stats_test_pac(algo, prob, heuristicas, limit)
                 stats_writer.writerow([algo.__name__,"nao usado", str(results[0]),str(results[1]),str(results[2]),str(results[3]),str(results[4]),str(results[5])])
     
 
def collect_write_stats_graph(algolist,prob,file, heuristica, limit):
     """Dado uma lista de algoritmos, uma problema grafo, um ficheiro, uma 
     heursitica, e uma limite, crea um fichero com o nome dado e escreve
     neste ficheiro os estatisticos pedidos. E necesario incluir uma heuristica e limite
     porque a lista dos algoritmos pode incluir uma procura que usa uma heuristica ou limite"""
     with open(file, mode='w') as stats_file:
         stats_writer = csv.writer(stats_file, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
 
         stats_writer.writerow(['Algoritmo', 'Solucao', 'Custo do caminho', "estados explorados", "estados expandidos", "maior tamanho da fronteira", "tempo"])
         for algo in algolist:
             results = stats_test_grapho(algo, prob, heuristica, limit)
             stats_writer.writerow([algo.__name__, str(results[0]),str(results[1]),str(results[2]),str(results[3]),str(results[4]),str(results[5])])
             

 
def average(l):
    """https://stackoverflow.com/a/13661814/9777925"""
    llen = len(l)
    def divide(x): return x / llen
    return map(divide, map(sum, zip(*l)))           






pos = [18, 18]
L_e1 = [[ [pos[0] - 1, pos[1] - 1], True], [ [pos[0], pos[1] - 1], True], [[pos[0] + 1, pos[1]], True], [[pos[0] - 1, pos[1]], True], [[pos[0], pos[1]], False], [[pos[0] + 1, pos[1] - 1], True], [[pos[0] - 1, pos[1] + 1], True], [[pos[0], pos[1] + 1], True], [[pos[0] + 1, pos[1] + 1], True]]

e1 = EstadoPacman( 20, 20, 0, 11,pos, L_e1)

e1.pastilhas = [pastilha(0,1), pastilha(1,4), pastilha(2,7), pastilha(3,2), pastilha(3,8), pastilha(4,9), pastilha(5,3), pastilha(6,8), pastilha(7,1), pastilha(7,7), pastilha(9,4)]

#print(e1)

prob_e1 = PlayPacman(e1)

#x = astar_search(prob_1,h1)





  





#Gerar Problemas dos estados dados no moodle, e aleatorio
#==============================================================================
e = GerarEstado(10)

prob = PlayPacman(e)




pos = [2, 2]
L_caso1 = [[ [pos[0] - 1, pos[1] - 1], True], [ [pos[0], pos[1] - 1], True], [[pos[0] + 1, pos[1]], True], [[pos[0] - 1, pos[1]], True], [[pos[0], pos[1]], False], [[pos[0] + 1, pos[1] - 1], True], [[pos[0] - 1, pos[1] + 1], True], [[pos[0], pos[1] + 1], True], [[pos[0] + 1, pos[1] + 1], True]]
caso1 = EstadoPacman( 8, 6, 0, 5,[2, 2], L_caso1)
caso1.pastilhas = [pastilha(5,0), pastilha(6,0), pastilha(7,0), pastilha(7,1), pastilha(7,2)]

prob_caso1 = PlayPacman(caso1)


pos = [0, 3]
L_caso2 = [[ [pos[0] - 1, pos[1] - 1], True], [ [pos[0], pos[1] - 1], True], [[pos[0] + 1, pos[1]], True], [[pos[0] - 1, pos[1]], True], [[pos[0], pos[1]], False], [[pos[0] + 1, pos[1] - 1], True], [[pos[0] - 1, pos[1] + 1], True], [[pos[0], pos[1] + 1], True], [[pos[0] + 1, pos[1] + 1], True]]
caso2 = EstadoPacman( 8, 9, 0, 11,[0,3], L_caso2)
caso2.pastilhas = [pastilha(0,0), pastilha(5,0), pastilha(6,0), pastilha(7,0), pastilha(0,1), pastilha(7,1), pastilha(7,2), pastilha(0,6), pastilha(0,7), pastilha(7,7), pastilha(7,8)]

prob_caso2 = PlayPacman(caso2)


pos = [0, 2]
L_caso3a = [[ [pos[0] - 1, pos[1] - 1], True], [ [pos[0], pos[1] - 1], True], [[pos[0] + 1, pos[1]], True], [[pos[0] - 1, pos[1]], True], [[pos[0], pos[1]], False], [[pos[0] + 1, pos[1] - 1], True], [[pos[0] - 1, pos[1] + 1], True], [[pos[0], pos[1] + 1], True], [[pos[0] + 1, pos[1] + 1], True]]
caso3a = EstadoPacman( 4, 5, 0, 4,[0,2], L_caso3a)
caso3a.pastilhas = [pastilha(0,0), pastilha(3,0), pastilha(0,4), pastilha(3,4)]

prob_caso3a = PlayPacman(caso3a)


pos = [0, 2]
L_caso3b =  [[ [pos[0] - 1, pos[1] - 1], True], [ [pos[0], pos[1] - 1], True], [[pos[0] + 1, pos[1]], True], [[pos[0] - 1, pos[1]], True], [[pos[0], pos[1]], False], [[pos[0] + 1, pos[1] - 1], True], [[pos[0] - 1, pos[1] + 1], True], [[pos[0], pos[1] + 1], True], [[pos[0] + 1, pos[1] + 1], True]] 
caso3b = EstadoPacman( 5, 5, 0, 4, [0,2], L_caso3b)
#e4.pastilhas = (pastilha(0,0), pastilha(4,0), pastilha(0,4), pastilha(4,4))
caso3b.pastilhas = [pastilha(0,0), pastilha(4,0), pastilha(0,4), pastilha(4,4)]

prob_caso3b = PlayPacman(caso3b)


pos = [2,1]
L_caso4a =  [[ [pos[0] - 1, pos[1] - 1], True], [ [pos[0], pos[1] - 1], True], [[pos[0] + 1, pos[1]], True], [[pos[0] - 1, pos[1]], True], [[pos[0], pos[1]], False], [[pos[0] + 1, pos[1] - 1], True], [[pos[0] - 1, pos[1] + 1], True], [[pos[0], pos[1] + 1], True], [[pos[0] + 1, pos[1] + 1], True]] 
caso4a = EstadoPacman(4,4,0,3, [2,1], L_caso4a)
caso4a.pastilhas = [pastilha(0,0), pastilha(0,2), pastilha(1,3)]

prob_caso4a = PlayPacman(caso4a)


pos = [2,3]
L_caso4b =  [[ [pos[0] - 1, pos[1] - 1], True], [ [pos[0], pos[1] - 1], True], [[pos[0] + 1, pos[1]], True], [[pos[0] - 1, pos[1]], True], [[pos[0], pos[1]], False], [[pos[0] + 1, pos[1] - 1], True], [[pos[0] - 1, pos[1] + 1], True], [[pos[0], pos[1] + 1], True], [[pos[0] + 1, pos[1] + 1], True]] 
caso4b = EstadoPacman(5,5,0,4, [2,3], L_caso4b)
caso4b.pastilhas = [pastilha(0,0), pastilha(0,2), pastilha(0,3), pastilha(0,4)]

prob_caso4b = PlayPacman(caso4b)


probromenia = grafo_romenia('Oradea')

probpistola = grafo_pistola()

probespelho = grafo_espelho()



#========================================================================================================================




        

algos = [depth_first_graph_search, breadth_first_graph_search, uniform_cost_search, greedy_best_first_graph_search, astar_search] 



def Gerar_Ficheiros():

    collect_write_stats_graph(algos, probespelho, "resultados_grapho_espelho.csv", h_espelho, 50)
    print(1)
    collect_write_stats_graph(algos, probromenia, "resultados_grapho_romenia.csv", h_romenia, 50)
    print(2)
    collect_write_stats_graph(algos, probpistola, "resultados_grapho_pistola.csv", h_pistola, 50)
    print(3)
    collect_write_stats_Pacinvisiv(algos, prob_caso1, "resultados_pac_caso1_h1.csv", [h1], 50)
    
    print(4)
    collect_write_stats_Pacinvisiv(algos, prob_caso2, "resultados_pac_caso2_h1.csv", [h1], 50)
    print(5)
    collect_write_stats_Pacinvisiv(algos, prob_caso3a, "resultados_pac_caso3a_h1.csv", [h1], 50)
    print(6)
    collect_write_stats_Pacinvisiv(algos, prob_caso3b, "resultados_pac_caso3b_h1.csv", [h1], 50)
    print(7)
    collect_write_stats_Pacinvisiv(algos, prob_caso4a, "resultados_pac_caso4a_h1.csv", [h1], 50)
    print(8)
    collect_write_stats_Pacinvisiv(algos, prob_caso4b, "resultados_pac_caso4b_h1.csv", [h1], 50)
    print(9)
    collect_write_stats_Pacinvisiv(algos, prob, "resultados_pac_caso_aleatorio_h1.csv", [h1], 50)
    print(10)


    collect_write_stats_Pacinvisiv(algos, prob_caso1, "resultados_pac_caso1_h2.csv", [h2], 50)
    print(11)
    collect_write_stats_Pacinvisiv(algos, prob_caso2, "resultados_pac_caso2_h2.csv", [h2], 50)
    print(12)
    collect_write_stats_Pacinvisiv(algos, prob_caso3a, "resultados_pac_caso3a_h2.csv", [h2], 50)
    print(13)
    collect_write_stats_Pacinvisiv(algos, prob_caso3b, "resultados_pac_caso3b_h2.csv", [h2], 50)
    print(14)
    collect_write_stats_Pacinvisiv(algos, prob_caso4a, "resultados_pac_caso4a_h2.csv", [h2], 50)
    print(15)
    collect_write_stats_Pacinvisiv(algos, prob_caso4b, "resultados_pac_caso4b_h2.csv", [h2], 50)
    print(16)
    collect_write_stats_Pacinvisiv(algos, prob, "resultados_pac_caso_aleatorio_h2.csv", [h2], 50)
    print(17)



