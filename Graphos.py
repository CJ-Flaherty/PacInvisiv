#####   Grafo Abstracto 

from search_pacman import *

class ProblemaGrafoAbs(Problem) :
    grafoD = {'I':{'A':2,'B':5},
              'A':{'C':2,'D':4,'I':2},
              'B':{'D':1,'F':5,'I':5},
              'C':{},
              'D':{'C':3,'F':2},
              'F':{}}

    hD = {'I':7, 'A':2,'B':3,'C':1,'D':5,'F':0}      

    def __init__(self,initial = 'I', goal = 'F',grafo=grafoD,h=hD) : 
        super().__init__(initial,goal)
        self.grafo = grafo
        self.h = h

    def actions(self,estado) :
        sucessores = self.grafo[estado].keys()  # métodos keys() devolve a lista das chaves do dicionário         accoes = list(map(lambda x : "ir de {} para {}".format(estado,x),sucessores))         return accoes 
        accoes = list(map(lambda x : "ir de {} para {}".format(estado,x),sucessores))
        return accoes
 
    def result(self, estado, accao) :
        """Assume-se que uma acção é da forma 'ir de X para Y'         """
        return accao.split()[-1] 
 
    def path_cost(self, c, state1, action, state2):
        return c + self.grafo[state1][state2] 
 
    def h(self,no) :
        """Uma heurística é uma função de um estado.
        Nesta implementação, é uma função do estado associado ao nó 
        (objecto da classe Node) fornecido como argumento.
        """
        return self.h[no.state] 
 
# ESPELHO
class grafo_espelho(Problem):
    espelho = {'0'  :{ '1':4,  '2':4},
               '1'  :{ '3':3,  '4':3},
               '2'  :{ '5':3,  '6':3},
               '3'  :{ '7':2,  '8':2},
               '4'  :{ '9':2,  '10':2},
               '5'  :{ '11':2, '12':2},
               '6'  :{ '13':2, '14':2},
               '7'  :{ '15':1, '16':1},
               '8'  :{ '17':1, '18':1},
               '9'  :{ '19':1, '20':1},
               '10' :{ '17':1, '22':1},
               '11' :{ '19':1, '24':1},
               '12' :{ '25':1, '26':1},
               '13' :{ '27':1, '28':1},
               '14' :{ '29':1, '30':1},
               '15' :{ '31':4.5},
               '16' :{ '31':3.7},
               '17' :{ '31':3.5},
               '18' :{ '31':3.5},
               '19' :{ '31':3.7},
               '20' :{ '31':4},
               '21' :{ '31':4.2},
               '22' :{ '31':5.5},
               '23' :{ '32':4.5},
               '24' :{ '32':3.7},
               '25' :{ '32':3.5},
               '26' :{ '32':3.5},
               '27' :{ '32':3.7},
               '28' :{ '32':4},
               '29' :{ '32':4.2},
               '30' :{ '32':5.5},
               '31' :{ '33':5, '34':4.3, '35':4.2, '36':4.2, '37':4.3, '38':4.8, '39':5, '40':6, '41':6.2, '42':7.5, '43':7.7, '44':9, '45':9.2, '46':10.5, '47':10.7, '48':12},
               '32' :{ '33':12, '34':10.7, '35':10.5, '36':9.2, '37':9, '38':7.7, '39':7.5, '40':6.2, '41':6, '42':5, '43':4.8, '44':4.3, '45':4.2, '46':4.2, '47':4.3, '48':5},
               '33' :{ '49':1},
               '34' :{ '49':1},
               '35' :{ '50':1},
               '36' :{ '50':1},
               '37' :{ '51':1},
               '38' :{ '51':1},
               '39' :{ '52':1},
               '40' :{ '52':1},
               '41' :{ '53':1},
               '42' :{ '53':1},
               '43' :{ '54':1},
               '44' :{ '54':1},
               '45' :{ '55':1},
               '46' :{ '55':1},
               '47' :{ '56':1},
               '48' :{ '56':1},
               '49' :{ '57':2},
               '50' :{ '57':2},
               '51' :{ '58':2},
               '52' :{ '58':2},
               '53' :{ '59':2},
               '54' :{ '59':2},
               '55' :{ '60':2},
               '56' :{ '60':2},
               '57' :{ '61':3},
               '58' :{ '61':3},
               '59' :{ '62':3},
               '60' :{ '62':3},
               '61' :{ '63':100},
               '62' :{ '63':100},
               '63' :{}}

    hespelho= {0:19.5, 1:18, 2:18, 3:17, 4:16, 5:16, 6:17, 7:15.7,8:15,9:14.8,10:14.2,11:14.2,12:14.7,13:15,14:15.7, 15:15.2, 16:14.8, 17:14.6, 18:14.2, 19:14, 20:13.6, 21:13.4, 22:13, 23:13, 24:13.4, 25:13.6, 26:14, 27:14.2, 28:14.6, 29:14.8, 30:15.2, 31:11.2, 32:11.2, 33:9.4, 34:8.4, 35:8.2, 36:7.2, 37:7,38:6.4,39:6.2,40:6, 41:6, 42:6.2, 43:6.4, 44:7, 45:7.2, 46:8.2, 47:8.4, 48:9.4, 49:8.5, 50:7, 51:6, 52:5.5 , 53:5.5, 54:6, 55:7, 56:8.5, 57:6.8,58:4.5,59:4.5, 60: 6.8, 61:120, 62:120, 63:0}

    def __init__(self,initial = '0', goal = '63',grafo=espelho,h=hespelho) : 
        super().__init__(initial,goal)
        self.grafo = grafo
        self.h = h

    def actions(self,estado) :
        sucessores = self.grafo[estado].keys()  # métodos keys() devolve a lista das chaves do dicionário         accoes = list(map(lambda x : "ir de {} para {}".format(estado,x),sucessores))         return accoes 
        accoes = list(map(lambda x : "ir de {} para {}".format(estado,x),sucessores))
        return accoes
 
    def result(self, estado, accao) :
        """Assume-se que uma acção é da forma 'ir de X para Y'         """
        return accao.split()[-1] 
 
    def path_cost(self, c, state1, action, state2):
        return c + self.grafo[state1][state2] 
 
    def h(self,no) :
        """Uma heurística é uma função de um estado.
        Nesta implementação, é uma função do estado associado ao nó 
        (objecto da classe Node) fornecido como argumento.
        """
        return self.h[no.state] 
# PISTOLA
class grafo_pistola(Problem):
    
    pistola = {'A':{'D':1,'B':4},
               'B':{'A':3,'C':8,'E':2},
               'C':{'B':4,'F':3},
               'D':{'A':3,'E':2,'H':4},
               'E':{'B':4,'D':1,'F':3},
               'F':{'C':8,'E':2,'G':5},
               'G':{'F':3,'I':1},
               'H':{'D':1,'J':5},
               'I':{'G':5,'M':3},
               'J':{'H':4,'K':0},
               'K':{'J':5,'L':1},
               'L':{'K':0,'M':3},
               'M':{'1':4,'L':1}}

    hpistola = {'A':8, 'B':9, 'C':10, 'D':6, 'E':9, 'F':8, 'G':5, 'H':5, 'I':3, 'J':0, 'K':0, 'L':1, 'M':1}

    def __init__(self,initial = 'A', goal = 'M',grafo=pistola,h=hpistola) : 
        super().__init__(initial,goal)
        self.grafo = grafo
        self.h = h

    def actions(self,estado) :
        sucessores = self.grafo[estado].keys()  # métodos keys() devolve a lista das chaves do dicionário         accoes = list(map(lambda x : "ir de {} para {}".format(estado,x),sucessores))         return accoes 
        accoes = list(map(lambda x : "ir de {} para {}".format(estado,x),sucessores))
        return accoes
 
    def result(self, estado, accao) :
        """Assume-se que uma acção é da forma 'ir de X para Y'         """
        return accao.split()[-1] 
 
    def path_cost(self, c, state1, action, state2):
        return c + self.grafo[state1][state2] 
 
    def h(self,no) :
        """Uma heurística é uma função de um estado.
        Nesta implementação, é uma função do estado associado ao nó 
        (objecto da classe Node) fornecido como argumento.
        """
        return self.h[no.state] 
###  ROMENIA

## Destino é sempre Bucareste, mas a cidade de partida pode variar
class grafo_romenia(Problem):
    romenia = {'Oradea':{'Sibiu':151,'Zerind':71},
               'Zerind':{'Arad':75,'Oradea':71},
               'Arad':{'Zerind':75,'Sibiu':140,'Timisoara':118},
               'Timisoara':{'Arad':118,'Lugoj':111},
               'Lugoj':{'Timisoara':111,'Mehadia':70},
               'Mehadia':{'Lugoj':70,'Dobreta':75},
               'Dobreta':{'Mehadia':75,'Craiova':120},
               'Craiova':{'Dobreta':120,'Pitesti':138, 'RimnicuVilcea':146},
               'Sibiu':{'Oradea':151,'Arad':140,'Fagaras':99, 'RimnicuVilcea':80},
               'Fagaras':{'Sibiu':99,'Bucharest':211},
               'RimnicuVilcea':{'Craiova':146,'Pitesti':97, 'Sibiu':80},
               'Pitesti':{'Craiova':138,'RimnicuVilcea':97,'Bucharest':101},
               'Bucharest':{'Fagaras':211,'Giurgiu':90, 'Pitesti':101,'Urziceni':85},
               'Giurgiu':{'Bucharest':90},
               'Urziceni':{'Bucharest':85,'Hirsova':98,'Vaslui':142},
               'Hirsova':{'Urziceni':98,'Eforie':86},
               'Eforie':{'Hirsova':86},
               'Vaslui':{'Urziceni':142,'Iasi':92},
               'Iasi':{'Neamt':87,'Vaslui':92},
               'Neamt': {'Iasi':87}}

    hromenia = {'Arad':366, 'Bucharest':0,'Craiova':160,'Dobreta':242,'Eforie':161,'Fagaras':178,'Giurgiu':77,'Hirsova':151,'Iasi':226,'Lugoj':244,'Mehadia':241, 'Neamt':234,'Oradea':380,'Pitesti':98,'RimnicuVilcea':193,'Sibiu':253,'Timisoara':329,'Urziceni':80,'Vaslui':199,'Zerind':374}

    def __init__(self,initial, goal = 'Bucharest',grafo=romenia,h=hromenia) : 
        super().__init__(initial,goal)
        self.grafo = grafo
        self.h = h

    def actions(self,estado) :
        sucessores = self.grafo[estado].keys()  # métodos keys() devolve a lista das chaves do dicionário         accoes = list(map(lambda x : "ir de {} para {}".format(estado,x),sucessores))         return accoes 
        accoes = list(map(lambda x : "ir de {} para {}".format(estado,x),sucessores))
        return accoes
 
    def result(self, estado, accao) :
        """Assume-se que uma acção é da forma 'ir de X para Y'         """
        return accao.split()[-1] 
 
    def path_cost(self, c, state1, action, state2):
        return c + self.grafo[state1][state2] 
 
    def h(self,no) :
        """Uma heurística é uma função de um estado.
        Nesta implementação, é uma função do estado associado ao nó 
        (objecto da classe Node) fornecido como argumento.
        """
        return self.h[no.state] 


