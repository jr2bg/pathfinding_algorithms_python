# requerimos un grid de m x n
# m filas, n columnas
# Dijkstra usa min heaps

import heapq 
import csv
import matplotlib.pyplot as plt
import matplotlib.animation as animation


# clase del nodito
# a partir de la información de la fila + columna 
# podemos determinar si hay otros nodos
class Node():
    def __init__(self, row, col):
        #self.up = None
        #self.down = None
        #self.left = None
        #self.right = None
        #self.UR = None
        #self.UL = None
        #self.DR = None
        #self.DL = None
        
        self.nodes_dist_dic = {}
        self.row = row
        self.col = col
    
    def coords(self):
        # función que da las coordenadas del nodito
        return (self.row, self.col)
        
    def isObstacle(self, isO):
        # determina si el nodo es libre o es un obstáculo
        if isO:
            self.iso = True
        else:
            self.iso = False
    
    
    # distancia
    def add_edge(self, nodo):
        if (abs(nodo.row - self.row) + abs(nodo.col - self.col) ) % 2 == 0:
            self.nodes_dist_dic[nodo] = 2 ** (1/2)
        else:
            self.nodes_dist_dic[nodo] = 1
    
    
    def init_single_source_node(self, row, col):
        # si el nodo corresponde al origen, la distancia es cero
        if row == self.row and col == self.col:
            self.d = 0
        # si no, entonces la distancia es cero
        else:
            self.d = float("inf")
        # nodo predecesor
        self.pre = None
        
    
    # add the relaxed nodes to Q
    def relax(self, adj_node, weight_s2ad, Q):
        # adj_node is a node in nodes_dist_dic
        #  distancia nodo adj >   distancia hasta ese nodo + peso
        if adj_node.d  > self.d + weight_s2ad:
            adj_node.d = self.d + weight_s2ad
            t = adj_node.d 
            #print(adj_node.coords())
            #print(Q)
            #print(Q)
            heapq.heappush(Q, (t, adj_node.coords()))
            adj_node.pre = self
        return Q
    
    def relaxing_adjNodes(self, Q):
        #print(self.nodes_dist_dic)
        for key, val in self.nodes_dist_dic.items():
            Q = self.relax(key, val, Q)
            #print(val)
            #print(key.pre.d)
        return Q
        

class Grid:
    def __init__(self, n_rows, n_cols):
        self.n_rows = n_rows
        self.n_cols = n_cols
        
    def generate_space(self):
        # generación del espacio a considerar
        self.space = [[Node(r,c) for c in range(self.n_cols)] for r in range(self.n_rows)]
        
    def add_obst_csv(self, csv_path, obst = "#"):
        # función para generar un array dependiendo de si hay un obstáculo o no
        
        #en los archivos, "1" es libre y "0" es ostáculo
        with open(path_csv, newline="") as maze:
            reader = csv.reader(maze)
            data = list(reader)
            
        self.n_rows = len(data)
        self.n_cols = len(data[0])


        #  obstáculos fijos -> 0; libres -> 1; target -> 2 ; obstáculos móvles -> 3
        
        self.display =[[1 for j in range(self.n_cols)] for i in range(self.n_rows)]
        
        # generación del espacio
        self.generate_space()
        
        # lista de obstáculos
        #l_obst = []
        # para cada fila
        for i in range(self.n_rows):
            #para cada columna
            for j in range(self.n_cols):
                # variable temporal
                tmp = self.space[i][j]
            
                # funciones para la creación de las edges
                if i == 0 and j == 0:
                    
                    tmp.nodes_dist_dic = {}
                
                if data[i][j] == obst:
                    #l_obst.append((i,j))
                    # el array a imprimir
                    self.display[i][j] = 0
                    tmp.isObstacle(True)
                else:
                    tmp.isObstacle(False)

        #return l_obst
    
    def generate_edges(self):
        # función para generar las edges que deja cada nodo
        # requiere haber generado previamente el espacio
        for i in range(self.n_rows):
            for j in range(self.n_cols):
                # variable temporal
                tmp = self.space[i][j]
                
                # si es obstáculo, ni lo tomamos en cuenta
                if tmp.iso:
                    continue
                
                # determinaremos si existen en la cuadrícula y se conectarán
                # solo con casillas libres
                
                # diagonal sup izquierda
                if i > 0 and j > 0 and not self.space[i-1][j-1].iso:
                    tmp.add_edge(self.space[i-1][j-1])
                
                # vertical arriba
                if i > 0 and not self.space[i-1][j].iso:
                    tmp.add_edge(self.space[i-1][j])
                
                # diagonal sup derecha
                if i > 0 and j < self.n_cols -1 and not self.space[i-1][j+1].iso:
                    tmp.add_edge(self.space[i-1][j+1])
                
                # horizontal derecha
                if j < self.n_cols - 1 and not self.space[i][j+1].iso:
                    tmp.add_edge(self.space[i][j+1])
                
                # diagonal inf derecha
                if i < self.n_rows - 1 and j < self.n_cols - 1 and not self.space[i+1][j+1].iso:
                    tmp.add_edge(self.space[i+1][j+1])
                
                # vertical abajo
                if i < self.n_rows - 1 and not self.space[i+1][j].iso:
                    tmp.add_edge(self.space[i+1][j])
                
                # diagonal inf izquierda
                if i < self.n_rows - 1 and j > 0 and not self.space[i+1][j-1].iso:
                    tmp.add_edge(self.space[i+1][j-1])
                
                #horizontal izquierda
                if j > 0 and not self.space[i][j-1].iso:
                    tmp.add_edge(self.space[i][j-1])
                
                ##### A BORRAR
                #print(len(tmp.nodes_dist_dic), end = " ")
            #print()
        
    def init_single_source_grid(self, row_p, col_p):
        # iteración sobre todos los nodos para inicializar Dijkstra
        for r in range(self.n_rows):
            for c in range(self.n_cols):
                # se inicializa el espacio para cada nodo
                self.space[r][c].init_single_source_node(row_p, col_p)

# función para remover los nodos nodo de un heap
def remove_nodes(Q, nodo):
    l = []
    # identifica todos los nodos correspondientes
    for i in range(len(Q)):
        if Q[i][1] == nodo:
            l.append(i)
    #print(l)
    #los elimina del heap
    for i in reversed(l):
        Q.pop(i)
    
    # crea el heap
    heapq.heapify(Q)
    return Q

def get_path(finishedDijkstra, r_target, c_target):
    # función para tener el path desde el origen hasta el destino
    pt = [(r_target,c_target)]
    nodito = finishedDijkstra.space[r_target][c_target]
    while nodito.pre:
        # en posición 0 insertamos el nodo previo
        nodito = nodito.pre
        pt.insert(0, nodito.coords())
    return pt



def dijkstra(generatedGrid, r_target, c_target, r_origin, c_origin):
    # empty heap
    # nodo de origen
    s_node = generatedGrid.space[r_origin][c_origin]
    Q = [(s_node.d, s_node.coords())]
    heapq.heapify(Q)
    S = []
    
    list_anims = []
    fig = plt.figure()
    while len(Q) > 0:
        dist, cds = heapq.heappop(Q)
        #print(cds)
        nodito = generatedGrid.space[cds[0]][cds[1]]
        
        
        generatedGrid.display[cds[0]][cds[1]] = 2
        img = plt.imshow(generatedGrid.display)
        list_anims.append([img])
        #plt.pause(0.01)
        #plt.show()
        
        S.append(cds)
        if cds == (r_target, c_target):
            break
        #print(S)
        Q = nodito.relaxing_adjNodes(Q)
        #print(Q)
        # removemos nodos repetidos, esto es, nodos en S
        for cons_node in S:
            print(cons_node)
            Q = remove_nodes(Q, cons_node)
    ani = animation.ArtistAnimation(fig, list_anims, interval=100, blit=True, repeat_delay=1000)
    writergif = animation.PillowWriter(fps=5)
    ani.save('filename.gif',writer=writergif)
    return S, list_anims

#nodo1 = Node(0,0)
#nodo2 = Node(1,1)

#nodo1.add_edge(nodo2)

#nodo1.init_single_source_node(0,0)
#nodo2.init_single_source_node(0,0)

#nodo1.relaxing_adjNodes()

#print(nodo2.pre.d)

#espacio = Grid(2,2)

#espacio.generate_space()

#espacio.init_single_source_grid(1,1)

####################################################
####
#### TEST 3
####
#### Obj: probar que se genera el laberinto a partir de un
#### csv y que lo imprime
####################################################
#path_csv = "maze_tst.csv"
#espacio = Grid(0,0)
#
## los obstáculos tienen caracter "0"
#espacio.add_obst_csv(path_csv, obst = "0")
#plt.imshow(espacio.display)
#plt.show()
## RESULTADO: COMPLETADO
## Cambios: se invirtió el caracter entre libres y obst fijos


####################################################
####
#### TEST 4
####
#### Obj: probar que se generan las aristas de forma adecuada
#### procedimiento: con la longitud de la lista de adyacencia
####     se obtendrán todos los vecinos
####################################################
#path_csv = "maze_tst.csv"
#espacio = Grid(0,0)
#espacio.add_obst_csv(path_csv, obst = "0")
## generación de las edges
#espacio.generate_edges()
## RESULTADO: COMPLETADO
## Cambios: se imprimió la longitud de la lista de adyacencia, pero se borrará

####################################################
####
#### TEST 5
####
#### Obj: probar que el algos de Dijkstra funciona
#### procedimiento: probar dijkstra
####################################################
#print(type(1.5))
path_csv = "maze_tst.csv"
espacio = Grid(0,0)
espacio.add_obst_csv(path_csv, obst = "0")
espacio.generate_edges()
# inicializamos los nodos
r_target= 31 
c_target=  1
r_origin= 41
c_origin=  1

espacio.init_single_source_grid(r_origin, c_origin)
S, list_anims = dijkstra(espacio, r_target, c_target, r_origin, c_origin)

print(get_path(espacio,r_target, c_target))
#fig = plt.figure()
#ani = animation.ArtistAnimation(fig, list_anims, interval=100, blit=True,
                                #repeat_delay=1000)
#Writer = animation.writers['ffmpeg']
#writer = Writer(fps=15, metadata=dict(artist='Me'), bitrate=1800)
#ani.save("test_dijkstra.mp4", writer = writer)
plt.show()
#writergif = animation.PillowWriter(fps=30)
#ani.save('filename.gif',writer=writergif)
#print(S)