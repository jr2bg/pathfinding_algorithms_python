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
        self.nodes_dist_dic = {}
        self.row = row
        self.col = col
        # heuristic distance
        self.hd = None

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
            self.g = 0
            # self.hd = None  # <- puede que no se requiera
        # si no, entonces la distancia es cero
        else:
            self.d = float("inf")
            self.g = float("inf")
            # heuristic distance
            #self.hd = None
        # nodo predecesor
        self.pre = None


    # add the relaxed nodes to Q
    def relax(self, r_target, c_target, adj_node, weight_s2ad, opn, h):
        '''
        método para encontrar la relajación de los nodos adjacentes a self que
        considera ya la heurística a emplear

        r_target -> fila del target (0-based)
        c_target -> columna del target (0-based)
        adj_node -> un nodo adyacente a self
        weight_s2ad -> peso de la arista desde self a adj_node
        opn -> heap correspondiente a los pesos acumulados para las aristas
        h -> heurística, en este caso emplearemos la distancia de Manhattan
        '''



        # si no se había calculado previamente la distancia heurística
        if not adj_node.hd:
            # fila y columna asociados a adj_node
            r_nodo = adj_node.row
            c_nodo = adj_node.col

            # distancia estimada a partir de la heurística
            adj_node.hd = h(r_target, c_target, r_nodo, c_nodo)

        # valor actual del g
        curr_g = self.g + weight_s2ad

        # adj_node is a node in nodes_dist_dic
        #  distancia previa desde el origen hasta ese nodo es mayor que la nueva
        # distancia calculada
        if adj_node.g  > curr_g:

            # si ya está en open, hay que reemplazarlo
            tup2check = (adj_node.d, adj_node.coords())
            if tup2check in opn:
                opn.remove(tup2check)

            adj_node.g = curr_g

            #  f(x)    =     g(x)   +     h(x)
            adj_node.d = adj_node.g + adj_node.hd
            t = adj_node.d
            #print(adj_node.coords())
            #print(Q)
            #print(Q)
            heapq.heappush(opn, (t, adj_node.coords()))
            adj_node.pre = self
        return opn

    def relaxing_adjNodes_aS(self, r_target, c_target, opn , cls, h):
        '''
        método para "relajar" los nodos adjacentes a self

        r_target -> fila del target (0-based)
        c_target -> columna del target (0-based)
        opn -> nodos aun no evaluados
        cls -> nodos ya evaluados
        h -> heurística, en este caso emplearemos la distancia de Manhattan
        '''
        #
        for nodo, dist in self.nodes_dist_dic.items():

            # si el nodo encontrado no se encuentra en los nodos ya explorados
            # entonces entra a la relajación uwu
            if nodo not in cls:
                opn = self.relax(r_target, c_target, nodo, dist, opn, h)
            #print(val)
            #print(key.pre.d)
        return opn


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


        #  obstáculos fijos -> 0; libres -> 30; target -> 2 ; obstáculos móvles -> 3

        self.display =[[30 for j in range(self.n_cols)] for i in range(self.n_rows)]

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

                ##### A BORRAR, SOLO TEST DE GENERACIÓN ADECUADA DE EDGES
                #print(len(tmp.nodes_dist_dic), end = " ")
            #print()

    def init_single_source_grid(self, row_p, col_p):
        '''
        iteración sobre todos los nodos para inicializar Dijkstra / A*

        row_p -> fila de origen
        col_p -> columna de origen
        '''
        for r in range(self.n_rows):
            for c in range(self.n_cols):
                # se inicializa el espacio para cada nodo
                self.space[r][c].init_single_source_node(row_p, col_p)


def manhattan(r_target, c_target, r_nodo, c_nodo):
    '''
    distancia de Manhattan
    empleada como heurística para el algoritmo A*
    '''
    return abs(r_target - r_nodo) + abs(c_target - c_nodo)

# función para remover los nodos nodo de un heap
def remove_nodes(Q, nodo):
    '''
    función para remover los nodos nodo de un heap

    Q -> heap
    nodo -> nodo a remover
    '''
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

def get_path(finishedAS, r_target, c_target):
    '''
    función para tener el path desde el origen hasta el destino

    finishedAS -> instancia de Grid que ya ha pasado por A* y terminado
    r_target -> fila de la ubicación del destino
    c_target -> columna de la ubicación del destino
    '''
    pt = [(r_target,c_target)]
    nodito = finishedAS.space[r_target][c_target]
    while nodito.pre:
        # en posición 0 insertamos el nodo previo
        nodito = nodito.pre
        pt.insert(0, nodito.coords())
    return pt

def show_path(pt, finishedAS):
    for par in pt:
        # path color : 20
        finishedAS.display[par[0]][par[1]] = 20
    plt.imshow(finishedAS.display)
    plt.show()


def a_star(generatedGrid, r_target, c_target, r_origin, c_origin, h):
    '''
    Implementación del algoritmo A*

    generatedGrid -> instancia de Grid que tiene ya tanto el mapa como el origen
    r_target -> fila del target (0-based)
    c_target -> columna del target (0-based)
    r_origin -> fila del origen (0-based)
    c_origin -> columna del origen (0-based)
    h -> heurística, en este caso emplearemos la distancia de Manhattan
    '''
    # row number and column number of the grid
    t_rows = len(generatedGrid.space)
    t_cols = len(generatedGrid.space[0])

    # nodo de origen
    s_node = generatedGrid.space[r_origin][c_origin]

    #generatedGrid.display[r_origin][c_origin] = 3
    #generatedGrid.display[r_target][c_target] = 4
    #Q = [(s_node.d, s_node.coords())]
    #heapq.heapify(Q)
    #S = []

    # initialize evaluated nodes (cls) and not yet evaluated nodes (opn)
    opn = [(s_node.d, s_node.coords())]
    cls = []

    # heapify the opn list to get the smallest element
    heapq.heapify(opn)

    # list to get the animations
    list_anims = []

    # figure initialization
    fig = plt.figure()

    while len(opn) > 0:

        # get the minimum distance of the heap
        dist, cds = heapq.heappop(opn)

        #testeo
        print(cds)

        # add "node" to cls
        cls.append(cds)

        # last extracted node's information
        nodito = generatedGrid.space[cds[0]][cds[1]]

        # explored color : 10
        generatedGrid.display[cds[0]][cds[1]] = 10
        img = plt.imshow(generatedGrid.display)
        list_anims.append([img])
        #plt.pause(0.01)
        #plt.show()

        #S.append(cds)
        if cds == (r_target, c_target):
            break

        # considering only those nodes NOT YET EVALUATED
        opn = nodito.relaxing_adjNodes_aS(r_target, c_target, opn , cls, h)
        #print(Q)
        # removemos nodos repetidos, esto es, nodos en S
        #for cons_node in S:
            #print(cons_node)
            #opn = remove_nodes(opn, cons_node)

    print( "#####       FINISHED A*      ######")

    #usando ffmpeg
    ani = animation.ArtistAnimation(fig, list_anims, interval=100, blit=True,
                                    repeat_delay=1000)
    Writer = animation.writers['ffmpeg']
    writer = Writer(fps=15, metadata=dict(artist='Me'), bitrate=1800)
    ani.save("test_dijkstra.mp4", writer = writer)

    # create the animation from the list of figures
    #ani = animation.ArtistAnimation(fig, list_anims, interval=100, blit=True, repeat_delay=500)
    #writergif = animation.PillowWriter(fps=5)
    #ani.save('filename.gif',writer=writergif)

    # get the list of the previous nodes
    return cls, list_anims

# ------------------------------------------------------------------------------
####################################################
####
#### TEST 1
####
#### Obj: probar que se genera el mapa de forma adecuada
#### procedimiento: usar solo la clase GRID y generar el espacio a partir de un
####                  csv
####################################################
#path_csv = "maze_tst.csv"
#espacio = Grid(0,0)
#espacio.add_obst_csv(path_csv, obst = "0")
#plt.imshow(espacio.display)
#plt.show()
####################################################
####
#### estatus: aprobado. Genera el grid sin inconvenientes
####
####################################################
# ------------------------------------------------------------------------------
####################################################
####
#### TEST 2
####
#### Obj: probar que se generan las aristas de forma adecuada
#### procedimiento: imprimir el número de edges para cada casilla abierta
####################################################
#path_csv = "maze_tst.csv"
#espacio = Grid(0,0)
#espacio.add_obst_csv(path_csv, obst = "0")
#espacio.generate_edges()    ##
#plt.imshow(espacio.display)
#plt.show()
####################################################
####
#### estatus: aprobado. imprime el número de aristas
####
####################################################
# ------------------------------------------------------------------------------
####################################################
####
#### TEST 3
####
#### Obj: probar que el algos A* funciona
#### procedimiento: probar a_star
####################################################
path_csv = "maze_tst.csv"
espacio = Grid(0,0)
espacio.add_obst_csv(path_csv, obst = "0")
espacio.generate_edges()
# inicializamos los nodos
r_target=  1
c_target=  1
r_origin= 41
c_origin= 41

espacio.init_single_source_grid(r_origin, c_origin)
S, list_anims = a_star(espacio, r_target, c_target, r_origin, c_origin, manhattan)

pt = get_path(espacio,r_target, c_target)
#show_path(pt, espacio)
####################################################
####
#### estatus: aprobado - obtiene una animación + la solución
####
####################################################
# ------------------------------------------------------------------------------
