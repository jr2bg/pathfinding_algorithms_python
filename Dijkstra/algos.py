# requerimos un grid de m x n
# m filas, n columnas
# Dijkstra usa min heaps
# 
import heapq 

# clase del nodito
# a partir de la información de la fila + columna 
# podemos determinar si hay otros nodos
class Node(row, col, n_rows, n_cols):
    def __init__(self):
        self.up = None
        self.down = None
        self.left = None
        self.right = None
        self.UR = None
        self.UL = None
        self.DR = None
        self.DL = None
        
        self.nodes_dist_dic = {}
        self.row = row
        self.col = col
        
    
    # distancia
    def add_node(self, nodo , r_nodo, c_nodo):
        if (abs(r_nodo - self.row) + abs(c_nodo - self.col) ) % 2 == 0:
            self.nodes_dist_dic[nodo] = 2 ** (1/2)
        else:
            self.nodes_dist_dic[nodo] = 1
    
    
    def init_single_source(self, row, col):
        # si el nodo corresponde al origen, la distancia es cero
        if row == self.row and col == self.col:
            self.d = 0
        # si no, entonces la distancia es cero
        else:
            self.d = float("inf")
        # nodo predecesor
        self.pre = None
        
        
    def relax(self, adj_node, weight_s2ad):
        # adj_node is a node in nodes_dist_dic
        #  distancia nodo adj >   distancia hasta ese nodo + peso
        if adj_node.d >  self.d + weight_s2ad:
            adj_node.d = self.d + weight_s2ad
            adj_node.pre = self
    
    def relaxing_adjNodes(self):
        for key, val in self.nodes_dist_dic.items():
            relax(self, key, val)
        



