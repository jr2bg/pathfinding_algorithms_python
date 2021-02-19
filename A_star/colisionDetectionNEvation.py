def f_isfront2front(Ped_i, Ped_j):
    '''
    función que determina si los peatones tienen un conflicto front 2 front
    '''
    i_cur_r = Ped_i.row
    i_cur_c = Ped_i.col
    i_nxt_r = Ped_i.r_nxt_p
    i_nxt_c = Ped_i.c_nxt_p

    j_cur_r = Ped_j.row
    j_cur_c = Ped_j.col
    j_nxt_r = Ped_j.r_nxt_p
    j_nxt_c = Ped_j.c_nxt_p

    #posiciones actuales corresponden a los siguientes pasos del otro
    if i_cur_r == j_nxt_r and i_cur_c == j_nxt_c and i_nxt_r == j_cur_r and
        j_cur_c == i_nxt_c:
        return True
    return False

def f_issame_cell_arrival(Ped_i, Ped_j):
    '''
    función que determina si los peatones llevan a la misma celda al siguiente
    paso
    '''
    i_nxt_r = Ped_i.r_nxt_p
    i_nxt_c = Ped_i.c_nxt_p

    j_nxt_r = Ped_j.r_nxt_p
    j_nxt_c = Ped_j.c_nxt_p


    if i_nxt_c == j_nxt_c and i_nxt_r == j_nxt_r:
        return True
    return False


def f_islet_pass(Ped_i, Ped_j):
    '''
    función que determina si al siguiente paso el peatón llega a una celda
    actualmente ocupada por un peatón que se desplaza
    '''
    i_nxt_r = Ped_i.r_nxt_p
    i_nxt_c = Ped_i.c_nxt_p

    j_cur_r = Ped_j.row
    j_cur_c = Ped_j.col
    j_nxt_r = Ped_j.r_nxt_p
    j_nxt_c = Ped_j.c_nxt_p

    if i_nxt_r == j_cur_r and j_cur_c == i_nxt_c and j_cur_r != j_nxt_r and
    j_cur_c != j_nxt_c:
        return True
    return False


def f_isto_waiter(Ped_i, Ped_j):
    '''
    función que determina si al siguiente paso el peatón llega a una celda
    actualmente ocupada por un peatón que está estático

    TERMINAR: puede haber pobemas con el waiting
    '''
    i_nxt_r = Ped_i.r_nxt_p
    i_nxt_c = Ped_i.c_nxt_p

    j_cur_r = Ped_j.row
    j_cur_c = Ped_j.col
    j_nxt_r = Ped_j.r_nxt_p
    j_nxt_c = Ped_j.c_nxt_p

    if i_nxt_r == j_cur_r and j_cur_c == i_nxt_c and Ped_j.estado == "waiting":
        return True
    return False

def f_other_crosses(Ped_i, Ped_j):
    '''
    función que determina si Ped_j se mueve en una diagonal que está dentro de
    la vecindad de Ped_i
    '''
    i_cur_r = Ped_i.row
    i_cur_c = Ped_i.col

    j_cur_r = Ped_j.row
    j_cur_c = Ped_j.col
    j_nxt_r = Ped_j.r_nxt_p
    j_nxt_c = Ped_j.c_nxt_p
    if  abs (i_cur_r - j_cur_r) + abs (i_cur_c - j_cur_c) == 1 and
      abs (i_cur_r - j_nxt_r) + abs (i_cur_c - j_nxt_c) == 1:
        return True
    return False


def f_iscross_diag(Ped_i, Ped_j):
    '''
    función que determina si al siguiente paso el peatón cruza en diagnal con
    otro peatón
    '''
    if f_other_crosses(Ped_i, Ped_j) and f_other_crosses(Ped_j, Ped_i):
        return True
    return False

def generate5x5neigh(Ped_i):
    '''
    función que genera la vecindad 5x5 que rodea a Ped_i, considerando como
    obstáculos a todos los peatones y sus siguientes pasos en esta vecindad

    TERMINAR: definir qué es obstaculo
                definir si la celda central es obstáculo
                definir qué es PEATON
                definir obstNeigh(r,c): regresa cualquier obstáculo
    '''
    neigh_ped_i = [[0 for c in range(-2,3)] for r in range(-2,3)]

    for r in range(-2,3):
        for j in range(-2,3):
            if r == 0 and j == 0:
                neigh_ped_i[r][c] = "OBSTACULO????"
                continue

            if Ped_i.row - r < 0 or Ped_i.row + r >= NUM_ROWS or
              Ped_i.col - c < 0 or Ped_i.col + c >= NUM_COLS:
                neigh_ped_i[r][c] = "OBSTACULO"
            else:
                neigh_ped_i[r][c] = obstNeigh(r,c)
                if Grid[r][c] == "PEATON":
                    if Ped_i.row-2 <= Grid[r][c].PEATON.r_nxt_p  <= Ped_i.row+2 and
                      Ped_i.col-2 <= Grid[r][c].PEATON.c_nxt_p  <= Ped_i.col+2:
                        # el siguiente paso del peatón se vuelve un obstáculo
                        neigh_ped_i[  Grid[r][c].PEATON.r_nxt_p - Ped_i.row ][ Grid[r][c].PEATON.c_nxt_p - Ped_i.col ] = "OBSTACULO"

    return neigh_ped_i

def f_free_in_3x3(Ped_i, neigh_ped_i):
    '''
    función que devuelve una lista de diccionarios con las posiciones libres
    en una vecindad 3x3 con el origen el centro

    TERMINAR: definir OBSTACULO
    '''
    l_free = []
    for i in range(1,4):
        for j in range(1,4):
            if i  == 2 and j == 2:
                continue
            if neigh_ped_i[i][j] != "OBSTACULO":
                l_free.append((Ped_i.row + i, Ped_i.col + j))
    return l_free


def f_free_next_cell(Ped_i, Grid):
    '''
    función para "liberar" a la siguiente casilla de Ped_i, en caso de que ya
    se considere ahí
    '''
    nxt_r = Ped_i.r_nxt_p
    nxt_c = Ped_i.c_nxt_p
    if Grid[nxt_r][nxt_c] == Ped_i:
        Grid[nxt_r][nxt_c] = None


def f_action2pedSameRoute(Ped_i, Ped_j, Grid):
    '''
    función que se usa para determinar qué acción resulta de la posible colisión
    entre dos peatones en colisiones f_issame_cell_arrival y f_iscross_diag
    '''
    if Ped_i.inercia > Ped_j.inercia:

        Ped_i.estado = "walking"
        nxt_r = Ped_i.r_nxt_p
        nxt_c = Ped_i.c_nxt_p
        Grid[nxt_r][nxt_c] = Ped_i

    else:
        Ped_i.estado = "waiting"
        f_free_next_cell(Ped_i, Grid)


def f_nuevoPaso(Ped_i):
    '''
    función que determina qué acción debe tomar el peatón que debe cambiar
    de dirección
    '''
    # vecindad 5x5
    neigh_ped_i = generate5x5neigh(Ped_i)
    #lista de celdas libres
    if l_free:
        '''TOMAR UN ELEMENTO "al azar" n_step'''
        Ped_i.next_step.row = n_step.row
        Ped_i.next_step.col = n_step.col
        Ped_i.calculate_path_in_next_step = True


def f_evationPairPedestrians(l_pedestrians, Grid):
    '''
    función que obtiene la evolución de los peatoncitos

    aquí está el ciclo anidado
    '''
    for i in range(len(l_pedestrians)):
        Ped_i = l_pedestrians[i]
        Ped_i.estado = "walking"

        for j in range(len(l_pedestrians)):
            Ped_j = l_pedestrians[j]

            if j != i:
                if f_isfront2front(Ped_i, Ped_j):
                    Ped_i.estado = "waiting"
                    if Ped_i.inercia < Ped_j.inercia:
                        f_free_next_cell(Ped_i, Grid)
                        f_nuevoPaso(Ped_i)
                    break

                elif f_isto_waiter(Ped_i, Ped_j):
                    Ped_i.estado = "waiting"
                    f_free_next_cell(Ped_i, Grid)
                    f_nuevoPaso(Ped_i)
                    break

                elif f_issame_cell_arrival(Ped_i, Ped_j):
                    f_action2pedSameRoute(Ped_i, Ped_j, Grid)

                elif f_islet_pass(Ped_i, Ped_j):
                    f_free_next_cell(Ped_i)

                elif f_iscross_diag(Ped_i, Ped_j):
                    f_action2pedSameRoute(Ped_i, Ped_j, Grid)
