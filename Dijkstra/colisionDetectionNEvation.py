def f_isfront2front(Ped_i, Ped_j):
    '''
    función que determina si los peatones tienen un conflicto front 2 front
    '''
    i_cur_r = Ped_i.row
    i_cur_c = Ped_i.col
    i_nxt_r = Ped_i.next.row
    i_nxt_c = Ped_i.next.col

    j_cur_r = Ped_j.row
    j_cur_c = Ped_j.col
    j_nxt_r = Ped_j.next.row
    j_nxt_c = Ped_j.next.col

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
    i_nxt_r = Ped_i.next.row
    i_nxt_c = Ped_i.next.col

    j_nxt_r = Ped_j.next.row
    j_nxt_c = Ped_j.next.col


    if i_nxt_c == j_nxt_c and i_nxt_r == j_nxt_r:
        return True
    return False


def f_islet_pass(Ped_i, Ped_j):
    '''
    función que determina si al siguiente paso el peatón llega a una celda
    actualmente ocupada por un peatón que se desplaza
    '''
    i_nxt_r = Ped_i.next.row
    i_nxt_c = Ped_i.next.col

    j_cur_r = Ped_j.row
    j_cur_c = Ped_j.col
    j_nxt_r = Ped_j.next.row
    j_nxt_c = Ped_j.next.col

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
    i_nxt_r = Ped_i.next.row
    i_nxt_c = Ped_i.next.col

    j_cur_r = Ped_j.row
    j_cur_c = Ped_j.col
    j_nxt_r = Ped_j.next.row
    j_nxt_c = Ped_j.next.col

    if i_nxt_r == j_cur_r and j_cur_c == i_nxt_c and Ped_j.estatus == "waiting":
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
    j_nxt_r = Ped_j.next.row
    j_nxt_c = Ped_j.next.col
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
    función que genera la vecindad 5x5 que rodea a ped_i, considerando como
    obstáculos a todos los peatones y sus siguientes pasos en esta vecindad

    TERMINAR LA CONSTRUCCIÓN, no todas las vecindades pueden ser de 5x5
    '''
    if Ped_i.row > 1:
        from_r = -2
    else:
        from_r = -2 +
    neigh_ped_i = [[ for c in range(-2,3)] for r in range(-2,3)]

    return neigh_ped_i

def f_free_in_3x3(neigh_ped_i):
    '''
    función que devuelve una lista de diccionarios con las posiciones libres
    en una vecindad 3x3 con el origen el centro

    TERMINAR EL CICLO
    '''
    l_free = []
    for celda in neigh_ped_i:
        if celda != centro and celda.is_free():
            l_free.append(celda)
    return l_free


def f_free_next_cell(Ped_i, Grid):
    '''
    función para "liberar" a la siguiente casilla de Ped_i, en caso de que ya
    se considere ahí
    '''
    nxt_r = Ped_i.next.row
    nxt_c = Ped_i.next.col
    if Grid[nxt_r][nxt_c] == Ped_i:
        Grid[nxt_r][nxt_c] = None


def f_action2pedSameRoute(Ped_i, Ped_j, Grid):
    '''
    función que se usa para determinar qué acción resulta de la posible colisión
    entre dos peatones en colisiones f_issame_cell_arrival y f_iscross_diag
    '''
    if Ped_i.inercia > Ped_j.inercia:

        Ped_i.estatus = "walking"
        nxt_r = Ped_i.next.row
        nxt_c = Ped_i.next.col
        Grid[nxt_r][nxt_c] = Ped_i

    else:
        Ped_i.estatus = "waiting"
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
    '''
    for i in range(len(l_pedestrians)):
        for j in range(len(l_pedestrians)):
            Ped_i = l_pedestrians[i]
            Ped_j = l_pedestrians[j]

            if j != i:
                if f_isfront2front(Ped_i, Ped_j):
                    Ped_i.estatus = "waiting"
                    if Ped_i.inercia < Ped_j.inercia:
                        f_free_next_cell(Ped_i, Grid)
                        f_nuevoPaso(Ped_i)
                    break

                elif f_isto_waiter(Ped_i, Ped_j):
                    Ped_i.estatus = "waiting"
                    f_free_next_cell(Ped_i, Grid)
                    f_nuevoPaso(Ped_i)
                    break

                elif f_issame_cell_arrival(Ped_i, Ped_j):
                    f_action2pedSameRoute(Ped_i, Ped_j, Grid)

                elif f_islet_pass(Ped_i, Ped_j):
                    f_free_next_cell(Ped_i)

                elif f_iscross_diag(Ped_i, Ped_j):
                    f_action2pedSameRoute(Ped_i, Ped_j, Grid)
