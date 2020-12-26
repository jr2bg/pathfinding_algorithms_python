import csv

# función para leer el maze en formato CSV
# asignamos el caracter asociado a obstáculo
def read_maze_csv(path_csv, chr_obs = "#"):
    #en los archivos, "1" es libre y "0" es ostáculo
    with open(path_csv, newline="") as maze:
        reader = csv.reader(maze)
        data = list(reader)

    # lista de obstáculos
    l_obst = []

    n_rows = len(data)
    n_cols = len(data[0])

    # para cada fila
    for i in range(n_rows):

        #para cada columna
        for j in range(n_cols):
            if data[i][j] == chr_obs:
                l_obst.append((i,j))

    return n_rows, n_cols, l_obst


if __name__ == "__main__":
    path_csv = "/home/jr2bg/Documents/masters/CIC/thesis/ai-maze-python/mazes/maze0.csv"
    chr_obs = "0"
    n_rows, n_cols, l_obst = read_maze_csv(path_csv, chr_obs = chr_obs)
    print(l_obst)
