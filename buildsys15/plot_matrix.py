import sys
import numpy as np
import matplotlib.pyplot as plt
import matplotlib as mpl

y_labels = ["Trash Can Blue", \
            "Storage Box 1", \
            "Trash Can Black", \
            "Storage Box 2", \
            "Door 1", \
            "Door 2", \
            "Red Chair", \
            "Robot 2", \
            "Robot 1", \
            "Ladder", \
            ]
type_labels = ["1: Not Visible, Identified", \
               "2: Not Visible, Not Identified", \
               "3: Visible, Identified", \
               "4: Visible, Not Identified", \
               "5: Occluded, Identified", \
               "6: Occluded, Not Identified", \
               "7: Partially Visible, Identified", \
               "8: Partially Visible, Not Identified", \
               ]


def str_to_truth_data(labels, line):
    def update_value(last_value, new_value):
        if last_value == 0:
            return new_value
        elif last_value == 1:
            return 1
        elif last_value == 2:
            if new_value == 1 or new_value == 3:
                return new_value
            else:
                return last_value
        elif last_value == 3:
            if new_value == 1:
                return new_value
            else:
                return last_value
    data = []
    last_label = None
    last_data_value = None
    values = [int(s.strip()) for s in line.split(",")[1:]] # values skip first col
    for i in range(len(values)):
        value = values[i]
        label = labels[i]
        if label == last_label:
            last_data_value = update_value(last_data_value, value)
        else:
            if last_label:
                data.append(last_data_value)
            last_data_value = value
            last_label = label
    if last_label:
        data.append(last_data_value)
    return data

def str_to_result_data(labels, line):
    data = []
    last_label = None
    last_data_value = False
    values = [int(s.strip()) for s in line.split(",")[1:]] # values skip first col
    for i in range(len(values)):
        value = values[i]
        label = labels[i]
        if label == last_label:
            last_data_value = last_data_value or (value != 0)
        else:
            if last_label:
                data.append(last_data_value)
            last_data_value = (value != 0) 
            last_label = label
    if last_label:
        data.append(last_data_value)
    return data

def result_type(truth_value, result_value):
    if truth_value == 0:
        if result_value:
            return 1
        else:
            return 2
    elif truth_value == 1:
        if result_value:
            return 3
        else:
            return 4
    elif truth_value == 2:
        if result_value:
            return 5
        else:
            return 6
    elif truth_value == 3:
        if result_value:
            return 7
        else:
            return 8

    print truth_value, result_value

def plot_matrix(type_matrix):
    global y_labels
    
    min_x, max_x = -0.5, type_matrix.shape[1] - 0.5
    min_y, max_y = -0.5, type_matrix.shape[0] - 0.5
    x_ticks = np.arange(min_x, max_x, 1.0)
    y_ticks = np.arange(min_y, max_y, 1.0)

    # color
    colors = mpl.cm.cubehelix(np.linspace(0, 1, len(type_labels)))
    cmap = mpl.colors.ListedColormap(colors)
    
    fig, ax = plt.subplots()
    cmat = ax.matshow(type_matrix, cmap=cmap, vmin = 1-0.5, vmax = len(type_labels) + 0.5)
    ax.set_xlim(min_x, max_x)
    ax.set_ylim(min_y, max_y)
    ax.xaxis.set_ticks(x_ticks, minor=True)
    ax.xaxis.set(ticks=x_ticks + 0.5, ticklabels=range(1, len(x_ticks)+1))
    # remove tick but keep label
    for tick in ax.xaxis.get_major_ticks():
        tick.tick1line.set_markersize(0)
        tick.tick2line.set_markersize(0)
        tick.label1.set_horizontalalignment('center')
    ax.yaxis.set_ticks(y_ticks, minor=True)
    ax.yaxis.set(ticks=y_ticks + 0.5, ticklabels=y_labels)
    # remove tick but keep label
    for tick in ax.yaxis.get_major_ticks():
        tick.tick1line.set_markersize(0)
        tick.tick2line.set_markersize(0)
        tick.label1.set_verticalalignment('center')
    ax.grid(True, which='minor')

    #text
    x, y = np.meshgrid(x_ticks, y_ticks)
    for i in range(len(x_ticks)):
        for j in range(len(y_ticks)):
            x_val = x_ticks[i] + 0.5
            y_val = y_ticks[j] + 0.5
            type_val = int(type_matrix[int(y_val), int(x_val)])
            ax.text(x_val, y_val, type_val, va='center', ha='center')

    # color
    cax = plt.colorbar(cmat, ticks=np.arange(1, len(type_labels)+1), shrink=.3, pad=.005)
    cax.set_ticklabels(type_labels)
    for tick in cax.ax.yaxis.get_major_ticks():
        tick.tick1line.set_markersize(0)
        tick.tick2line.set_markersize(0)
        tick.label1.set_verticalalignment('center')



    plt.show()

def run():
    truth_file = open(sys.argv[-2], 'r')
    result_file = open(sys.argv[-1], 'r')
    recall_data = []
    precision_data = []
    
    header = [s.strip() for s in truth_file.readline().split(",")]
    labels = [s.split(' ')[0] for s in header[1:]] # labels skip first col
    result_file.readline()

    truth_lines = truth_file.readlines()
    result_lines = result_file.readlines()
    
    type_matrix = np.zeros((len(set(labels)), len(truth_lines))) # matrix has images as col
    for i in range(len(truth_lines)):
        truth_line = truth_lines[i]
        result_line = result_lines[i]
        truth_data = str_to_truth_data(labels, truth_line)
        result_data = str_to_result_data(labels, result_line)
        for j in range(len(truth_data)):
            truth_value = truth_data[j]
            result_value = result_data[j]
            type_matrix[j, i] = result_type(truth_value, result_value)

    print type_matrix
    plot_matrix(type_matrix)

    
    
if __name__ == "__main__":
    run()
