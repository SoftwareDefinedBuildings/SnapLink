import sys
import numpy as np
import matplotlib.pyplot as plt

def str_to_data(labels, line):
    data = []
    last_label = None
    last_data_value = False
    values = [int(s.strip()) for s in line.split(",")[1:]] # values skip first col
    for i in range(len(values)):
        value = values[i]
        label = labels[i]
        if label == last_label:
            last_data_value = last_data_value or (value != 0 and value != 3)
        else:
            if last_label:
                data.append(last_data_value)
            last_data_value = (value != 0 and value != 3)
            last_label = label
    if last_label:
        data.append(last_data_value)
    return data

def recall(truth_data, result_data):
    assert len(truth_data) == len(result_data)
    truth_sum = sum(truth_data)
    if truth_sum > 0:
        correct_result_sum = sum(x == y for x,y in zip(truth_data, result_data) if x)
        return float(correct_result_sum)/truth_sum
    else:
        return 1.0

def precision(truth_data, result_data):
    assert len(truth_data) == len(result_data)
    result_sum = sum(result_data)
    if result_sum > 0:
        correct_result_sum = sum(x == y for x,y in zip(truth_data, result_data) if y)
        return float(correct_result_sum)/result_sum
    else:
        return 1.0


def run():
    truth_file = open(sys.argv[-2], 'r')
    result_file = open(sys.argv[-1], 'r')
    recall_data = []
    precision_data = []
    
    header = [s.strip() for s in truth_file.readline().split(",")]
    labels = [s.split(' ')[0] for s in header[1:]] # labels skip first col
    result_file.readline()
    
    for truth_line in truth_file:
        result_line = result_file.readline()
        truth_data = str_to_data(labels, truth_line)
        result_data = str_to_data(labels, result_line)
        recall_data.append(recall(truth_data, result_data))
        precision_data.append(precision(truth_data, result_data))
    
    plt.plot(map(lambda x: 100 * x, recall_data), 'bo-')
    plt.plot(map(lambda x: 100 * x, precision_data), 'rx-')
    plt.ylim(0, 100)
    plt.title('Recall and Precision of RTAB-Map multisession data')
    plt.ylabel('%')
    
    plt.show()
    
if __name__ == "__main__":
    run()
