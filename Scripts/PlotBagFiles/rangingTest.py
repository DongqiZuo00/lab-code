#This simple program is to be used with "quad test ranging"
#Put ranging measurements got by "quad test ranging" into a text document
#and use this .py program to plot the histogram of those measurements
#It calculates the mean, median and std of the good measurements as well as good 
#measurements' ratio.

import csv
import numpy as np
import matplotlib.pyplot as plt

file_path = '/home/xiangyu/Desktop/ranging_data/data3'

with open(file_path, 'r') as csv_file:
    csv_reader = csv.reader(csv_file, delimiter=',',quotechar='"')
    data_list = list(csv_reader)
    data_vector_string = np.squeeze(data_list)
    data_vector_string = data_vector_string[data_vector_string!=''] #remove empty element
    data_vector_float = data_vector_string.astype(float)
    good_data_vector_float = data_vector_float[data_vector_float>0] #only analyze good measurement
    
    plt.hist(good_data_vector_float, bins='auto')
    plt.title("Histogram of good ranging measurement")
    plt.xlabel('distance(m)')
    plt.ylabel('count')
    plt.show()
    
    mean = np.mean(good_data_vector_float)
    median = np.median(good_data_vector_float)
    std = np.std(good_data_vector_float)
    print('mean of good measurements')
    print(mean)
    print('median of good measurements')
    print(median)
    print('std of good measurements')
    print(std)
    print('good measurement ratio')
    print(float(len(good_data_vector_float))/float(len(data_vector_float)))