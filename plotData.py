import os
import os.path as osp
import numpy as np
import matplotlib.pyplot as plt

MAX_DATA_POINTS = 12

## PURPOSE: find the files given by 'filenames' in the 'data' folder
## INPUT:   - filename (str[]): name of files to process
## OUTPUT:  - axis (str[]): x-axis labels
##          - data (arr[float[]]): array of data values for each
##            x-value (one per file)
##          - mean (arr[float[]]): mean of data values for each
##            x-value (one per file)
def get_data( filenames ):
    axis = []
    data = []
    mean = []
    for i in range( len(filenames) ):
        filepath = "data/" + filenames[i]
        if not os.path.isfile( filepath ):
            print "===> \"" + filenames[i] + "\"", "is not a valid file in the data folder."
            continue
        with open( filepath ) as filename:
            file_data = []
            file_mean = []
            for line in filename:
                words = line.split(", ")
                axis.append(words[0])
                words = words[1:]
                datavals = [float(w) for w in words]
                file_data.append(datavals)
                file_mean.append(sum(datavals)/len(datavals))
            data.append(file_data)
            mean.append(file_mean)
    if len(data) > 0:
        axis = axis[:len(data[0])]
    return axis, data, mean

## PURPOSE: to better display the data, truncate float axis labels
##          and hide some by setting them to empty strings
## INPUT:   - axis (str[]): original x-axis labels
## OUTPUT:  - axis (str[]): modified x-axis labels
def cleanup_axis( axis ):
    if axis[0].find("."):
        ## If float, truncate to 2 decimal places
        axis = ['%.2f'%(float(x)) for x in axis]
        # print axis
    if len(axis) > MAX_DATA_POINTS:
        step = int(len(axis)/MAX_DATA_POINTS)
        for i in range(len(axis)):
            if i % step != 0 and i != len(axis)-1:
                axis[i] = ""
        # print axis
    return axis

## PURPOSE: plots graphs
## OUTPUT:  - axis (str[]): x-axis labels
##          - data (arr[float[]]): array of data values for each
##            x-value (one per file)
##          - mean (arr[float[]]): mean of data values for each
##            x-value (one per file)
##          - xlabel (str[]): label for the x-axis
##          - ylabel (str[]): label for the y-axis
## OUTPUT:  (NONE)
def plot( axis, data, mean, xlabel, ylabel ):
    # Random test data
    # np.random.seed(1231)
    # all_data = [np.random.normal(0, std, 100) for std in range(1, 4)]
    fig, axes = plt.subplots(nrows=1, ncols=len(data),
def get_data( filenames ):
    axis = []
    data = []
    mean = []
    for i in range( len(filenames) ):
        filepath = "data/" + filenames[i]
        if not os.path.isfile( filepath ):
            print "===> \"" + filenames[i] + "\"", "is not a valid file in the data folder."
            continue
        with open( filepath ) as filename:
            file_data = []
            file_mean = []
            for line in filename:
                words = line.split(", ")
                axis.append(words[0])
                words = words[1:]
                datavals = [float(w) for w in words]
                file_data.append(datavals)
                file_mean.append(sum(datavals)/len(datavals))
            data.append(file_data)
            mean.append(file_mean)
    if len(data) > 0:
        axis = axis[:len(data[0])]
    return axis, data, mean

def main():
    names = raw_input("===> Enter filename (include .txt and separate by space): ").split()
    xlabel = raw_input("===  xlabel(s): ").split()
    ylabel = raw_input("===  ylabel(s): ").split()
    axis, data, mean = get_data(names)
    if not axis:
        print "===> Nothing to display."
    else:
        print "===> Generating plot..."
        plot( axis, data, mean, xlabel, ylabel )

if __name__ == "__main__":
    main()
