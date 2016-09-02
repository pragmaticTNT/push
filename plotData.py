import os
import os.path as osp
import numpy as np
import matplotlib.pyplot as plt

## ============================================================================
## Code for 'plot' function courtesy: matplotlib/gallery -
##      statistics example code: boxplot_color_demo.py
## Latest Revision: 2 September 2016
## ============================================================================

MAX_DATA_POINTS = 12

## PURPOSE: find the files given by 'filenames' in the 'data' folder
## INPUT:   - filename (str[]): name of files to process
## OUTPUT:  - xaxis (str[]): x-axis labels
##          - data (arr[float[]]): array of data values for each
##            x-value (one per file)
##          - mean (arr[float[]]): mean of data values for each
##            x-value (one per file)
def get_data( filenames ):
    xaxis = []
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
                xaxis.append(words[0])
                words = words[1:]
                datavals = [float(w) for w in words]
                file_data.append(datavals)
                file_mean.append(sum(datavals)/len(datavals))
            data.append(file_data)
            mean.append(file_mean)
    if len(data) > 0:
        xaxis = xaxis[:len(data[0])]
    return xaxis, data, mean

## PURPOSE: to better display the data, truncate float axis labels
##          and hide some by setting them to empty strings
## INPUT:   - axis (str[]): original axis labels
## OUTPUT:  - axis (str[]): modified axis labels
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
## OUTPUT:  - xaxis (str[]): x-axis labels
##          - data (arr[float[]]): array of data values for each
##            x-value (one per file)
##          - mean (arr[float[]]): mean of data values for each
##            x-value (one per file)
##          - xlabel (str[]): label for the x-axis
##          - ylabel (str[]): label for the y-axis
## OUTPUT:  (NONE)
def plot( xaxis, data, mean, xlabel, ylabel ):
    fig, axes = plt.subplots(nrows=1, ncols=len(data), figsize=(12, 5))
    axis_marker = [y+1 for y in range(len(data[0]))]

    if len(data) > 1:
        # Data 1: box plot
        bplot1 = axes[0].boxplot(data[0],
                                 vert=True,   # vertical box aligmnent
                                 patch_artist=True)   # fill with color
        # Data 1: plot mean
        mplot1 = axes[0].plot(axis_marker, mean[0], 'r+')
        # Data 2: box plot
        bplot2 = axes[1].boxplot(data[1],
                                 vert=True,   # vertical box aligmnent
                                 patch_artist=True)   # fill with color
        # Data 2: plot mean
        mplot2 = axes[1].plot(axis_marker, mean[1], 'r+')

        # Fill bars with color
        color = 'lightblue'
        for bplot in (bplot1, bplot2):
            for patch in bplot['boxes']:
                patch.set_facecolor(color)

        # adding horizontal grid lines
        for i in range(len(data)):
            axes[i].yaxis.grid(True)
            axes[i].set_xticks(axis_marker, )
            axes[i].set_xlabel(xlabel[i])
            axes[i].set_ylabel(ylabel[i])
    else:
        # Data box plot
        bplot = axes.boxplot(data[0], patch_artist=True)   # fill with color
        # Plot mean
        mplot = axes.plot(axis_marker, mean[0], 'r+')

        # Fill bars with color
        color = 'lightblue'
        for patch in bplot['boxes']:
            patch.set_facecolor(color)

        # adding horizontal grid lines
        axes.yaxis.grid(True)
        axes.set_xticks(axis_marker, )
        axes.set_xlabel(xlabel[0])
        axes.set_ylabel(ylabel[0])


    # add x-tick labels
    xaxis = cleanup_axis( xaxis )
    plt.setp( axes, xticks = axis_marker, xticklabels = xaxis )

    plt.show()

def main():
    names = raw_input("===> Enter filename (include .txt and separate by space): ").split()
    xlabel = raw_input("===  xlabel(s): ").split()
    ylabel = raw_input("===  ylabel(s): ").split()
    xaxis, data, mean = get_data(names)
    if not xaxis:
        print "===> Nothing to display."
    else:
        print "===> Generating plot..."
        plot( xaxis, data, mean, xlabel, ylabel )

if __name__ == "__main__":
    main()
