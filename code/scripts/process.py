#!/bin/python3
import sys
import statistics

def main():
    fname = sys.argv[1];
    f = open(fname, 'r');

    data = []
    # read from the data file into a list, splitting on the space delimiter
    for i, line in enumerate(f):
        data.append(line.split(' '))

    nfiles = len(data)
    filerange = range(1, nfiles);
    results = dict() # store results in a dictionary
    
    # get each column of data, ignoring the first row containing column headings
    orig_pts = [int(data[i][1]) for i in filerange]
    downsample_pts = [int(data[i][2]) for i in filerange]
    trim_pts = [int(data[i][3]) for i in filerange]
    plane_pts = [int(data[i][4]) for i in filerange]
    num_planes = [int(data[i][9]) for i in filerange]
    load_time = [float(data[i][5]) for i in filerange]
    downsample_time = [float(data[i][6]) for i in filerange]
    trim_time = [float(data[i][7]) for i in filerange]
    normals_time = [float(data[i][8]) for i in filerange]
    plane_time = [float(data[i][10]) for i in filerange]
    time_per_plane = [plane_time[i]/num_planes[i] for i in range(0, nfiles - 1)]

    print(num_planes)
    print(plane_time)
    print(time_per_plane)

    results['orig_pts_mean'] = statistics.mean(orig_pts_cloud)
    results['orig_pts_std'] = statistics.stdev(orig_pts_cloud)

    print(results)
    
if __name__ == '__main__':
    main()

