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

    results['orig_pts_mean'] = statistics.mean(orig_pts)
    results['orig_pts_std'] = statistics.stdev(orig_pts)
    results['downsample_pts_mean'] = statistics.mean(downsample_pts)
    results['downsample_pts_std'] = statistics.stdev(downsample_pts)
    results['trim_pts_mean'] = statistics.mean(trim_pts)
    results['trim_pts_std'] = statistics.stdev(trim_pts)
    results['plane_pts_mean'] = statistics.mean(plane_pts)
    results['plane_pts_std'] = statistics.stdev(plane_pts)
    results['num_planes_mean'] = statistics.mean(num_planes)
    results['num_planes_std'] = statistics.stdev(num_planes)
    results['load_time_mean'] = statistics.mean(load_time)
    results['load_time_std'] = statistics.stdev(load_time)
    results['downsample_time_mean'] = statistics.mean(downsample_time)
    results['downsample_time_std'] = statistics.stdev(downsample_time)
    results['trim_time_mean'] = statistics.mean(trim_time)
    results['trim_time_std'] = statistics.stdev(trim_time)
    results['normals_time_mean'] = statistics.mean(normals_time)
    results['normals_time_std'] = statistics.stdev(normals_time)
    results['plane_time_mean'] = statistics.mean(plane_time)
    results['plane_time_std'] = statistics.stdev(plane_time)
    results['time_per_plane_mean'] = statistics.mean(time_per_plane)
    results['time_per_plane_std'] = statistics.stdev(time_per_plane)

    for key in sorted(results):
        print("%s: %s" % (key, results[key]))
    
if __name__ == '__main__':
    main()

