#!/bin/python3
import sys
import statistics
import os

# to convert to better format, use org-table-convert-region and org-table-transpose-at-point
def results_to_tex(results, addheader=False):
    number_line = ""
    if (addheader):
        number_line += "Setting & Original & Downsampled & Trimmed & NPlanes & Planes & Final\\\\\hline\n"
    else:
        number_line += "\n"


    #        "{:,}".format(number);
    number_line += os.path.basename(results['fname']) + " & "
    number_line += "{:,}".format(results['orig_pts_mean']) + "$\pm$" + "{:,}".format(results['orig_pts_std']) + " & "
    number_line += "{:,}".format(results['downsample_pts_mean']) + "$\pm$" + "{:,}".format(results['downsample_pts_std']) + " & "
    number_line += "{:,}".format(results['trim_pts_mean']) + "$\pm$" + "{:,}".format(results['trim_pts_std']) + " & "
    number_line += "%.2f" % results['num_planes_mean']  + "$\pm$" +  "%.2f" % results['num_planes_std'] + " & "
    number_line += "{:,}".format(results['plane_pts_mean']) + "$\pm$" + "{:,}".format(results['plane_pts_std']) + " & "
    number_line += "{:,}".format(results['final_pts_mean']) + "$\pm$" + "{:,}".format(results['final_pts_std']) + "\\\\"

    time_line = ""
    if (addheader):
        time_line += "Setting & Load & Downsample & Trim & Normals & Planes & PerPlane & Annotation & NormalsF \\\\\hline\n"
    else:
        time_line += "\n"
    time_line += os.path.basename(results['fname']) + " & "
    time_line += "%.2f" % results['load_time_mean']  + "$\pm$" +  "%.2f" % results['load_time_std'] + " & "
    time_line += "%.2f" % results['downsample_time_mean']  + "$\pm$" +  "%.2f" % results['downsample_time_std'] + " & "
    time_line += "%.2f" % results['trim_time_mean']  + "$\pm$" +  "%.2f" % results['trim_time_std'] + " & "
    time_line += "%.2f" % results['normals_time_mean']  + "$\pm$" +  "%.2f" % results['normals_time_std'] + " & "
    time_line += "%.2f" % results['plane_time_mean']  + "$\pm$" +  "%.2f" % results['plane_time_std'] + " & "
    time_line += "%.2f" % results['time_per_plane_mean']  + "$\pm$" +  "%.2f" % results['time_per_plane_std'] + " "
    if ('annotation_time_mean' in results):
        time_line += "& %.2f" % results['annotation_time_mean']  + "$\pm$" +  "%.2f" % results['annotation_time_std'] + " & "
    if ('feature_normal_time_mean' in results):
        if ('annotation_time_mean' not in results):
            time_line += "& - "
        time_line += "& %.2f" % results['feature_normal_time_mean']  + "$\pm$" +  "%.2f" % results['feature_normal_time_std'] + ""
        
    time_line += "\\\\"
    return number_line, time_line

def processFile(fname):
    f = open(fname, 'r')
    data = []
    # read from the data file into a list, splitting on the space delimiter
    for i, line in enumerate(f):
        data.append(line.split(' '))

    nfiles = len(data)
    ncols = len(data[0])
    filerange = range(1, nfiles)
    results = dict() # store results in a dictionary
    results['fname'] = fname
    
    # get each column of data, ignoring the first row containing column headings
    orig_pts = [int(data[i][1]) for i in filerange]
    downsample_pts = [int(data[i][2]) for i in filerange]
    trim_pts = [int(data[i][3]) for i in filerange]
    plane_pts = [int(data[i][4]) for i in filerange]
    load_time = [float(data[i][5]) for i in filerange]
    downsample_time = [float(data[i][6]) for i in filerange]
    trim_time = [float(data[i][7]) for i in filerange]
    normals_time = [float(data[i][8]) for i in filerange]
    num_planes = [int(data[i][9]) for i in filerange]
    plane_time = [float(data[i][10]) for i in filerange]

    # sometimes these values do not exist, so we put zeros in that column or
    # part of that column instead so need to exclude those zeros from the count
    if (ncols > 11):
        feature_normal_time = [float(data[i][11]) for i in filerange if (float(data[i][11]) > 0.00001)]
    if (ncols > 12):
        annotation_time = [float(data[i][12]) for i in filerange if (float(data[i][12]) > 0.00001)]

    # average time to extract each plane
    time_per_plane = [(plane_time[i]/num_planes[i] if num_planes[i] != 0 else plane_time[i]) for i in range(0, nfiles - 1)]
    # size of clouds after preprocessing
    final_size = [trim_pts[i]-plane_pts[i] for i in range(0, len(orig_pts))]
    # relative size of original and downsampled
    downsample_prop = [downsample_pts[i]/orig_pts[i] for i in range(0, len(orig_pts))]

    results['orig_pts_mean'] = int(statistics.mean(orig_pts))
    results['orig_pts_std'] = int(statistics.stdev(orig_pts))
    results['downsample_pts_mean'] = int(statistics.mean(downsample_pts))
    results['downsample_pts_std'] = int(statistics.stdev(downsample_pts))
    results['trim_pts_mean'] = int(statistics.mean(trim_pts))
    results['trim_pts_std'] = int(statistics.stdev(trim_pts))
    results['plane_pts_mean'] = int(statistics.mean(plane_pts))
    results['plane_pts_std'] = int(statistics.stdev(plane_pts))
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

    if (ncols > 11):
        results['feature_normal_time_mean'] = statistics.mean(feature_normal_time)
        results['feature_normal_time_std'] = statistics.stdev(feature_normal_time)
    if (ncols > 12):
        results['annotation_time_mean'] = statistics.mean(annotation_time)
        results['annotation_time_std'] = statistics.stdev(annotation_time)
    
    downsample_prop_mean = statistics.mean(downsample_prop)
    
    results['final_pts_mean'] = int(statistics.mean(final_size))
    results['final_pts_std'] = int(statistics.stdev(final_size))

    return results

    # for key in sorted(results):
    #     print("%s: %s" % (key, results[key]))

    # texed = results_to_tex(results, True)
    # print(texed[0])
    # print(texed[1])
    
def main():
    args = sys.argv[1:]
    results = []
    # go through all the filenames and extract results from each one
    for fname in args:
        print("Processing %s" % fname)
        results.append(processFile(fname))

    numbers = ""
    times = ""
    addhead = True
    for result in results:
        textuple = results_to_tex(result, addhead)
        numbers += textuple[0]
        times += textuple[1]
        addhead = False

    print(numbers)
    print(times)

if __name__ == '__main__':
    main()

