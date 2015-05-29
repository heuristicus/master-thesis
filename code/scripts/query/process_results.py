#!/usr/bin/python3
import sys
import os
import statistics

# move the pointer in an open file to the first line which contains data
# if skip is false, do not skip the next line
def go_to_data(openfile, skip=True):
    for line in openfile:
        if (line.find("BEGIN_DATA") != -1) :
            if (skip):
                openfile.next()
            return

# get the cluster file corresponding to the file name
def get_corresp_clusters(fname):
    return os.path.splitext(fname)[0] + "_clusters.txt"

# convert string with comma separated values to a list containing ints
def strlist_to_int_list(strlist):
    return list(map(int,strlist.split(',')))

# find the length of the longest sublist in a list of lists
def max_sublist_len(lst):
    return max(list(map(len, lst)))

# convert a list of lists to another list of lists, but which contains in each
# index the list of elements at corresponding indices in the given list if the
# addzero flag is set to true, then we ensure that all columnised lists are the
# same length by padding them with zeroes when there is no value in a given
# column for a specific sublist
def columnise_sublists(lst, addzero=False):
    maxind = max_sublist_len(lst)
    cols=[[] for x in range(0, maxind)]
    for subl in lst:
        if (subl[0] == -1):
            continue
        for i in range(0,maxind):
            try:
                cols[i].append(subl[i])
            except IndexError:
                if (addzero):
                    cols[i].append(0)

    return cols

def get_mean_histogram(lst):
    columnised=columnise_sublists(lst, True)
    means=[]
    stdevs=[]
    for col in columnised:
        means.append("%.2f" % statistics.mean(col))
        stdevs.append("%.2f" % statistics.stdev(col))
        
    return means,stdevs

def single_hist_to_gnuplot(hist, stdev=[]):
    lines = ""
    for i, abin in enumerate(hist):
        if (i == 0):
            continue
        lines += str(i) + " " + str(abin)
        if (stdev):
            lines += " " + stdev[i]
        lines += "\n"

    return lines

def multiple_hist_to_gnuplot(hists, stdevs=[]):
    lines = ""
    nhists = len(hists)
    rng = 1
    percol = round(rng/nhists, 2)
    maxlen = max_sublist_len(hists)
    for i in range(0, maxlen):
        for j,hist in enumerate(hists):
            lines += str(i-0.2 + j * percol) + " "
            try:
                lines += str(hist[i]) + " " + str(stdevs[j][i]) + " "
            except IndexError:
                lines += "0 0 "

        lines += "\n"
        
    return lines
        
def get_data(fname):
    f = open(fname, 'r')
    go_to_data(f, False)
    headings = ""
    data = {}
    # information about the object and the interest point and descriptors used
    # to find them
    finfo=fname.split('/')[-3:-1]
    data['objlabel'] = finfo[0]
    # 0 contains interest, 1 contains desc
    idesc=finfo[1].split('-')[0].split('_')
    data['interest']=idesc[0]
    data['feature']=idesc[1]
    # push everything into a dict containing lists for each column
    for line in f:
        # initialise the headings on the first read
        if (headings == ""):
            headings = line.split()[1:]
            # put an empty list into the dict corresponding to each header
            for item in headings:
                data[item] = []
            continue

        spl = line.split()
        for i, val in enumerate(spl):
            # push each value into the list for the corresponding heading
            data[headings[i]].append(val)


    data['t_query'] = list(map(float, data['t_query']))
    data['t_hough'] = list(map(float, data['t_hough']))
    data['t_cluster'] = list(map(float, data['t_cluster']))
    data['cluster_n'] = list(map(int, data['cluster_n']))
    data['cluster_scores'] = list(map(strlist_to_int_list, data['cluster_scores']))
    data['cluster_points'] = list(map(strlist_to_int_list, data['cluster_points']))
    data['cluster_inobb'] = list(map(strlist_to_int_list, data['cluster_inobb']))
    data['n_hough_tot'] = list(map(int, data['n_hough_tot']))
    data['nonzero_hough'] = list(map(int, data['nonzero_hough']))
    data['hough_votes'] = list(map(int, data['hough_votes']))
    data['boxpts'] = list(map(int, data['boxpts']))
    data['boxvotes'] = list(map(int, data['boxvotes']))
    data['maxpts'] = list(map(int, data['maxpts']))
    data['maxvotes'] = list(map(int, data['maxvotes']))
    data['maxboxpts'] = list(map(int, data['maxboxpts']))
    data['maxboxvotes'] = list(map(int, data['maxboxvotes']))
    data['hough_hist'] = list(map(strlist_to_int_list, data['hough_hist']))
    data['box_hist'] = list(map(strlist_to_int_list, data['box_hist']))
    data['max_hist'] = list(map(strlist_to_int_list, data['max_hist']))
    data['boxmax_hist'] = list(map(strlist_to_int_list, data['boxmax_hist']))

#    print(columnise_sublists(data['cluster_scores'], True))
    
    # print(data['hough_hist'])
    # print(data['boxmax_hist'])
    # print(data['box_hist'])

#    print(get_mean_histogram(data['hough_hist']))
#    print(get_mean_histogram(data['box_hist']))
    houghmean, houghstd = get_mean_histogram(data['hough_hist'])

    results = {}
    results['hough_hist_mean'] = list(map(float, houghmean))
    results['hough_hist_std'] = list(map(float, houghstd))
    results['query_time_mean'] = statistics.mean(data['t_query'])
    results['query_time_std'] = statistics.stdev(data['t_query'])
    results['hough_time_mean'] = statistics.mean(data['t_hough'])
    results['hough_time_std'] = statistics.stdev(data['t_hough'])
    results['cluster_time_mean'] = statistics.mean(data['t_cluster'])
    results['cluster_time_std'] = statistics.stdev(data['t_cluster'])

    return results

def main():
    switch = sys.argv[1]
    args = map(os.path.abspath,sys.argv[2:])

    if (switch == "-s"): 
        get_data(args[0])
    elif (switch == "-m"):
        results = []
        for fname in args:
            results.append(get_data(fname))


        houghhist_mu = []
        houghhist_std = []
        for res in results:
            houghhist_mu.append(res['hough_hist_mean'])
            houghhist_std.append(res['hough_hist_std'])

        print(multiple_hist_to_gnuplot(houghhist_mu, houghhist_std))
    else:
        print("Must provide argument to define what action to perform.")
        return

if __name__ == '__main__':
    main()
