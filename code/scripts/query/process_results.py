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

# get the cluster file corresponding to the 
def get_corresp_clusters(fname):
    return os.path.splitext(fname)[0] + "_clusters.txt"

def strlist_to_int_list(strlist):
    return list(map(int,strlist.split(',')))

def max_sublist_len(lst):
    return max(list(map(len, lst)))

def columnise_sublists(lst):
    maxind = max_sublist_len(lst)
    cols=[[] for x in range(0, maxind)]
    for subl in lst:
        for i,val in enumerate(subl):
            cols[i].append(subl[i])

    return cols
    
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

    print(data['cluster_scores'])
    print(max_sublist_len(data['cluster_scores']))
    print(columnise_sublists(data['cluster_scores']))

    results = {}
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
        print(results)
            
    else:
        print("Must provide argument to define what action to perform.")
        return

if __name__ == '__main__':
    main()
