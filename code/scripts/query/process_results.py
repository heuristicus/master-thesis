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
        mu = statistics.mean(col)
        std = statistics.stdev(col) if len(col) > 1 else -1
        means.append("%.2f" % mu)
        stdevs.append("%.2f" % std)
        
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

# read each line of the cluster file into a list, and add it to a list. Sort
# this aggregate list to get top scoring clusters first.
def process_cluster_file(fname):
    f = open(fname, 'r')
    clusters = []
    for line in f:
        if (line[0] == '#'): # skip the first line
            continue
        
        spl = line.split()
        spl[0] = int(spl[0])
        clusters.append(spl)

    return sorted(clusters, key=lambda line: line[0], reverse=True)

# extract data from the file and put it into a dict
def get_data(fname):
    f = open(fname, 'r')
    go_to_data(f, False)
    headings = ""
    data = {}
    # information about the object and the interest point and descriptors used
    # to find them
    finfo=fname.split('/')[-3:-1]
    data['resfile'] = fname
    data['clusterfile'] = get_corresp_clusters(fname)
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
    
    return data

def process_data(data):
    houghmean, houghstd = get_mean_histogram(data['hough_hist'])
    print("Processing " + data['resfile'])
    results = {}
    results['resfile'] = data['resfile']
    results['clusterfile'] = data['clusterfile']
#    results['clusters'] = process_cluster_file(data['clusterfile'])
    results['nclusters_avg'] = statistics.mean(data['cluster_n'])
    results['nclusters_std'] = statistics.stdev(data['cluster_n'])
    results['total_clusters'] = sum(data['cluster_n'])
    results['interest'] = data['interest']
    results['feature'] = data['feature']
    results['objlabel'] = data['objlabel']
    results['hough_hist_mean'] = list(map(float, houghmean))
    results['hough_hist_std'] = list(map(float, houghstd))
    results['query_time_mean'] = statistics.mean(data['t_query'])
    results['query_time_std'] = statistics.stdev(data['t_query'])
    results['hough_time_mean'] = statistics.mean(data['t_hough'])
    results['hough_time_std'] = statistics.stdev(data['t_hough'])
    results['cluster_time_mean'] = statistics.mean(data['t_cluster'])
    results['cluster_time_std'] = statistics.stdev(data['t_cluster'])

    clusterpos = []
    totalmatches = 0 # total number of clusters in obb
    totaldistinct = 0 # total number of distinct clouds which have matches
    topmatches = 0
    matchingscores = []
    nonmatchingscores = []
    topnonmatchingscores = []
    topmatchingscores = []
    for ind,obblist in enumerate(data['cluster_inobb']):
        first = True
        scores = data['cluster_scores'][ind]
        for i,pos in enumerate(obblist):
            if (pos == 1):
                if (first):
                    # only consider the highest ranked cluster in the average
                    # position. divide the value in clusterpos by the total
                    # number of lists which have a match in them
                    first = False
                    clusterpos.append(i+1) # position of the matched cluster
                    totaldistinct += 1
                if (i == 0):
                    topmatchingscores.append(scores[i])
                    topmatches +=1
                if (i > 0):
                    matchingscores.append(scores[i])
                totalmatches += 1
            elif (pos == 0):
                if (i == 0):
                    topnonmatchingscores.append(scores[i])
                else:
                    nonmatchingscores.append(scores[i])

    results['top_matches'] = topmatches
    results['total_present'] = len(data['cluster_inobb']) - len(list(filter(lambda x: x[0] == -1, data['cluster_inobb'])))
    if (results['total_present'] > 89):
        print(results['total_present'])
        sys.exit(0)
    if (len(data['cluster_inobb']) < 80):
        print(len(data['cluster_inobb']))
        print("========== ERROR ==========")

    results['total_clouds'] = len(data['cluster_inobb'])
    results['cluster_pos_avg'] = statistics.mean(clusterpos) if len(clusterpos) != 0 else -1
    results['cluster_pos_std'] = statistics.stdev(clusterpos) if len(clusterpos) > 1 else -1
    results['matching_score_avg'] = statistics.mean(matchingscores) if len(matchingscores) != 0 else -1
    results['matching_score_std'] = statistics.stdev(matchingscores) if len(matchingscores) > 1 else -1
    results['top_matching_score_avg'] = statistics.mean(topmatchingscores) if len(topmatchingscores) != 0 else -1
    results['top_matching_score_std'] = statistics.stdev(topmatchingscores) if len(topmatchingscores) > 1 else -1
    results['nonmatching_score_avg'] = statistics.mean(nonmatchingscores) if len(nonmatchingscores) != 0 else -1
    results['nonmatching_score_std'] = statistics.stdev(nonmatchingscores) if len(nonmatchingscores) > 1 else -1
    results['top_nonmatching_score_avg'] = statistics.mean(topnonmatchingscores) if len(topnonmatchingscores) != 0 else -1
    results['top_nonmatching_score_std'] = statistics.stdev(topnonmatchingscores) if len(topnonmatchingscores) > 1 else -1
    results['total_cluster_matches'] = totalmatches
    results['total_distinct_matches'] = totaldistinct

    toppoints = []
    points = []
    for clist in data['cluster_points']:
        toppoints.append(clist[0])
        points += clist[1:]
        
    results['cluster_points_avg'] = statistics.mean(points) if len(points) != 0 else -1
    results['cluster_points_std'] = statistics.stdev(points) if len(points) > 1 else -1
    results['cluster_points_top_avg'] = statistics.mean(toppoints)
    results['cluster_points_top_std'] = statistics.stdev(toppoints)
    results['hough_votes'] = int(statistics.mean(data['hough_votes'])) # should always be the same
    results['hough_points_avg'] = statistics.mean(data['nonzero_hough'])
    results['hough_points_std'] = statistics.stdev(data['nonzero_hough'])
    results['region_points_avg'] = statistics.mean(data['boxpts'])
    results['region_points_std'] = statistics.stdev(data['boxpts'])
    results['region_votes_avg'] = statistics.mean(data['boxvotes'])
    results['region_votes_std'] = statistics.stdev(data['boxvotes'])
    results['region_max_points_avg'] = statistics.mean(data['maxboxpts'])
    results['region_max_points_std'] = statistics.stdev(data['maxboxpts'])
    results['region_max_votes_avg'] = statistics.mean(data['maxboxvotes'])
    results['region_max_votes_std'] = statistics.stdev(data['maxboxvotes'])

    return results

# check the key in this result and see if it has a value. If it does, add it to
# a string as usual, otherwise replace the value with something indicative
def check(result, key, end=False):
    if (result[key + "_avg"] != -1):
        
        return "%.1f" % result[key + "_avg"] +("$\pm$" + "%.1f" % result[key + "_std"] if result[key + "_std"] != -1 else "") + (" & " if not end else "")

    return "n/a" + (" & " if not end else "")
    
# restype should be time, cluster, votes, matches
def write_result(openfile, result, restype, multirowprefix='', prefix='', header=False, multirow=False):
    lines = ""
    if (restype == "time"):
        if (header):
            lines += ("& " if prefix else "") + "querytime & houghtime & clustertime \\\\\n"

        lines += "" if (prefix == '') else (prefix + " & ")
        lines += "%.4f" % result['query_time_mean'] + "$\pm$" + "%.4f" % result['query_time_std'] + " & "
        lines += "%.4f" % result['hough_time_mean'] + "$\pm$" + "%.4f" % result['hough_time_std'] + " & "
        lines += "%.4f" % result['cluster_time_mean'] + "$\pm$" + "%.4f" % result['cluster_time_std'] + "\\\\\n"
    elif (restype == "matches"):
        if (header):
            lines += (" & desc & " if prefix else "") + "top & total & uniq & actual & clusters & clouds & avg rank & top m score & m score & top nm score & nm score\\\\\hline\n"
        if (multirowprefix):
            lines += ("\\multirow{5}{*}{" + multirowprefix + "} & " if multirowprefix else "")

        if (multirow):
            lines += "&"
        lines += "" if (prefix == '') else (prefix + " & ")
        lines += str(result['top_matches']) + " & "
        lines += str(result['total_cluster_matches']) + " & "
        lines += str(result['total_distinct_matches']) + " & "
        lines += str(result['total_present']) + " & "
        lines += str(result['total_clusters']) + " & "
        lines += str(result['total_clouds']) + " & "
        lines += check(result, 'cluster_pos')
        lines += check(result, 'top_matching_score')
        lines += check(result, 'matching_score')
        lines += check(result, 'top_nonmatching_score')
        lines += check(result, 'nonmatching_score', True) + "\\\\\n"

        # lines += "%.1f" % result['cluster_pos_avg'] + "$\pm$" + "%.1f" % result['cluster_pos_std'] + " & "
        # lines += "%.1f" % result['top_matching_score_avg'] + "$\pm$" + "%.1f" % result['top_matching_score_std'] + " & "
        # lines += "%.1f" % result['matching_score_avg'] + "$\pm$" + "%.1f" % result['matching_score_std'] + " & "
        # lines += "%.1f" % result['top_nonmatching_score_avg'] + "$\pm$" + "%.1f" % result['top_nonmatching_score_std'] + " & "
        # lines += "%.1f" % result['nonmatching_score_avg'] + "$\pm$" + "%.1f" % result['nonmatching_score_std'] + "\\\\\n"

        
    openfile.write(lines)

# expect features to be a dict containing four interest type keys which have a
# result set attached
def write_subset(openfile, features, subsetname, typetowrite, header=True):
    multirowpref = subsetname.upper()
    multirow = False
    for interest in features:
        if (not features[interest]):
            continue
        write_result(openfile, features[interest], typetowrite, multirowpref, interest.upper(), header, multirow)

        header = False
        multirow = True
        multirowpref = ''

def write_aggregate_data(fname, data):
    # loop over objects
    objects = set()
    groupings = set()
    processed = {}
    for obj in sorted(data): # first level of keys is objects
        processed[obj] = {}
        objects.add(obj)
        for group in sorted(data[obj]): # second level is interest points or descriptors
            processed[obj][group] = {}
            groupings.add(group)
            for dkey in data[obj][group]: # third level is the data references
                # compute means and deviations for each data array and put into new dict
                processed[obj][group][dkey + "_mean"] = statistics.mean(data[obj][group][dkey]) if len(data[obj][group][dkey]) != 0 else -1
                processed[obj][group][dkey + "_stdd"] = statistics.stdev(data[obj][group][dkey]) if len(data[obj][group][dkey]) > 1 else -1

    lines = "\\begin{table}\n"
    lines += "\\begin{tabular}{cc|cccc}\n"
    lines += "Object & Type & Top & Total & Unique & Actual \\\\\\hline\n" # Clusters & Clouds
    for obj in sorted(processed):
        lines += "\\multirow{" + str(len(processed[obj])) + "}{*}{" + obj.replace('_', ' ') + "} & "
        for i,group in enumerate(sorted(processed[obj])):
            if (i != 0):
                lines += " & "
            lines += group.upper() + " & "
            plusminus = True # separate with plusminus or amp
            for j, item in enumerate(sorted(processed[obj][group])): # hacky sorting based on keys we hacked earlier
                lines += "%.1f" % processed[obj][group][item]
                if (plusminus):
                    lines += "$\pm$"
                    plusminus = not plusminus
                else:
                    if (j + 1 == len(processed[obj][group]) and i + 1 != len(processed[obj])):
                        lines += "\\\\\n"
                    elif (j + 1 != len(processed[obj][group])):
                        lines += " & "
                    plusminus = not plusminus

        lines += "\\\\\\hline\n"
        
    lines += "\\end{tabular}\n"
    lines += "\\end{table}\n"
    print(lines)
    f = open(fname,'w')
    f.write(lines)

def write_timing_data(fname, data):
    # first level of keys contains feature/interest point names
    f = open(fname, 'w')
    f.write("Feature & Query & Hough & Cluster\\\\\n")
    for feature in sorted(data):
        line = feature + " & "
        line += "%.5f" % statistics.mean(data[feature]['query']) + "$\pm$" + "%.5f" % statistics.stdev(data[feature]['query']) + " & "
        line += "%.5f" % statistics.mean(data[feature]['hough']) + "$\pm$" + "%.5f" % statistics.stdev(data[feature]['hough']) + " & "
        line += "%.5f" % statistics.mean(data[feature]['cluster']) + "$\pm$" + "%.5f" % statistics.stdev(data[feature]['cluster']) + "\\\\\n"
        f.write(line)
        
def output_processed_data(fdir, processed, interestset, skip):
    objects = set()
    features = set()
    interests = set()
    groupinterest = interestset
    for p in processed:
        objects.add(p['objlabel'])
        features.add(p['feature'])
        interests.add(p['interest'])

    # group all data associated with a single object
    objdict = {}
    for obj in objects:
        objdict[obj] = {} # each key of the dict contains another dict
        if (not groupinterest):
            for feature in features:
                # the sub-dicts are for files containing data corresponding to a
                # given feature for the current object. each is also a dict
                objdict[obj][feature] = {}
                for interest in interests:
                    matches = [x for x in processed if x['objlabel'] == obj and x['feature'] == feature and x['interest'] == interest]
                    objdict[obj][feature][interest] = matches[0] if len(matches) != 0 else {}
        else:
            for interest in interests:
                # the sub-dicts are for files containing data corresponding to a
                # given feature for the current object. each is also a dict
                objdict[obj][interest] = {}
                for feature in features:
                    matches = [x for x in processed if x['objlabel'] == obj and x['feature'] == feature and x['interest'] == interest]
                    objdict[obj][interest][feature] = matches[0] if len(matches) != 0 else {}

    # super aggregated data
    skippable = ["shot", "pfh", "fpfh", "sift", "susan"]
    f = open(fdir + "results_short" + ("_interest" if groupinterest else "_feature") + ("_skipped" if skip else "") + ".txt", 'w')
    f.write(" & ".join(sorted(objdict)) + "\\\\\n")
    aggregate = {} # aggregate all the data
    timings = {}
    timings_agg = {}
    timings_agg['query'] = []
    timings_agg['cluster'] = []
    timings_agg['hough'] = []
    first = True
    for i,obj in enumerate(sorted(objdict)):
        aggregate[obj] = {}
        matches = []
        count = 0
        thisobj = objdict[obj] # data for this object
        for it in sorted(thisobj):
            if (skip and it in skippable):
                continue
            aggregate[obj][it] = {}
            # hack to keep ordering in dict when sorted
            aggregate[obj][it]['1top'] = []
            aggregate[obj][it]['2total'] = []
            aggregate[obj][it]['3distinct'] = []
            aggregate[obj][it]['4actual'] = []
            #aggregate[obj][it]['5clusters'] = []
            #aggregate[obj][it]['6clouds'] = []
            sub = thisobj[it] # data for specific feature/interest type
            timings[it] = {}
            timings[it]['hough'] = []
            timings[it]['cluster'] = []
            timings[it]['query'] = []
            for ft in sub: # data for each feature/interest
                data = sub[ft]
                if (not data):
                    continue

                aggregate[obj][it]['1top'].append(data['top_matches'])
                aggregate[obj][it]['2total'].append(data['total_cluster_matches'])
                aggregate[obj][it]['3distinct'].append(data['total_distinct_matches'])
                aggregate[obj][it]['4actual'].append(data['total_present'])
                #aggregate[obj][it]['5clusters'] .append(data['total_clusters'])
                #aggregate[obj][it]['6clouds'].append(data['total_clouds'])

                matches.append(data['total_distinct_matches'])
                timings[it]['query'].append(data['query_time_mean'])
                timings[it]['cluster'].append(data['cluster_time_mean'])
                timings[it]['hough'].append(data['hough_time_mean'])
                timings_agg['query'].append(data['query_time_mean'])
                timings_agg['cluster'].append(data['cluster_time_mean'])
                timings_agg['hough'].append(data['hough_time_mean'])
                
            first = False
        
        f.write("%.1f" % statistics.mean(matches) + "$\pm$" + "%.1f" % statistics.stdev(matches) + (" & " if i + 1 < len(objdict) else "\\\\\n"))

    line = "Query & Hough & Cluster\\\\\n"
    line += "%.5f" % statistics.mean(timings_agg['query']) + "$\pm$" + "%.5f" % statistics.stdev(timings_agg['query']) + " & "
    line += "%.5f" % statistics.mean(timings_agg['hough']) + "$\pm$" + "%.5f" % statistics.stdev(timings_agg['hough']) + " & "
    line += "%.5f" % statistics.mean(timings_agg['cluster']) + "$\pm$" + "%.5f" % statistics.stdev(timings_agg['cluster']) + "\\\\\n"
    f.write(line)
    f.close()

    write_aggregate_data(fdir + "results_aggregate" + ("_interest" if groupinterest else "_feature") + ("_skipped" if skip else "") + ".txt", aggregate)
    write_timing_data(fdir + "results_timing"  + ("_interest" if groupinterest else "_feature") + ("_skipped" if skip else "") + ".txt", timings)

    typ = "matches" # or time
    for obj in objdict:
        f = open(fdir + obj + "_results" + ("_interest" if groupinterest else "_feature") + ("_time" if typ == "time" else "") + ".txt", 'w')

        # current object dict - full data output
        thisobj = objdict[obj]
        header = True
        f.write("\\begin{table}\n\\begin{tabular}{cc|ccccccccccc}\n")
        for i,ft in enumerate(sorted(objdict[obj])):
            thisfeature = thisobj[ft]
            write_subset(f, thisfeature, ft, typ, header)
            if (i + 1 == len(objdict[obj])):
                break
            f.write("\hline")
            header = False
        f.write("\\end{tabular}\n")
        f.write("\\caption{query results " + obj + "}\n")
        f.write("\\label{tab:q" + obj + "}\n")
        f.write("\\end{table}\n")
        f.close()
    
def process_multiple(outdir, args, interest, skip):
    # first, get the data out of the files and into a useful form
    data = []
    for fname in args:
        print(fname)
        data.append(get_data(fname))

    # process the data, extracting useful information from all of the results
    # lines
    processed = []
    for d in data:
        processed.append(process_data(d))

    output_processed_data(outdir + "/", processed, interest, skip)
        
def main():
    switch = sys.argv[1]
    args = list(map(os.path.abspath,sys.argv[2:]))

    if ("-s" in switch): 
        get_data(args[0])
    elif ("-m" in switch): # the first arg in the remaining args is the output location
        interest = False
        skip = False
        if ("i" in switch): # add i to the switch to aggregate data in terms of interest for the short data
            interest = True
        if ("k" in switch): # skip sift and susan, as well as shot, fpfh and pfh
            skip = True
        process_multiple(args[0], args[1:], interest, skip)
    else:
        print("Must provide argument to define what action to perform.")
        return

if __name__ == '__main__':
    main()
