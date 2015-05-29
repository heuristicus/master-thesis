import sys
import os

def get_interest_feature_types(pfname):
    try:
        pf = open(pfname, 'r')
    except IOError:
        print("Could not find parameter file corresponding to " + pfname)
        return "", ""

    full = ""
    for line in enumerate(pf):
        full += line[1][:-1]

    
    spaces = full.split('}') #split to namespaces
    feat = "" # want to get the feature extraction namespace
    for space in spaces:
        if (space.find("feature_extraction") != -1):
            feat = space

    if (feat == ""):
        print("Parameter file does not seem to match expected structure (does not contain feature_extraction namespace)")
        return "",""

    # feat now contains the feature namespace with information about descriptor
    # and interest point selection method
    params = feat.split('{')[1]
    plist = params.split(',')
    # go through the list of parameters and find the descriptor and interest point strings
    select = ""
    descriptor = ""
    for param in plist:
        if (param.find("feature_selection") != -1):
            select = param.split(':')[1][1:].lower()
        if (param.find("feature_type") != -1):
            descriptor = param.split(':')[1][1:].lower()

    pf.close()
    return select, descriptor

def combine_files(fname):
    data = []
    ind = fname.find('_') # find first underscore - splits date and featuredata string
    dirn = os.path.dirname(fname)
    date = os.path.splitext(fname[ind+1:])[0]
    paramfile = dirn + "/featureparams_" + date + ".yaml" # this is the filename of the corresponding param file
    
    print("Corresponding parameter file should be " + paramfile)
    select,descriptor = get_interest_feature_types(paramfile)

    if (select == "" or descriptor == ""):
        print("Didn't find feature selection or descriptor types")
        return

    # create a new filename for the combined data
    newname = dirn + "/featuredata_" + descriptor + "_" + select + "_" + date + ".txt"

    out = open(newname, 'w')

    pf = open(paramfile, 'r')
    pf.seek(0) # go back to the start of the file
    for line in pf:
        out.write(line)

    out.write("BEGIN_DATA\n")

    for i,line in enumerate(f):
        if (i == 0):
            out.write("#" + line.split('#')[1])
        else:
            out.write(line)
    pf.close()
    out.close()


# move the pointer to the line in the file to the first line whch contains the data
def go_to_data(datafile):
    for line in datafile:
        if (line.find("BEGIN_DATA") != -1):
            datafile.next() # advance to skip the headings
            return
            
def contains_interest_data(fname):
    f = open(fname, 'r')

    go_to_data(f)
    for line in f:
        if(line.split(' ')[3] != '0'):
            f.close()
            return True
            
    f.close()
    return False

def get_feature_data(fname):
    f = open(fname, 'r')
    go_to_data(f)

    fnames = []
    origpoints = []
    interestpoints = []
    interesttime = []
    featuretime = []
    for line in f:
        sp = line.split(' ')
        fnames.append(sp[0])
        origpoints.append(sp[1])
        interestpoints.append(sp[2])
        interesttime.append(sp[3])
        featuretime.append(sp[4])
        
    return zip(fnames, origpoints, interestpoints, interesttime, featuretime)

# get a list of files which contain the number of interest points and time taken
# to compute them - not all files have this due to bad coding
def get_interest_files(files):
    gotdata = []
    for fname in files:
        if (contains_interest_data(fname)):
            gotdata.append(fname)

    return gotdata

def fix_interest_files(args):
    # note that this assumes that there is only a single
    # file which has interest point data for each interest
    # point selection type
    idict = {}
    first = True
    interestfiles = get_interest_files(args)
    for intf in interestfiles:
        interest, feature = get_interest_feature_types(intf) # get the interest and feature type for this file
        data = get_feature_data(intf) # get the corresponding data out of the file
        # go through the data and populate the dictionary
        for line in data:
            # if this is the first item, then make each of the filename keys
            # in the dictionary dictionaries themselves
            if (first):
                idict[line[0]] = {} # each file has a corresponding dict

            # the files have different values for each line depending on
            # which interest type was used. each line contains the filename
            # and then timing and point data - put the timing and point data
            # into the sub-dict of the filename
            idict[line[0]][interest] = line[1:]
                
        first = False
        # also write these files with the same name so that we can grab files with a regex
        outfile = os.path.dirname(intf) + "/" + "fixed_" + os.path.basename(intf)
        out = open(outfile, 'w')
        with open(intf, 'r') as f:
            out.write(f.read())
        out.close()

    # idict is a dictionary whics has keys of filenames, which have values
    # that are dictionaries with keys of interest types, which have values
    # of a tuple containing data
    # loop over all the files in the list that don't have interest point values in them
    for fname in [x for x in args if not x in interestfiles]:
        f = open(fname, 'r')
        interest, feature = get_interest_feature_types(fname) # get the interest and feature type for this file
        outfile = os.path.dirname(fname) + "/" + "fixed_" + os.path.basename(fname)
        out = open(outfile, 'w')
        # put the parameter settings into the file
        for line in f:
            out.write(line)
            if (line[0] == '#'):
                break
            
        # read each remaining line of the file, put in the two missing values, by
        # extracting information from the dict, and then write it to the
        # fixed file.
        for line in f:
            sp = line.split(' ')
            # this should give the data tuple that we are looking for
            try:
                tp = idict[sp[0]][interest]
            except KeyError:
                print(fname + ": key error for " + interest + " in file " + sp[0])
                    
            # replace the number of interest points in the file
            out.write(sp[0] + " " + sp[1] + " " + tp[1] + " 0 " + sp[4])

        f.close()
        out.close()

# aggregate results from different runs            
def aggregate(outputdir, files, dofeature=True):
    fdict = {} # store open files here
    if (outputdir[-1] != '/'): # make sure the dir has a slash
        outputdir += '/'
    
    for fname in files:
        interest, feature = get_interest_feature_types(fname)
        # decide what to use as the key based on boolean parameter
        agg = feature if dofeature else interest
        if (not agg in fdict): # open a file inside the dictionary if one doesn't exist yet
            out = outputdir + agg + "_data_agg.txt"
            print(agg + " will be output to " + out)
            fdict[agg] = open(out, 'w')

        f = open(fname, 'r')
        go_to_data(f)
        for line in f:
            # split the line, remove the filename and then put things back
            # together and write them to the file
            fdict[agg].write(" ".join(line.split()[1:]) + "\n")

    # close all the aggregate files
    for key in fdict:
        fdict[key].close()

def merge_interest(outputdir, files):
    interest_files = get_interest_files(files)
    if (outputdir[-1] != '/'): # make sure the dir has a slash
        outputdir += '/'

    fdict = {}
    for fname in interest_files:
        interest, feature = get_interest_feature_types(fname)
        if (not interest in fdict):
            out = outputdir + interest + "_data_short.txt"
            fdict[interest] = open(out, 'w')

        f = open(fname, 'r')
        go_to_data(f)
        for line in f:
            # split the line, remove the filename and then put things back
            # together and write them to the file
            fdict[interest].write(" ".join(line.split()[1:]) + "\n")
            
# assumes that input is the data files, will parse parameter files
def main():
    switch = sys.argv[1]
    args = map(os.path.abspath,sys.argv[2:])

    combine = False
    check = False
    fixinterest = False
    aggfiles = False
    aggfeatures = False
    mergeint = False
    if (switch == "-c"):
        combine = True
    elif (switch == "-h"): # look for files which contain interest point timings
        check = True
    elif (switch == "-f"):
        fixinterest = True
    elif (switch == "-af"):
        # merge the data for files which use the same feature types
        aggfiles = True
        aggfeatures = True
    elif (switch == "-ai"):
        # merge the data for files which use the same interest point types
        aggfiles = True
        aggfeatures = False
    elif (switch == "-al"):
        # merge data for interest points of the same type, without repetition
        mergeint = True
    else:
        print("Must provide argument to define what action to perform.")
        return
        
    if (combine):
        for fname in args:
            print("Processing %s" % fname)
            combine_files(fname)
    elif (check):
        print("The following files have interest point data:")
        for f in get_interest_files(args):
            print(f)
    elif (fixinterest):
        fix_interest_files(args)
    elif (aggfiles):
        aggregate(args[0], args[1:], aggfeatures) # first argument is the output location
    elif (mergeint):
        merge_interest(args[0], args[1:]) # first argument is output location
        
if __name__ == '__main__':
    main()
