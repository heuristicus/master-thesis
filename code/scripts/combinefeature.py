import sys
import os

def combineFiles(fname):
    f = open(fname, 'r')
    data = []
    ind = fname.find('_') # find first underscore - splits date and featuredata string
    dirn = os.path.dirname(fname)
    date = os.path.splitext(fname[ind+1:])[0]
    paramfile = "/featureparams_" + date + ".yaml" # this is the filename of the corresponding param file
    
    try:
        print("Corresponding parameter file should be " + dirn + paramfile)
        pf = open(dirn + paramfile, 'r')
    except IOError:
        print("Could not find parameter file corresponding to " + fname)
        return {}

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
        return {}

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

    if (select == "" or descriptor == ""):
        print("Didn't find feature selection or descriptor types")
        return {}

    # create a new filename for the combined data
    newname = dirn + "/featuredata_" + descriptor + "_" + select + "_" + date + ".txt"

    out = open(newname, 'w')

    pf.seek(0) # go back to the start of the file
    for line in pf:
        out.write(line)

    out.write("BEGIN_DATA\n")

    for i,line in enumerate(f):
        if (i == 0):
            out.write("#" + line.split('#')[1])
        else:
            out.write(line)
    
# assumes that input is the data files, will parse parameter files
def main():
    args = sys.argv[1:]
    results = []
    # go through all the filenames and extract results from each one
    for fname in args:
        print("Processing %s" % os.path.abspath(fname))
        results.append(combineFiles(os.path.abspath(fname)))
    print(results)

if __name__ == '__main__':
    main()
