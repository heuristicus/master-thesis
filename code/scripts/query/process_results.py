#!/usr/bin/python
import sys
import os

# move the pointer in an open file to the first line which contains data
# if skip is false, do not skip the next line
def go_to_data(openfile, skip=True):
    for line in openfile:
        if (line.find("BEGIN_DATA") != -1) :
            if (skip):
                openfile.next()
            return
            
# move the pointer in an open file to the first line which contains a top cluster
# if skip is false, then do not skip the next line
def go_to_files(openfile, skip=True):
    for line in openfile:
        if (line.find("BEGIN_FILES") != -1):
            if (skip):
                openfile.next()
            return

def show_single(fname):
    f = open(fname, 'r')
    go_to_data(f, False)
    headings = ""
    data = {}
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
        print(spl)
        for i, val in enumerate(spl):
            # push each value into the list for the corresponding heading
            data[headings[i]].append(val)
        
        if (line.find("BEGIN_FILES") != -1):
            break

    for key in data:
        print(key, data[key][0])

def main():
    switch = sys.argv[1]
    args = map(os.path.abspath,sys.argv[2:])

    if (switch == "-s"):
        show_single(args[0])
    else:
        print("Must provide argument to define what action to perform.")
        return



if __name__ == '__main__':
    main()
