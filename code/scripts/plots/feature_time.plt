# plot aggregated data for feature point computations. Shows the computation
# time for the methods depending on the number of points to be computed. extract
# using featureprocess.py -af
reset
set output "$0.png"

set terminal pngcairo 
set ylabel 'Time (s)'
set xlabel '# interest points'
set key top left
set xrange [1:35000]
set style line 12 lc rgb '#808080' lt 0 lw 1
set grid back ls 12

set style line 11 lc rgb '#808080' lt 1
set border 3 back ls 11
set tics nomirror

plot "$1" using 2:4 with points title 'SHOT',\
     "$2" using 2:4 with points title 'SHOTCOLOR',\
     "$3" using 2:4 with points title 'PFH',\
     "$4" using 2:4 with points title 'FPFH',\
     "$5" using 2:4 with points title 'PFHRGB'

     