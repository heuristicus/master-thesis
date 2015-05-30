# plot aggregated data for interest point computations - sift, susan, iss and
# uniform. Shows the computation time for the methods depending on the number of
# points to be computed. extract using featureprocess.py -al
reset
set output "$0.png"

set terminal pngcairo 
#set logscale x
set ylabel 'Time (s)'
set xlabel '# points'
#set xrange [100:40000]
#set xtics (0,100,500,1000,5000,10000,40000)
set key top right

set style line 12 lc rgb '#808080' lt 0 lw 1
set grid back ls 12

set style line 11 lc rgb '#808080' lt 1
set border 3 back ls 11
set tics nomirror


plot "$1" using 2:3 with points title 'SIFT',\
     "$2" using 2:3 with points title 'SUSAN',\
     "$3" using 2:3 with points title 'ISS',\
     "$4" using 2:3 with points title 'uniform'
     