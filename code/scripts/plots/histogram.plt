# plot aggregated data for interest point computations - sift, susan, iss and
# uniform. Shows the computation time for the methods depending on the number of
# points to be computed. extract using featureprocess.py -ai
reset
#set output "$0.png"

#set terminal pngcairo 
#set ylabel 'Time (s)'
#set xlabel '# points'
#set key top left

set style line 12 lc rgb '#808080' lt 0 lw 1
set grid back ls 12

set style line 11 lc rgb '#808080' lt 1
set border 3 back ls 11
set tics nomirror

set key off
stats '$0' using 1
set xrange [STATS_min - 0.5:STATS_max + 0.5]
stats '$0' using 2
ymi=STATS_min
yma=STATS_max
stats '$0' using 3
stdma=STATS_max
set yrange [ymi + 0.01 : yma + stdma]
set xrange [-0.5:7]

set boxwidth 0.25
set style fill solid border -1
set xlabel "Vote value"
set ylabel "Mean vote count"
set logscale y

plot for [i=1:2] "$0" using i:i+1:i+2 with boxerrorbars