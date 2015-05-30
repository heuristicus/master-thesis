# plot data for interest point computations. Shows the number of interest points
# extracted vs size of the cloud from which they are to be extracted. Extract
# data using featureprocess.py -al

reset
set output "$0.png"

set terminal pngcairo
set ylabel 'Extracted interest points'
set xlabel 'Points in cloud'
set key top left
set format x "%.0t*10^%T"
set xrange [300000:800000]
set xtics (300000,400000,500000,600000,700000,800000)
set style line 12 lc rgb '#808080' lt 0 lw 1
set grid back ls 12

set style line 11 lc rgb '#808080' lt 1
set border 3 back ls 11
set tics nomirror

plot "$1" using 1:2 with points title 'SIFT',\
     "$2" using 1:2 with points title 'SUSAN',\
     "$3" using 1:2 with points title 'ISS',\
     "$4" using 1:2 with points title 'uniform'
     