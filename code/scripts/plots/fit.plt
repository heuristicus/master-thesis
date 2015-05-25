reset

set terminal wxt

# Line styles
set border linewidth 2
set style line 1 linecolor rgb '#dd181f' linetype 1 linewidth 2  # red
set style line 2 linecolor rgb '#0060ad' linetype 1 linewidth 2  # blue
set style line 3 linecolor rgb '#34D800' linetype 1 linewidth 2 # green

# Axis labels
#set ylabel '$$\lambda$$'

# Axis ranges
#set yrange [-5:50]

# Tics in latex format
#set format '$$%g$$'

# Tics
#set xtics 20

#set key bmargin horizontal



# plot "$1" index 0 using 1:2 with lines linestyle 1 title 'Generating function',\
#      "$2" using 1:2 with lines linestyle 2 title 'Estimated function',\
#      "$3" using 1:2 with points linestyle 3 title 'Bin counts'