set xrange[0:8]
set yrange[0:13]
set xlabel '実験回数(回)' font "VL-PGothic-Regular.ttf,18"
set ylabel '流量' font "VL-PGothic-Regular.ttf,18"
#set xlabel offset 0,0
#set ylabel offset -3,0
set xtics font 'Arial,14'
set ytics font 'Arial,14'

#set lmargin 13  #左余白
#set rmargin 11 #右余白
#set tmargin 11 #上余白
#set bmargin 4   #下余白
set title offset -1,0

set key left top

f(x) = x

set grid
set title "各実験における流量の変化"font "VL-PGothic-Regular.ttf,24"
set datafile separator ","
set terminal eps
set output "/home/yamada/m2/k交通流シンポジウム/pic/flowrate.eps"
plot "/home/yamada/m2/k交通流シンポジウム/pic/2dovr_f.csv" using 1:2 with lines lw 5 title "2dovr(8 robot)" ,\
      f(2.0) with lines lw 5 title "1 robot flow rate" ,\
     "/home/yamada/m2/k交通流シンポジウム/pic/ela_f.csv" using 1:2 with lines lw 5 title "elastic scattaring(8 robot)" 
set output

