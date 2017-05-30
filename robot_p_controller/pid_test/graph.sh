#!/bin/sh

create_graph() {
gnuplot << END
set grid
set xlabel 'time in 1/25 s'
set ylabel 'pid output in [-100;100] interval'
set terminal png size 1024,720
#set logscale x 2 
#set logscale y 2 
set output '$2.png'
set format x '%g'
set term png
set title "$1"
plot '${2}.csv' title "pid output" with lines , '${2}-s.csv' title "current speed" with lines 
END
}


cat "$1" | awk -v out1="${2}.csv" -v out2="${3}.csv" -v out3="${2}-s.csv" -v out4="${3}-s.csv" ' 
BEGIN{
	x=0
	x1=0
	x2=0
	print > out1
	print > out2
	print > out3
	print > out4
}

/^-?[0-9]/{
	print x " " $1 >> out1
	print x " " $2 >> out2
	x++;
}

/^SL/{
	print x1 " " $2 >> out3
	x1++;
}

/^SR/{
	print x2 " " $2 >> out4
	x2++;
}

END{

}'


create_graph $1 ${2}
create_graph $1 ${3}

