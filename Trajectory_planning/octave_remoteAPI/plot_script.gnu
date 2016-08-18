set term wxt 1
set xtic auto
set ytic auto
splot "myfile.txt" u 1:2:3 with lines
pause -1 "Hit any key to continue"
