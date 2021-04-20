set terminal svg
set output "graphs/cv03_1.svg"
set title "priemerna priepustnost A2B"
set xlabel "Vzdialenost anten (m)"
set ylabel "Priepustnost (Mbit/s)"
plot "-"  title "WifiManager Arf " with linespoints
10 0
30 0
50 0
70 0
90 0
110 0
130 0
150 0
e
