\ Objective function sense defines the sense of multiple objectives
Minimize 0
s.t.

\ Row assignment constraints
X1X1 + X1X2 + X1X3 + X1X4 + X1X5 = 1
X2X1 + X2X2 + X2X3 + X2X4 + X2X5 = 1
X3X1 + X3X2 + X3X3 + X3X4 + X3X5 = 1
X4X1 + X4X2 + X4X3 + X4X4 + X4X5 = 1
X5X1 + X5X2 + X5X3 + X5X4 + X5X5 = 1

\ Column assingment constraints
X1X1 + X2X1 + X3X1 + X4X1 + X5X1 = 1
X1X2 + X2X2 + X3X2 + X4X2 + X5X2 = 1
X1X3 + X2X3 + X3X3 + X4X3 + X5X3 = 1
X1X4 + X2X4 + X3X4 + X4X4 + X5X4 = 1
X1X5 + X2X5 + X3X5 + X4X5 + X5X5 = 1

\ Multiple objectives are defined as the last constraints, with the RHS value of the last defining the number of objectives in total.
\ First objective is defined
19 X1X1 + 13 X1X2 + 13 X1X3 +  3 X1X4 +  3 X1X5 +
 2 X2X1 +  3 X2X2 + 15 X2X3 + 15 X2X4 +  0 X2X5 +
15 X3X1 + 11 X3X2 + 14 X3X3 +  4 X3X4 +  5 X3X5 +
 8 X4X1 + 15 X4X2 +  9 X4X3 +  1 X4X4 +  2 X4X5 + 
 9 X5X1 +  3 X5X2 + 10 X5X3 + 12 X5X4 +  9 X5X5 < 1

\ Second objective is defined
 5 X1X1 + 14 X1X2 +  2 X1X3 + 15 X1X4 +  3 X1X5 +
14 X2X1 +  7 X2X2 +  8 X2X3 +  7 X2X4 + 10 X2X5 +
11 X3X1 +  1 X3X2 +  5 X3X3 +  7 X3X4 + 16 X3X5 +
17 X4X1 + 14 X4X2 + 19 X4X3 + 12 X4X4 + 10 X4X5 +
 4 X5X1 + 12 X5X2 +  6 X5X3 +  5 X5X4 + 13 X5X5 < 2

\ Third objective is defined, RHS indicates the total number of objectives
 8 X1X1 + 14 X1X2 +  8 X1X3 + 10 X1X4 +  7 X1X5 +
10 X2X1 +  7 X2X2 + 13 X2X3 +  4 X2X4 +  3 X2X5 +
17 X3X1 + 18 X3X2 + 10 X3X3 +  5 X3X4 + 17 X3X5 +
12 X4X1 +  9 X4X2 + 18 X4X3 +  9 X4X4 +  8 X4X5 +
 6 X5X1 +  7 X5X2 + 14 X5X3 +  5 X5X4 + 11 X5X5 < 3

\ Forth objective is defined, RHS indicates the total number of objectives
14 X1X1 + 18 X1X2 + 11 X1X3 +  7 X1X4 +  1 X1X5 +
11 X2X1 + 14 X2X2 + 14 X2X3 +  0 X2X4 +  3 X2X5 +
10 X3X1 +  2 X3X2 + 12 X3X3 +  7 X3X4 +  4 X3X5 +
 6 X4X1 +  0 X4X2 + 19 X4X3 + 18 X4X4 + 17 X4X5 +
18 X5X1 +  2 X5X2 + 18 X5X3 +  3 X5X4 +  3 X5X5 < 4

\ Binary constraints
BINARY
X1X1
X1X2
X1X3
X1X4
X1X5
X2X1
X2X2
X2X3
X2X4
X2X5
X3X1
X3X2
X3X3
X3X4
X3X5
X4X1
X4X2
X4X3
X4X4
X4X5
X5X1
X5X2
X5X3
X5X4
X5X5
END
