\ Objective function sense defines the sense of multiple objectives
maximize 0

subject to
\Capacity Constraint
84 x0 + 49 x1 + 68 x2 + 20 x3 + 97 x4 + 74 x5 + 60 x6 + 30 x7 + 13 x8 + 95 x9 <= 295

\ Multiple objectives are defined as the last constraints, with the RHS value of the last defining the number of objectives in total.
\ First objective is defined
 21 x0 +  69 x1 +  26 x2 +  92 x3 +  77 x4 +  30 x5 +  96 x6 +  80 x7 +  60 x8 +  61 x9 > 1

\ Second objective is defined
 52 x0 +  92 x1 +  19 x2 +  10 x3 +  63 x4 +  34 x5 + 100 x6 +  60 x7 +  11 x8 +  12 x9 > 2

\ Third objective is defined
 37 x0 + 100 x1 +  74 x2 +  17 x3 +  60 x4 +  69 x5 +  49 x6 +  69 x7 +  49 x8 +  59 x9 > 3

\ Forth objective is defined, RHS indicates the total number of objectives
 66 x0 +  92 x1 +  25 x2 +  84 x3 +  89 x4 +  21 x5 +  46 x6 +  94 x7 +  96 x8 +  92 x9 > 4

\ Integer constraints
integers
 x0  x1  x2  x3  x4  x5  x6  x7  x8  x9
end
