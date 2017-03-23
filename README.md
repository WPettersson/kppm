# README #

This is a multi-objective integer programming (MOIP) algorithm called k-PPM, based off the following paper:

C. Dhaenens, , J. Lemesre, E.G. Talbi (2008). K-PPM: A new exact method to solve multi-objective combinatorial optimization problems. European Journal of Operational Research, 200(1), 45-53.

### What is this repository for? ###

This software can be used to optimise various multi-objective integer programming problems. Currently, work is not yet stabilised and no guarantees can be offered on the correctness of various versions of the implementation. Official releases will hopefully be coming shortly.

### How do I get set up? ###

The implementation works on Linux operating systems, and requires IBM ILOG CPLEX 12.6.3 (or possibly greater) and Boost. It currently uses a rudimentary Makefile for configuration, a better configuration system is planned.

It uses an extended LP file format where multiple objectives are defined as additional constraints after the original problem's constraints. The right-hand-side value of the last constraint defines the number of objectives. Example LP files are provided under a  separate folder.

### Who do I talk to? ###

Dr William Pettersson (william@ewpettersson.se) is the lead developer of this particular implementation of this algorithm. For more details on the original algorithm, you may also wish to contact the authors of the above paper.
