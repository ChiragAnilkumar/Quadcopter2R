# Dynamics and Control of Quadcopter with 2R Manipulator

The code files attached contains the main file Quadplot.m which is used to plot and animate the quadcopter system with 2R manipulator. The code file eom_quad.m contains the set of differential equations which are solved using ode45 function.

The initial conditions and desired conditions for each case of simulation can be considered by uncommenting the required lines of code in each file. Also, the PD control gains are provided for each case separately in both files. The main file contains code respectively for plotting the generalized system of coordinates as well as for simulation of quadcopter system.

The EOM_Derive_syms.m file is used to derive the Coriolis matrix as well the Gravity matrix from inertia matrix in the equations of motion using syms function

