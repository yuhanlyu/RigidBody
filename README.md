
This package contains analytical solvers for Differential Drive, Reeds-Shepp car, and Dubins Car.
I optimized the code by using the symetry optimal trajectories and the synthesis of optimal trajectories.

It is possible to further speed-up the computation by precompute optimal trajectory types for a uniform grid with fine resolution so that the program does not need to compute several trajectories to find the optimal trajectory.
However, this method makes solves not analytical and hence I did not use this method.
