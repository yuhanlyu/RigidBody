
This package contains analytical solvers for Differential Drive, Reeds-Shepp car, and Dubins Car.
I optimized the code by using the symetry optimal trajectories and the synthesis of optimal trajectories.
You can find example usage in the Test.cpp in each folder.

It is possible to further speed-up the computation by precompute optimal trajectory types for a uniform grid with fine resolution. Then, the program can look up just one trajectory type and compute the corresponding trajectory other than compute several trajectories and choose the best one. However, this method makes solvers not analytical and hence I did not use this method.
