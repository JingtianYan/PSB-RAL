# Multi-Agent Motion Planning with Bézier Curve Optimization under Kinodynamic Constraints

## Note
This repository contains codes for
*Yan, J. and Li, J., 2024. Multi-Agent Motion Planning With Bézier Curve Optimization Under Kinodynamic Constraints. IEEE Robotics and Automation Letters.*
The repository is developed and tested in Ubuntu 20.04.

## General structure
The project consists two folders, namely `Intersection` and `gridmap`. The `Intersection` folders contains the implementation of PSB for traffic intersection coordination problem. The `gridmap` folder contains the implementation of PSB for obstacle rich grid map navigation domain.
In each method folder `src` `include` and `data` folders are included.

## Requirements
The code makes use of the external library [boost](https://www.boost.org/) and [Cplex](https://www.ibm.com/products/ilog-cplex-optimization-studio/cplex-optimizer).

If you are using Ubuntu, you can install the boost by
```shell script
sudo apt install libboost-all-dev
``` 

If the above method does not work, you can also follow the instructions
on the [boost](https://www.boost.org/) website and install it manually.

The required external library Cplex is from IBM. You can follow this guide [link](https://www.ibm.com/docs/en/icos/20.1.0?topic=cplex-installing) from IBM to install the package.




## Overall Usage
To compile each Algorithm, please go into the directory of the source code and compile it with CMake. 
Then, you can run the algorithm with designated map file and scenario file.

For example, to run the PSB in the Intersection domain
```shell script
cd Intersection/PSB-intersection
cmake .
make
./AIMPSB -k 10 -m ./data/intro_graph_3.json -a ./data/500vphpl/vehicleArrival_500vphpl_random1.json -c vehicle500vphpl_random1.csv
```

To run the PSB in the grid map domain
```shell script
cd gridmap/PSB-MAPF
cmake .
make
./MAPFPSB -k 10 -m ./data/Boston_0_256/Boston_0_256.map -a ./data/Boston_0_256/scen-random/Boston_0_256-random-1.scen -c Boston_0_256-random-1-output.csv
```

- m: the map file from the MAPF benchmark
- a: the scenario file from the MAPF benchmark
- c: the output file that contains the search statistics
- k: the number of agents

## Acknowledgement

[1] Levin, M. W., and Rey, D. 2017. Conflict-point formulation of intersection control for autonomous vehicles. Transportation Research Part C: Emerging Technologies 85:528–547.

[2] Li, J.; Lin, E.; Vu, H. L.; Koenig, S.; et al. 2023. Intersection coordination with priority-based search for autonomous vehicles. In Proceedings of the AAAI Conference on Artificial Intelligence, volume 37, 11578–11585.

[3] Ali, Z. A., and Yakovlev, K. 2023. Safe interval path planning with kinodynamic constraints. In Proceedings of the AAAI Conference on Artificial Intelligence, volume 37, 12330–12337.