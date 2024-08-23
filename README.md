# Persistent Team Patrolling Solver

## Solver Prerequisites

### Install Gurobi
Go to https://www.gurobi.com/academia/academic-program-and-licenses/, download the Linux version then get an academic license. Unpack the downloaded archive, start following the directions found in gurobi951/linux64/docs/quickstart_linux.pdf in the 'Software Installation Guide' section. This should get you started.

#### To Build Gurobi C++ Library
Follow these directions: https://stackoverflow.com/a/48867074

### Get LKH Solver
Go to http://webhotel4.ruc.dk/~keld/research/LKH-3/, download the latest version
Build the executable, then move it to 
`~/bin`

Add the following line to your .bashrc file:

`export PATH="/home/$USER/bin:$PATH"`

### Install VRP Solver
We are using the VRP solver found here: https://reinterpretcat.github.io/vrp/index.html. Follow the "Install from Cargo" steps found here: https://reinterpretcat.github.io/vrp/getting-started/installation.html#install-from-cargo

## To build and run:
To build the project, perform the following from the root directory

```
mkdir build
cd build
cmake ..
make
```

To run the solver, perform the following from the root directory

```
mkdir test/run
cd test/run
../../build/patrolling-solver ../Exp_01/plot_1_2_10_1.yaml 1
```

The solver will list input arguments by running

```
../../build/patrolling-solver
```
