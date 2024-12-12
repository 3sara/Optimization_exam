# Optimization_exam

This repository contains the project for the course of *"Optimization for Artificial Intelligence"*. 

The aim of the project is to solve an inverse kinematic problem: find the angles of rotation of a N-link robotic arm to reach a specified target in the space. 

The problem has been solved in a 3 dimensional space. (In a first implementation it has been solved in a 2 dimensional space to allow easyier computations, the code is available in the [`PSO_2dim.ipynb`] notebook for completeness).
The PSO implementation available in [`ParticleSwarmOptimization.py`] allow to deal with unconstrained and constraied problems:
- *Unconstrained*: the joints of the robotic arm are assumed to allow rotation around z and x axis without constraints on the range of angles.
- *Constrained*: the joints of the robotic arm are assumed to be revolute. User of the program has to provide the range of theta (rotation around z axis) for each arm, and the values od alpha angles (rotation around x) which are fixed and depend on the topology of the robotic arm. (It is also possible to specify only one between alpha_values and constraints_theta, the remaining variable will be considered as in the unconstrained case)


## Files in this repository:
- [`PSO_2d/`](PSO_2d.ipynb)  : first basical implementation in a 2 dimensional space. It isn't allowed to specify constraints.
- [`ParticleSwarmOptimization`](ParticleSwarmOptimization.py) : implementation of the PSO algorithm and useful function to allow computations.
- [`display`](display.py): implementation of method to do static and animated plot of robotic arms
- [`examples`](examples.ipynb) : notebook with 2 examples of robotic arms with and without constraints. Display of te problem and analysis of the hyperparameters.
- [`gif`](gif\) : a folder containing animated plot of the optimization of the robotic arm position.
- [`hyperpars_plt`](hyperpars_plt\) : folder containg plots to inspect number of iterations and fitness with different values of hyperparameters