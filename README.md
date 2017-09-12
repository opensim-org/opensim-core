Muscollo
========

Muscollo is a library for solving optimal control problems for 
musculoskeletal systems. The library is written in C++ and will be part of 
the OpenSim biomechanics simulation software package. Here are some examples of 
problems you will be able to solve with Muscollo:

1. Solve for the muscle activity that tracks a known motion.
2. Solve for a new motion that optimizes some objective functional.
3. Solve for mass properties that minimize residual "hand of God" forces.

The optimal control problems are solved using the direct collocation method.

Building Muscollo
=================

Muscollo depends on the following software:

1. **OpenSim**: Platform for simulating musculoskeletal systems.
   1. **Simbody**: Multibody dynamics.
2. **Tropter**: C++ library for solving general optimal control problems with 
direct collocation. Currently, Tropter's source code is part of Muscollo.
   1. **Ipopt**: Nonlinear programming solver.
   2. **Eigen**: C++ matrix library.
   3. **ColPack**: Used to efficiently 
   4. **ADOL-C**: Automatic differentiation.

On **Linux** and **macOS**, you must obtain these packages on your own. Most of
them can be found in a package manager (`apt-get` on Ubuntu, Homebrew on macOS).

Ubuntu
------

```bash
sudo apt install git cmake pkg-config liblapack-dev coinor-libipopt-dev libadolc-dev
```

macOS
-----
If you are using Homebrew to obtain ColPack and ADOL-C, make 
sure they are compiled with the same compiler you will use for Muscollo and 
tropter (by default, Homebrew compiles these with GCC):

```bash
brew install --cc=clang colpack
brew install --cc=clang adol-c
```

On **Windows**, you can run the `build_on_windows.ps1` PowerShell script to 
obtain Muscollo's dependencies and to build Muscollo. This script assumes you
have installed **Microsoft Visual Studio 2015** (with C++ support) and **CMake**
3.2 or greater.

Design goals
============

1. Allow biomechanists to solve certain classes of optimal control problems with
   ease and without writing any code.

   1. Solving for muscle activity from a known motion should be faster than 
      using OpenSim Computed Muscle Control.
      
   2. Users should be able to solve for mass properties that minimize 
      residual forces.
      
   3. Advanced users can construct optimal control problems programmatically in
      C++.
   
3. Advanced users can create plugins to create custom cost terms and 
   constraints.
      
2. Allow biomechanists to customize an optimal control problem.

   1. Choose an objective functional (sum of squared muscle activation, 
      metabolic cost, joint loads, coordinate tracking, marker tracking).
      
   2. Choose constraints (activation within range of EMG).
      
3. The software and its source code are made freely available in a way that 
   allows for commercial use (permissive licensing).
   
4. Users do not need to manually specify derivatives (gradient, Jacobian, 
   Hessian) for their optimal control problems.
   
   1. Optimal control problems can be written using either `double`s 
      (derivatives computed with finite differences) or `adouble`s (allowing use
      of automatic differentiation).

5. For advanced users, there should be utilities to easily debug issues with 
   problem formulation (which variables are hitting their constraints?) and to 
   improve performance (visualize sparsity pattern).
   
6. The software should fully exploit all cores available on a user's 
   computer, but should provide the option to only use 1 thread (if the user is 
   solving multiple problems in parallel).
   
7. Users can construct a Muscollo problem in MATLAB and Python.

8. The software is easy to build from source.

9. The software runs on Windows, macOS, and Linux (Ubuntu).

Non-goals
=========

1. We do not aim to provide a full-fledged direct collocation solver (in 
   Tropter). Rather, we will decide what features to add to Tropter based on 
   what is useful for solving biomechanics optimal control problems.
    
2. Specifying custom cost or constraint modules in MATLAB/Python.

User stories
============

1. A user has joint angles from OpenSim and estimates muscle activity
   that minimizes the sum of squared muscle activity.
 
2. A user plots the solution to an optimal control problem in Python or MATLAB.
   
3. A user postprocesses their optimal control solution in OpenSim to compute 
   metabolic cost across the trajectory.

