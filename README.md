Muscollo
========

Muscollo is a library for solving optimal control problems for 
musculoskeletal systems. The library is written in C++ and will be part of 
the OpenSim biomechanics simulation software package. Here are some examples of 
problems you will be able to solve with Muscollo:

1. Solve for the muscle activity that tracks a known motion.
2. Solve for a new motion that optimizes some objective functional.
3. TODO add more to this list.

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
      
   2. Advanced users can construct optimal control problems programmatically in
      C++.
   
3. Advanced users can create plugins to create custom cost terms and 
   constraints.
      
2. Allow biomechanists to customize an optimal control problem.

   1. Choose an objective functional (sum of squared muscle activation, 
      metabolic cost, joint loads).
      
3. The software and its source code are made freely available in a way that 
   allows for commercial use (permissive licensing).
   
4. Users do not need to manually specify derivatives (gradient, Jacobian, 
   Hessian) for their optimal control problems.
   
   1. Optimal control problems can be written using either `double`s 
      (derivatives computed with finite differences) or `adouble`s (allowing use
      of automatic differentiation).


Non-goals
=========

1.  We do not aim to provide a full-fledged direct collocation solver (in 
    Tropter). Rather, we will decide what features to add to Tropter based on 
    what is useful for solving biomechanics optimal control problems.


MATLAB, Python support?
