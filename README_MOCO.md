OpenSim Moco
============

![Build Status with GitHub Actions][buildstatus_ghactions]

OpenSim Moco is a toolkit for solving optimal control problems involving
musculoskeletal systems using the direct collocation method. Moco solves the
following broad categories of problems:

1. Solve for the muscle activity that produces an observed motion.
1. Solve for the muscle activity that approximately tracks an observed motion.
2. Solve for a new motion that optimizes user-defined costs.
3. Solve for muscle properties that yield a good match between simulated and
   measured muscle activity.

Building Moco
=============

Moco depends on the following software:

1. **OpenSim**: Platform for simulating musculoskeletal systems.
   1. **Simbody**: Multibody dynamics.
2. **CasADi**: Algorithmic differentiation and interface to nonlinear solvers.
3. **Tropter**: C++ library for solving general optimal control problems with
direct collocation. Currently, Tropter's source code is part of Moco.
   1. **Eigen**: C++ matrix library.
   2. **ColPack**: Used to efficiently compute derivatives.
   3. **ADOL-C**: Automatic differentiation.
4. **Ipopt**: Nonlinear program solver.

Build the dependencies by building the CMake project in the `dependencies`
folder.


Windows
-------

On **Windows**, you can run the `build_on_windows.ps1` PowerShell script to
obtain Moco's dependencies and to build Moco. This script assumes you
have installed **Microsoft Visual Studio 2019** (with C++ support) and **CMake**
3.2 or greater. You can alternatively use **Microsoft Visual Studio 2015** or
**Microsoft Visual Studio 2017**.


Mac
---

Install the following:
- `gfortran`
- `pkgconfig`
- `autoreconf`
- `aclocal`
- `glibtoolize`
- `wget`
- `cmake`
- `doxygen` (optional)

You can install these with Homebrew:

```bash
brew install cmake pkgconfig gcc autoconf libtool automake wget doxygen
```

Nagivate to the directory where you placed the opensim-moco source code.

ex: `cd ~/opensim-moco`

Run build_on_mac from the terminal.

ex `./build_on_mac.sh`


Ubuntu
------

```bash
sudo apt install git wget build-essential libtool autoconf cmake pkg-config gfortran liblapack-dev
```

Use the CMake project in the
`dependencies` directory to install remaining dependencies.


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

   2. Choose constraints (activation within range of electromyography).

3. The software and its source code are made freely available in a way that
   allows for commercial use (permissive licensing).

4. Users do not need to manually specify derivatives (gradient, Jacobian,
   Hessian) for their optimal control problems.

5. For advanced users, there should be utilities to easily debug issues with
   problem formulation (which variables are hitting their constraints?) and to
   improve performance (visualize sparsity pattern).

6. The software should fully exploit all cores available on a user's
   computer, but should provide the option to only use 1 thread (if the user is
   solving multiple problems in parallel).

7. Users can construct a Moco problem in MATLAB and Python.

8. The software is easy to build from source.

9. The software runs on Windows, macOS, and Linux (Ubuntu).



[buildstatus_ghactions]: https://github.com/opensim-org/opensim-moco/workflows/continuous-integration/badge.svg
