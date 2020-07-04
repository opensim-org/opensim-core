
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
