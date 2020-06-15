Tropter
=======

Tropter is a C++ library for solving optimal control problems (trajectory
optimization) with direct collocation. 

Goals
=====

1. Detailed and helpful error messages and diagnostics (visualizing a 
sparsity pattern, inspecting constraint violations).

2. A simple and modern (C++11) interface for specifying the optimal control 
   problem.

3. Users do not need to supply derivative information (gradient, Jacobian, 
   Hessian) themselves.
   
4. Transcription via trapezoidal rule, Hermite-Simpson, and arbitrary order 
   Legendre polynomials.
   
5. Multiple phases.

6. Basic mesh refinement.
