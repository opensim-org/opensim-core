#include "OptimizationProblem.h"

namespace mesh {

// Explicit instantiation.
template class OptimizationProblem<adouble>;
// TODO extern to avoid implicit instantiation and improve compile time?

} // namespace mesh
