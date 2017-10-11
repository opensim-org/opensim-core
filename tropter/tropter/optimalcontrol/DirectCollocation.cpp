
#include "DirectCollocation.hpp"

namespace tropter {

template class DirectCollocationSolver<double>;
template class DirectCollocationSolver<adouble>;

namespace transcription {

template class Trapezoidal<double>;
template class Trapezoidal<adouble>;

} // namespace transcription
} // namespace tropter
