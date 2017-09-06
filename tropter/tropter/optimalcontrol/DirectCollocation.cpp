
#include "DirectCollocation.hpp"

namespace tropter {

template class DirectCollocationSolver<double>;
template class DirectCollocationSolver<adouble>;

namespace transcription {

template class LowOrder<double>;
template class LowOrder<adouble>;

} // namespace transcription
} // namespace tropter
