
#include "DirectCollocation.hpp"

namespace tropter {

template class DirectCollocationSolver<adouble>;

namespace transcription {

template class LowOrder<adouble>;

} // namespace transcription
} // namespace tropter
