
#include "DirectCollocation.hpp"

namespace mesh {

template class DirectCollocationSolver<adouble>;

namespace transcription {

template class LowOrder<adouble>;

} // namespace transcription
} // namespace mesh
