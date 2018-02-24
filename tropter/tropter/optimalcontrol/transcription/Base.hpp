#ifndef TROPTER_OPTIMALCONTROL_TRANSCRIPTION_BASE_HPP
#define TROPTER_OPTIMALCONTROL_TRANSCRIPTION_BASE_HPP
// ----------------------------------------------------------------------------
// tropter: Base.hpp
// ----------------------------------------------------------------------------
// Copyright (c) 2017 tropter authors
//
// Licensed under the Apache License, Version 2.0 (the "License"); you may
// not use this file except in compliance with the License. You may obtain a
// copy of the License at http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
// ----------------------------------------------------------------------------

#include "Base.h"
#include <tropter/optimalcontrol/DirectCollocationSolver.h>

namespace tropter {
namespace transcription {
template <typename T>
int Base<T>::get_current_iteration() const {
    return m_dcs_proxy.get_current_iteration();
}

} // namespace transcription
} // namespace tropter

#endif // TROPTER_OPTIMALCONTROL_TRANSCRIPTION_BASE_HPP
