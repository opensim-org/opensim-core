#ifndef OPENSIM_MOCOCONTACTINITIALIZER_H
#define OPENSIM_MOCOCONTACTINITIALIZER_H
/* -------------------------------------------------------------------------- *
 * OpenSim: MocoContactInitializer.h                                          *
 * -------------------------------------------------------------------------- *
 * Copyright (c) 2024 Stanford University and the Authors                     *
 *                                                                            *
 * Author(s): Nicholas Bianco                                                 *
 *                                                                            *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may    *
 * not use this file except in compliance with the License. You may obtain a  *
 * copy of the License at http://www.apache.org/licenses/LICENSE-2.0          *
 *                                                                            *
 * Unless required by applicable law or agreed to in writing, software        *
 * distributed under the License is distributed on an "AS IS" BASIS,          *
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.   *
 * See the License for the specific language governing permissions and        *
 * limitations under the License.                                             *
 * -------------------------------------------------------------------------- */

#include "osimMocoDLL.h"
#include <OpenSim/Common/Object.h>

namespace OpenSim {

class OSIMMOCO_API MocoContactSolver : public Object {
    OpenSim_DECLARE_CONCRETE_OBJECT(MocoContactSolver, Object);

};

} // namespace OpenSim

#endif // OPENSIM_MOCOCONTACTINITIALIZER_H
