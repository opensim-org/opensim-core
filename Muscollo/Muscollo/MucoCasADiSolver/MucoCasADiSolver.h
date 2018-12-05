#ifndef MUSCOLLO_MUCOCASADISOLVER_H
#define MUSCOLLO_MUCOCASADISOLVER_H
/* -------------------------------------------------------------------------- *
 * OpenSim Muscollo: MucoCasADiSolver.h                                       *
 * -------------------------------------------------------------------------- *
 * Copyright (c) 2017 Stanford University and the Authors                     *
 *                                                                            *
 * Author(s): Christopher Dembia                                              *
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

#include "../osimMuscolloDLL.h"

#include "../MucoSolver.h"

namespace OpenSim {

// TODO have a different class for each transcription scheme.
// TODO need functions to convert between data types
class OSIMMUSCOLLO_API MucoCasADiSolver : public MucoSolver {
    OpenSim_DECLARE_CONCRETE_OBJECT(MucoCasADiSolver, MucoSolver);
public:
    OpenSim_DECLARE_PROPERTY(num_mesh_points, int,
    "The number of mesh points for discretizing the problem (default: 100).");
    MucoCasADiSolver();
private:
    void constructProperties();
    void resetProblemImpl(const MucoProblemRep&) const override {}
    MucoSolution solveImpl() const override;
};

} // namespace OpenSim


#endif // MUSCOLLO_MUCOCASADISOLVER_H
