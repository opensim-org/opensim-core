#ifndef OPENSIM_SOLVER_H_
#define OPENSIM_SOLVER_H_
/* -------------------------------------------------------------------------- *
 *                             OpenSim:  Solver.h                             *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2017 Stanford University and the Authors                *
 * Author(s): Ajay Seth                                                       *
 *                                                                            *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may    *
 * not use this file except in compliance with the License. You may obtain a  *
 * copy of the License at http://www.apache.org/licenses/LICENSE-2.0.         *
 *                                                                            *
 * Unless required by applicable law or agreed to in writing, software        *
 * distributed under the License is distributed on an "AS IS" BASIS,          *
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.   *
 * See the License for the specific language governing permissions and        *
 * limitations under the License.                                             *
 * -------------------------------------------------------------------------- */

#include <OpenSim/Simulation/osimSimulationDLL.h>
#include "OpenSim/Common/Object.h"
#include "SimTKcommon/internal/ReferencePtr.h"

namespace OpenSim {

class Model;

//=============================================================================
//=============================================================================
/**
 * The base (abstract) class for a family of objects responsible for solving
 * system equations (statics, dynamic, kinematics, muscle, etc...) given by a
 * model for values of interest.
 *
 * @author Ajay Seth
 * @version 1.0
 */
class OSIMSIMULATION_API Solver: public Object {
OpenSim_DECLARE_ABSTRACT_OBJECT(Solver, Object);

//=============================================================================
// METHODS
//=============================================================================
public:
    //-------------------------------------------------------------------------
    // CONSTRUCTION
    //-------------------------------------------------------------------------
    virtual ~Solver() {}
    explicit Solver(const Model &model) : _modelp(&model) {}

    // default copy constructor and copy assignment

    const Model& getModel() const {return *_modelp;}

protected:

//=============================================================================
// MEMBER VARIABLES
//=============================================================================
private:
    // The model handed to the solver to operate on; just a reference.
    SimTK::ReferencePtr<const Model> _modelp;

//=============================================================================
};  // END of class Solver
//=============================================================================
} // namespace

#endif // OPENSIM_SOLVER_H_
