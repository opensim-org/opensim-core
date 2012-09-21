#ifndef __PathPointSet_h__
#define __PathPointSet_h__
/* -------------------------------------------------------------------------- *
 *                          OpenSim:  PathPointSet.h                          *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2012 Stanford University and the Authors                *
 * Author(s): Peter Loan                                                      *
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
#include <OpenSim/Common/Set.h>
#include "PathPoint.h"
#include "SimTKsimbody.h"

namespace OpenSim {

//=============================================================================
//=============================================================================
/**
 * A class for holding a set of muscle points.
 *
 * @authors Peter Loan
 * @version 1.0
 */

class OSIMSIMULATION_API PathPointSet :	public Set<PathPoint> {
OpenSim_DECLARE_CONCRETE_OBJECT(PathPointSet, Set<PathPoint>);

private:
	void setNull();
public:
	PathPointSet();
	PathPointSet(const PathPointSet& aPathPointSet);
	~PathPointSet(void);

	//--------------------------------------------------------------------------
	// OPERATORS
	//--------------------------------------------------------------------------
#ifndef SWIG
	PathPointSet& operator=(const PathPointSet &aPathPointSet);
#endif

//=============================================================================
};	// END of class PathPointSet
//=============================================================================
//=============================================================================

} // end of namespace OpenSim

#endif // __PathPointSet_h__
