#ifndef CMCOrientation_h__
#define CMCOrientation_h__
/* -------------------------------------------------------------------------- *
 *                        OpenSim:  CMC_Orientation.h                         *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2012 Stanford University and the Authors                *
 * Author(s): Frank C. Anderson                                               *
 * Contributor(s): Frank C. Anderson                                          *
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
// This software, originally developed by Realistic Dynamics, Inc., was
// transferred to Stanford University on November 1, 2006.
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


//============================================================================
// INCLUDE
//============================================================================
#include "osimToolsDLL.h"
#include <OpenSim/Simulation/Model/Model.h>
#include "CMC_Task.h"

namespace OpenSim {

//=============================================================================
//=============================================================================
/**
 * A class for tracking the orientation of a body.  This class, although
 * roughed out, is not yet fully implemented.
 *
 * @author Frank C. Anderson
 * @version 1.0
 */
class OSIMTOOLS_API CMC_Orientation : public CMC_Task {
OpenSim_DECLARE_ABSTRACT_OBJECT(CMC_Orientation, CMC_Task);

//=============================================================================
// DATA
//=============================================================================
public:


protected:


//=============================================================================
// METHODS
//=============================================================================
	//--------------------------------------------------------------------------
	// CONSTRUCTION
	//--------------------------------------------------------------------------
public:
	CMC_Orientation();
	virtual ~CMC_Orientation();
	void setNull();

	//--------------------------------------------------------------------------
	// GET AND SET
	//--------------------------------------------------------------------------

	//--------------------------------------------------------------------------
	// COMPUTATIONS
	//--------------------------------------------------------------------------
	virtual void computeDesiredAccelerations();
	virtual void computeJacobian();
	virtual void computeEffectiveMassMatrix();


//=============================================================================
};	// END of class CMC_Orientation
//=============================================================================
//=============================================================================

}; // end namespace

#endif // CMCOrientation_h__


