#ifndef _InterruptedException_h_
#define _InterruptedException_h_
/* -------------------------------------------------------------------------- *
 *                      OpenSim:  InterruptedException.h                      *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2012 Stanford University and the Authors                *
 * Author(s): Eran Guendelman                                                 *
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


// INCLUDES
#include "Exception.h"

//=============================================================================
/**
 * Typically used by the GUI to stop long computations (e.g. IK, CMC, etc.)
 *
 * @version 1.0
 * @author Eran Guendelman
 */

namespace OpenSim { 

class InterruptedException : public Exception {

public:
	// CONSTRUCTORS
	InterruptedException(const std::string &aMsg="",const std::string &aFile="",int aLine=-1)
		: Exception(aMsg,aFile,aLine)
	{}

#ifndef SWIG
	virtual void print(std::ostream &aOut) {
		if(!_msg.empty()) aOut << "Operation interrupted: " << _msg << std::endl;
		else aOut << "Operation interrupted" << std::endl;
	}
#endif
//=============================================================================
};	// END CLASS InterruptedException

}; //namespace
//=============================================================================
//=============================================================================

#endif // __InterruptedException_h__
