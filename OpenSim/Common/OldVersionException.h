#ifndef _OldVersionException_h_
#define _OldVersionException_h_
/* -------------------------------------------------------------------------- *
 *                      OpenSim:  OldVersionException.h                       *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2012 Stanford University and the Authors                *
 * Author(s): Ayman Habib                                                     *
 *                                                                            *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may    *
 * not use this file except in compliance with the License. You may obtain a  *
 * copy of the License at http://www.apache.org/licenses/LICENSE-2.0.         *
 *                                                                            *
 * Unless required by applicable law or agreed to in writing, software        *
 * distributed under the License is distributed on an "AS IS" BASIS,          *
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied    *
 * See the License for the specific language governing permissions and        *
 * limitations under the License.                                             *
 * -------------------------------------------------------------------------- */


// INCLUDES
#include "Exception.h"


//=============================================================================
//=============================================================================
/**
 * A class for reporting issues with using models/files from older versions
 * that require user action.
 *
 * @version 1.0
 * @author Ayman Habib
 */
#ifdef SWIG
	#ifdef OSIMCOMMON_API
		#undef OSIMCOMMON_API
		#define OSIMCOMMON_API
	#endif
#endif

namespace OpenSim { 

class OSIMCOMMON_API OldVersionException : public Exception {

//=============================================================================
// DATA
//=============================================================================
protected:
//=============================================================================
// METHODS
//=============================================================================
public:
	// CONSTRUCTORS
	OldVersionException(const std::string &aMsg="", const std::string &aFile="",int aLine=-1):
	  Exception(aMsg, aFile, aLine)
	  {};

	
//=============================================================================
};	// END CLASS OldVersionException

}; //namespace
//=============================================================================
//=============================================================================

#endif // __OldVersionException_h__
