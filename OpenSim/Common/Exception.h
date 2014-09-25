#ifndef OPENSIM_EXCEPTION_H_
#define OPENSIM_EXCEPTION_H_
/* -------------------------------------------------------------------------- *
 *                           OpenSim:  Exception.h                            *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2012 Stanford University and the Authors                *
 * Author(s): Frank C. Anderson                                               *
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

/* Note: This code was originally developed by Realistic Dynamics Inc. 
 * Author: Frank C. Anderson 
 */


// INCLUDES
#include "osimCommonDLL.h"
#include <string>

#ifdef WIN32
#pragma warning( disable : 4251 )	// VC2010 no-dll export of std::string
#endif

#ifdef SWIG
	#ifdef OSIMCOMMON_API
		#undef OSIMCOMMON_API
		#define OSIMCOMMON_API
	#endif
#endif

//=============================================================================
//=============================================================================

namespace OpenSim { 

/**
 * A class for basic exception functionality.
 *
 * @version 1.0
 * @author Frank C. Anderson
 */
class OSIMCOMMON_API Exception  : public std::exception {

//=============================================================================
// DATA
//=============================================================================
protected:
	/** A user set message for the exception. */
	std::string _msg;
	/** File in which the error occurred. */
	std::string _file;
	/** Line number at which the error occurred. */
	int _line;

//=============================================================================
// METHODS
//=============================================================================
public:
	// CONSTRUCTORS
	Exception(const std::string &aMsg="",const std::string &aFile="",int aLine=-1);
	virtual ~Exception() throw() {}
private:
	void setNull();

public:
	// SET AND GET
	void setMessage(const std::string &aMsg);
	const char* getMessage() const;

#ifndef SWIG
	// PRINT
	virtual void print(std::ostream &aOut) const;
#endif
    // override virtual function from std::exception
    const char* what() const throw() {return getMessage();}

//=============================================================================
};	// END CLASS Exception

}; //namespace
//=============================================================================
//=============================================================================

#endif // OPENSIM_EXCEPTION_H_
