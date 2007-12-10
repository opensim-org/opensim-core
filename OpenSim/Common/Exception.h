#ifndef _Exception_h_
#define _Exception_h_
// Exception.h
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
/*
* Copyright (c)  2005, Stanford University. All rights reserved. 
* Use of the OpenSim software in source form is permitted provided that the following
* conditions are met:
* 	1. The software is used only for non-commercial research and education. It may not
*     be used in relation to any commercial activity.
* 	2. The software is not distributed or redistributed.  Software distribution is allowed 
*     only through https://simtk.org/home/opensim.
* 	3. Use of the OpenSim software or derivatives must be acknowledged in all publications,
*      presentations, or documents describing work in which OpenSim or derivatives are used.
* 	4. Credits to developers may not be removed from executables
*     created from modifications of the source.
* 	5. Modifications of source code must retain the above copyright notice, this list of
*     conditions and the following disclaimer. 
* 
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY
*  EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
*  OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT
*  SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
*  TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; 
*  HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
*  OR BUSINESS INTERRUPTION) OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY
*  WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

/* Note: This code was originally developed by Realistic Dynamics Inc. 
 * Author: Frank C. Anderson 
 */


// INCLUDES
#include "osimCommonDLL.h"
#include <string>

//using namespace std;


//=============================================================================
//=============================================================================
/**
 * A class for basic exception functionality.
 *
 * @version 1.0
 * @author Frank C. Anderson
 */
#ifdef SWIG
	#ifdef OSIMCOMMON_API
		#undef OSIMCOMMON_API
		#define OSIMCOMMON_API
	#endif
#endif

namespace OpenSim { 

class OSIMCOMMON_API Exception {

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
	virtual ~Exception() {}
private:
	void setNull();

public:
	// SET AND GET
	void setMessage(const std::string &aMsg);
	const char* getMessage();

	// PRINT
	virtual void print(std::ostream &aOut);

//=============================================================================
};	// END CLASS Exception

}; //namespace
//=============================================================================
//=============================================================================

#endif // __Exception_h__
