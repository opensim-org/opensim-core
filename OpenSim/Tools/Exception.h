#ifndef _Exception_h_
#define _Exception_h_
// Exception.h
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
/*
* Copyright (c) 2005, Stanford University. All rights reserved. 
* Redistribution and use in source and binary forms, with or without 
* modification, are permitted provided that the following conditions
* are met: 
*  - Redistributions of source code must retain the above copyright 
*    notice, this list of conditions and the following disclaimer. 
*  - Redistributions in binary form must reproduce the above copyright 
*    notice, this list of conditions and the following disclaimer in the 
*    documentation and/or other materials provided with the distribution. 
*  - Neither the name of the Stanford University nor the names of its 
*    contributors may be used to endorse or promote products derived 
*    from this software without specific prior written permission. 
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
* "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
* LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS 
* FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE 
* COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
* INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, 
* BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; 
* LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER 
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
* LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN 
* ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE 
* POSSIBILITY OF SUCH DAMAGE. 
*/

/* Note: This code was originally developed by Realistic Dynamics Inc. 
 * Author: Frank C. Anderson 
 */


// INCLUDES
#include "rdTools.h"
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
	#ifdef RDTOOLS_API
		#undef RDTOOLS_API
		#define RDTOOLS_API
	#endif
#endif

namespace OpenSim { 

class RDTOOLS_API Exception {

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
	Exception(const std::string &aMsg=NULL,const std::string &aFile="",int aLine=-1);
private:
	void setNull();

public:
	// SET AND GET
	void setMessage(const std::string &aMsg);
	const char* getMessage();

	// PRINT
	void print(std::ostream &aOut);

//=============================================================================
};	// END CLASS Exception

}; //namespace
//=============================================================================
//=============================================================================

#endif // __Exception_h__
