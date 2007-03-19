// Exception.cpp
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
#include <iostream>
#include <string>
#include <cassert>
#include "osimCommon.h"
#include "Exception.h"
#include "IO.h"




using namespace OpenSim;
using namespace std;


//=============================================================================
// CONSTRUCTOR(S) AND DESTRUCTOR
//=============================================================================
//_____________________________________________________________________________
/**
 * Default constructor.
 *
 * @param aTrueFalse Scientific notation if true, and float if false.
 */
Exception::
Exception(const string &aMsg,const string &aFile,int aLine)
{
// make it assert false when debugging...
//	assert(false);
	setNull();

	setMessage(aMsg);
	_file = aFile;
	_line = aLine;
}

//-----------------------------------------------------------------------------
// NULL
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Set the member variables to NULL values.
 */
void Exception::
setNull()
{
	setMessage("");
	_line = -1;
}


//=============================================================================
// GET AND SET
//=============================================================================
//-----------------------------------------------------------------------------
// MESSAGE
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Set the exception message.
 *
 * @param aMsg Message.
 */
void Exception::
setMessage(const string &aMsg)
{
	_msg = aMsg;
}
//_____________________________________________________________________________
/**
 * Get the exception message.
 *
 * @return Message.
 */
const char* Exception::
getMessage()
{
	return(_msg.c_str());
}


//=============================================================================
// IO
//=============================================================================
//-----------------------------------------------------------------------------
// PRINT
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Print the exception to an output stream.
 *
 * @param aOut Output stream
 */
void Exception::
print(ostream &aOut)
{
	// HEADER
	aOut << "\nException:\n";

	// MESSAGE
	// Account for the _msg being multiple lines -- we want to prepend two spaces before each new line
	string formattedMsg = IO::formatText(_msg, "  ", 75);
	aOut << "  " << formattedMsg << endl;

	// FILE
	if(_file.size()>0) {
		aOut << "  file= " << _file << '\n';
	}

	// LINE
	if(_line>=0) {
		aOut << "  line= " << _line << '\n';
	}

	// RETURN
	aOut << '\n';
}

