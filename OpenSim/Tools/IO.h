#ifndef _IO_h_
#define _IO_h_
// IO.h
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


// DEFINES
const int IO_STRLEN = 2048;
const int IO_STRING_INCREMENT = 2048;


//=============================================================================
//=============================================================================
/**
 * A class for performing input and output.
 *
 * @version 1.0
 * @author Frank C. Anderson
 */
namespace OpenSim { 

class RDTOOLS_API IO {

//=============================================================================
// DATA
//=============================================================================
public:
	/** Default size of a statically or locally allocated character array. */
	static const int STRLEN;
	/** Increment by which the size of character strings are incremented. */
	static const int STRING_INCREMENT;

private:
	// NUMBER OUTPUT
	/** Specifies whether number output is in scientific or float format. */
	static bool _Scientific;
	/** Specifies number of digits of padding in number output. */ 
	static int _Pad;
	/** Specifies the precision of number output. */
	static int _Precision;
	/** The output format string. */
	static char _DoubleFormat[256];


//=============================================================================
// METHODS
//=============================================================================
public:
	// FILE NAMES
	static char* ConstructDateAndTimeStamp();
	static std::string FixSlashesInFilePath(const std::string &path);
	// NUMBER OUTPUT FORMAT
	static void SetScientific(bool aTrueFalse);
	static bool GetScientific();
	static void SetDigitsPad(int aPad);
	static int GetDigitsPad();
	static void SetPrecision(int aPlaces);
	static int GetPrecision();
	static const char*
		GetDoubleOutputFormat();
private:
	static void ConstructDoubleOutputFormat();

public:
	// READ
	static char* ReadToTokenLine(FILE *aFP,const char *aToken);
	static char* ReadLine(FILE *aFP);
	static int ComputeLineLength(FILE *aFP);
	static int ComputeNumberOfSteps(double aTI,double aTF,double aDT);
	static char* ReadCharacters(FILE *aFP,int aNChar);
	static FILE* OpenFile(const std::string &aFileName,const std::string &aMode);
	// Directory management
	static int makeDir(const std::string &aDirName);
	static int chDir(const std::string &aDirName);
	static std::string getCwd();
	static std::string getParentDirectory(const std::string& fileName);
//=============================================================================
};	// END CLASS IO

}; //namespace
//=============================================================================
//=============================================================================

#endif // __IO_h__
