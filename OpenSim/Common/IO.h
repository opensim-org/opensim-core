#ifndef _IO_h_
#define _IO_h_
// IO.h
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
#include <fstream>

// DEFINES
const int IO_STRLEN = 2048;

//=============================================================================
//=============================================================================
/**
 * A class for performing input and output.
 *
 * @version 1.0
 * @author Frank C. Anderson
 */
namespace OpenSim { 

class OSIMCOMMON_API IO {

//=============================================================================
// DATA
//=============================================================================
private:
	// NUMBER OUTPUT
	/** Specifies whether number output is in scientific or float format. */
	static bool _Scientific;
	/** Specifies whether number output is in %g format or not. */
	static bool _GFormatForDoubleOutput;
	/** Specifies number of digits of padding in number output. */ 
	static int _Pad;
	/** Specifies the precision of number output. */
	static int _Precision;
	/** The output format string. */
	static char _DoubleFormat[256];
	/** Whether offline documents should also be printed when Object::print is called. */
	static bool _PrintOfflineDocuments;


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
	static void SetGFormatForDoubleOutput(bool aTrueFalse);
	static bool GetGFormatForDoubleOutput();
	static void SetDigitsPad(int aPad);
	static int GetDigitsPad();
	static void SetPrecision(int aPlaces);
	static int GetPrecision();
	static const char*
		GetDoubleOutputFormat();
private:
	static void ConstructDoubleOutputFormat();

public:
	// Object printing
	static void SetPrintOfflineDocuments(bool aTrueFalse);
	static bool GetPrintOfflineDocuments();
	// READ
	static std::string ReadToTokenLine(std::istream &aIS,const std::string &aToken);
	static std::string ReadLine(std::istream &aIS);
	static int ComputeNumberOfSteps(double aTI,double aTF,double aDT);
	static std::string ReadCharacters(std::istream &aIS,int aNChar);
	static FILE* OpenFile(const std::string &aFileName,const std::string &aMode);
	static std::ifstream* OpenInputFile(const std::string &aFileName,std::ios_base::openmode mode=std::ios_base::in);
	static std::ofstream* OpenOutputFile(const std::string &aFileName,std::ios_base::openmode mode=std::ios_base::out);
	// Directory management
	static int makeDir(const std::string &aDirName);
	static int chDir(const std::string &aDirName);
	static std::string getCwd();
	static std::string getParentDirectory(const std::string& fileName);
	static std::string GetFileNameFromURI(const std::string& aURI);
	static std::string formatText(const std::string& aComment,const std::string& leadingWhitespace,int width,const std::string& endlineTokenToInsert="\n");

	// String utilities
	static std::string GetSuffix(const std::string &aStr, int aLen);
	static void RemoveSuffix(std::string &rStr, int aLen);
	static std::string replaceSubstring(const std::string &aStr, const std::string &aFrom, const std::string &aTo);
	static void TrimLeadingWhitespace(std::string &rStr);
	static void TrimTrailingWhitespace(std::string &rStr);
	static void TrimWhitespace(std::string &rStr) { TrimLeadingWhitespace(rStr); TrimTrailingWhitespace(rStr); }
	static std::string Lowercase(const std::string &aStr);
	static std::string Uppercase(const std::string &aStr);
//=============================================================================
};	// END CLASS IO

}; //namespace
//=============================================================================
//=============================================================================

#endif // __IO_h__
