#ifndef _IO_h_
#define _IO_h_
/* -------------------------------------------------------------------------- *
 *                               OpenSim:  IO.h                               *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2017 Stanford University and the Authors                *
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
#include <fstream>
#include <iostream>
#include <vector>

// DEFINES
const int IO_STRLEN = 2048;


namespace OpenSim { 
//=============================================================================
//=============================================================================
/**
 * A class for performing input and output with OpenSim API.
 *
 * @version 1.0
 * @author Frank C. Anderson
 */
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
#ifndef SWIG
    static std::string ReadToTokenLine(std::istream &aIS,const std::string &aToken);
    static std::string ReadLine(std::istream &aIS);
    static int ComputeNumberOfSteps(double aTI,double aTF,double aDT);
    static std::string ReadCharacters(std::istream &aIS,int aNChar);
    static bool FileExists(const std::string& filePath);
    static FILE* OpenFile(const std::string &aFileName,const std::string &aMode);
    static std::ifstream* OpenInputFile(const std::string &aFileName,std::ios_base::openmode mode=std::ios_base::in);
    static std::ofstream* OpenOutputFile(const std::string &aFileName,std::ios_base::openmode mode=std::ios_base::out);
#endif
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
    static void eraseEmptyElements(std::vector<std::string>& list);
//=============================================================================
};  // END CLASS IO

/// @name Filling in a string with variables.
/// @{

#ifndef SWIG
/// Return type for make_printable()
template <typename T>
struct make_printable_return {
    typedef T type;
};
/// Convert to types that can be printed with sprintf() (vsnprintf()).
/// The generic template does not alter the type.
template <typename T>
inline typename make_printable_return<T>::type make_printable(const T& x) {
    return x;
}

/// Specialization for std::string.
template <>
struct make_printable_return<std::string> {
    typedef const char* type;
};
/// Specialization for std::string.
template <>
inline typename make_printable_return<std::string>::type make_printable(
        const std::string& x) {
    return x.c_str();
}

/// Format a char array using (C interface; mainly for internal use).
OSIMCOMMON_API std::string format_c(const char*, ...);

/// Format a string in the style of sprintf. For example, the code
/// `format("%s %d and %d yields %d", "adding", 2, 2, 4)` will produce
/// "adding 2 and 2 yields 4".
template <typename... Types>
std::string format(const std::string& formatString, Types... args) {
    return format_c(formatString.c_str(), make_printable(args)...);
}

/// Print a formatted string to std::cout. A newline is not included, but the
/// stream is flushed.
template <typename... Types>
void printMessage(const std::string& formatString, Types... args) {
    std::cout << format(formatString, args...);
    std::cout.flush();
}

#endif // SWIG

/// @}

}; //namespace
//=============================================================================
//=============================================================================

#endif // __IO_h__
