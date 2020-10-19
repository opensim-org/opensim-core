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
    /// Determine if `string` starts with the substring `start`.
    static bool StartsWith(const std::string& string, const std::string& start);
    /// Determine if `string` ends with the substring `ending`.
    static bool EndsWith(const std::string& string, const std::string& ending);
    /// Same as StartsWith() except both arguments are first converted to
    /// lowercase before performing the check.
    static bool StartsWithIgnoringCase(
            const std::string& string, const std::string& start);
    /// Same as EndsWith() except both arguments are first converted to
    /// lowercase before performing the check.
    static bool EndsWithIgnoringCase(
            const std::string& string, const std::string& ending);
    static void eraseEmptyElements(std::vector<std::string>& list);

    /**
     * A class that:
     *
     * - On construction: switches the calling process's current working
     *   directory to the supplied directory
     *
     * - On destruction: switches the calling process's working directory
     *   back to its original directory.
     */
    class OSIMCOMMON_API CwdChanger final {
        std::string _existingDir;

        /**
         * Constructs a CwdChanger that does nothing.
         */
        CwdChanger();

        /**
         * Constructs a CwdChanger that, during construction, switches the
         * calling process's working directory to `newDir`.
         */
        CwdChanger(const std::string& newDir);

    public:
        /**
         * Returns a CwdChanger that does nothing.
         *
         * This is useful for conditional directory changing:
         *
         *     CwdChanger c = shouldSwitch ? CwdChanger::changeTo(p) : CwdChanger::noop();
         */
        static CwdChanger noop();

        /**
         * Returns a CwdChanger that changes to `newDir`.
         */
        static CwdChanger changeTo(const std::string& newDir);

        /**
         * Returns a CwdChanger that changes to the parent of `path`.
         *
         * This is useful when changing into a file's parent dir:
         *
         *     CwdChanger::changeToParentOf(xmlFile);
         */
        static CwdChanger changeToParentOf(const std::string& path);

        CwdChanger(const CwdChanger&) = delete;
        CwdChanger(CwdChanger&& tmp);
        CwdChanger& operator=(const CwdChanger&) = delete;
        CwdChanger& operator=(CwdChanger&&);

        /**
         * Prematurely change the current working directory back to its
         * original location.
         *
         * This is functionally equivalent to prematurely destructing the
         * CwdChanger. After calling CwdChanger::restore(), the now-restored
         * CwdChanger will become a noop instance that, when it destructs,
         * will not attempt to change back to the original directory.
         */
        void restore();

        /**
         * Release CwdChanger's control over the current working directory,
         * such that the CwdChanger instance does not attempt to change back
         * to its original directory on destruction.
         */
        void stay() noexcept;

        /**
         * Destructs a CwdChanger instance.
         *
         * The destructor switches the calling process's current working
         * directory back to whatever it was before constructing the instance,
         * unless:
         *
         * - The instance was constructed with `noop()`: in this case, it does
         *   nothing
         *
         * - The instance is an rvalue temporary: does nothing
         */
        ~CwdChanger() noexcept;
    };
//=============================================================================
};  // END CLASS IO

}; //namespace
//=============================================================================
//=============================================================================

#endif // __IO_h__
