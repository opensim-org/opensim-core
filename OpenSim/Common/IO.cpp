/* -------------------------------------------------------------------------- *
 *                              OpenSim:  IO.cpp                              *
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
#include "IO.h"

#include "Logger.h"
#include <climits>
#include <math.h>
#include <string>
#include <time.h>
#if defined(__linux__) || defined(__APPLE__)
    #include <sys/stat.h>
    #include <sys/types.h>
#elif defined(_MSC_VER)
    #include <direct.h>
#else
    #include <unistd.h>
#endif

// PATH stuff from Kenny
#ifdef _MSC_VER
    #include <direct.h>
    #define PATH_MAX _MAX_PATH
#else
    #include <unistd.h>
#endif

// CONSTANTS


using namespace OpenSim;
using namespace std;

// STATICS
bool IO::_Scientific = false;
bool IO::_GFormatForDoubleOutput = false;
int IO::_Pad = 8;
int IO::_Precision = 8;
char IO::_DoubleFormat[] = "%16.8lf";
bool IO::_PrintOfflineDocuments = true;


//=============================================================================
// FILE NAME UTILITIES
//=============================================================================
//-----------------------------------------------------------------------------
// TIME STAMP
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Construct a date and time stamp with the format YYYYMMDD_HHMMSS.
 *
 * @return Null terminated character array of the stamp.  The caller must
 * delete this memory.
 */
char* IO::
ConstructDateAndTimeStamp()
{
    // GET DATE AND TIME
    time_t timeInSeconds;
    struct tm *timeStruct;
    time(&timeInSeconds);
    timeStruct = localtime(&timeInSeconds);

    // CONSTRUCT STAMP
    char *stamp = new char[64];
    sprintf(stamp,"%d%02d%02d_%02d%02d%02d",
        timeStruct->tm_year+1900,timeStruct->tm_mon+1,timeStruct->tm_mday,
        timeStruct->tm_hour,timeStruct->tm_min,timeStruct->tm_sec);

    return(stamp);
}
//-----------------------------------------------------------------------------
// FIXING SLASHES IN PATH
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Ensures slashes are back slashes ('\') in Windows and forward slashes ('/') in Linux.
 */
std::string IO::
FixSlashesInFilePath(const std::string &path)
{
    std::string fixedPath = path;
    for(unsigned int i=0;i<fixedPath.length();i++) {
#ifdef _WIN32
        if(fixedPath[i] == '/') fixedPath[i] = '\\';
#else
        if(fixedPath[i] == '\\') fixedPath[i] = '/';
#endif
    }
    return fixedPath;
}

//=============================================================================
// NUMBERED OUTPUT
//=============================================================================
//-----------------------------------------------------------------------------
// SCIENTIFIC
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Set whether or not output of numbers should be in scientific or float
 * format.
 *
 * @param aTrueFalse Scientific notation if true, and float if false.
 */
void IO::
SetScientific(bool aTrueFalse)
{
    _Scientific = aTrueFalse;
    ConstructDoubleOutputFormat();
}

//_____________________________________________________________________________
/**
 * Set whether or not output of numbers should be in scientific or float
 * format.
 *
 * @return True if scientific notation, false if float.
 */
bool IO::
GetScientific()
{
    return(_Scientific);
}

//-----------------------------------------------------------------------------
// %g formatting for doubles
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Set whether or not output of numbers should be printed using %g.
 */
void IO::
SetGFormatForDoubleOutput(bool aTrueFalse)
{
    _GFormatForDoubleOutput = aTrueFalse;
    ConstructDoubleOutputFormat();
}

//_____________________________________________________________________________
/**
 */
bool IO::
GetGFormatForDoubleOutput()
{
    return(_GFormatForDoubleOutput);
}

//-----------------------------------------------------------------------------
// PAD
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Set the number of digits with which to pad output of numbers.
 * The pad number is used to set the minimum field width by the following
 * formula:
 *      width = pad + precision
 *
 * The formated output string, for example, would have the form
 *      %width.precisionlf.
 *
 * @param aPad Number of digits with which output of numbers is padded.
 */
void IO::
SetDigitsPad(int aPad)
{
    if(aPad<0) aPad = -1;
    _Pad = aPad;
    ConstructDoubleOutputFormat();
}
//_____________________________________________________________________________
/**
 * Get the number of digits with which output of numbers is padded.
 * The pad number is used to set the minimum field width by the following
 * formula:
 *      width = pad + precision
 *
 * The formated output string, for example, would have the form
 *      %width.precisionlf.
 *
 * @return aPad Number of digits with which output of number is padded.
 */
int IO::
GetDigitsPad()
{
    return(_Pad);
}

//-----------------------------------------------------------------------------
// PRECISION
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Set the precision with which numbers are output.
 * The precision is usually simply the number of decimal places.
 *
 * @param aPrecision Precision.
 */
void IO::
SetPrecision(int aPrecision)
{
    if(aPrecision<0) aPrecision = 0;
    _Precision = aPrecision;
    ConstructDoubleOutputFormat();
}
//_____________________________________________________________________________
/**
 * Get the precision with which numbers are output.
 * The precision is usually simply the number of decimal places.
 *
 * @return Precision (often number of decimal places).
 */
int IO::
GetPrecision()
{
    return(_Precision);
}

//-----------------------------------------------------------------------------
// OUTPUT FORMAT
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Get the current output format for numbers of type double.
 *
 * The format is maintained as a static variable in class IO, so any
 * changes made to the output parameters in class IO will be seen globally
 * by all classes using this method.
 *
 * The returned output format will be of the form
 *
 *      "% width.precision lf" (if not scientific)  or
 *      "% width.precision le" (if scientific),
 *
 * where the spaces have been removed and width = pad + precision.
 *
 * @return Format string for output of doubles.
 * @see SetScientific(), SetDigitsPad, SetPrecision.
 */
const char* IO::
GetDoubleOutputFormat()
{
    return(_DoubleFormat);
}

//_____________________________________________________________________________
/**
 * Construct a valid output format for numbers of type double.
 * This method is called each time a change is made to any of the output
 * format parameters.
 *
 * @see SetScientific(), SetDigitsPad, SetPrecision.
 */
void IO::
ConstructDoubleOutputFormat()
{
    if(_GFormatForDoubleOutput) {
        sprintf(_DoubleFormat,"%%g");
    } else if(_Scientific) {
        if(_Pad<0) {
            sprintf(_DoubleFormat,"%%.%dle",_Precision);
        } else {
            sprintf(_DoubleFormat,"%%%d.%dle",_Pad+_Precision,_Precision);
        }
    } else {
        if(_Pad<0) {
            sprintf(_DoubleFormat,"%%.%dlf",_Precision);
        } else {
            sprintf(_DoubleFormat,"%%%d.%dlf",_Pad+_Precision,_Precision);
        }
    }
}

//=============================================================================
// Object printing
//=============================================================================
//_____________________________________________________________________________
/**
 * If Object::print is called and an xml node has a file attribute pointing
 * to an external file, then the _PrintOfflineDocuments determines whether
 * that file's contents will be printed to that file or not.
 * It's "offline" because it represents non-inlined xml code.
 */
void IO::
SetPrintOfflineDocuments(bool aTrueFalse)
{
    _PrintOfflineDocuments = aTrueFalse;
}
//_____________________________________________________________________________
/**
 */
bool IO::
GetPrintOfflineDocuments()
{
    return _PrintOfflineDocuments;
}
//=============================================================================
// READ
//=============================================================================
//_____________________________________________________________________________
/**
 * Read lines from the file until a line consisting entirely of a
 * token is read or until the end of the file.  The file pointer is
 * advanced to the first character after the token.
 *
 * Memory is allocated for the description, so the caller is responsible
 * for deleting the returned description.
 *
 * @param aIS input stream.
 * @param aToken Token to which to read.  The token should not contain
 * a carriage return.
 * @return String up to a token line.  The ending carriage
 * return is included in the returned string.
 */
string IO::
ReadToTokenLine(istream &aIS,const string &aToken)
{
    string text;
    while(aIS) {
        string line = IO::ReadLine(aIS);
        if(line == aToken) break;
        text+=line+"\n";
    }
    return text;
}

//_____________________________________________________________________________
/**
 * Read a line of text from file.
 *
 * The terminating carriage return is not included in the returned string.
 *
 * This handles both DOS- and Unix-style line endings.
 *
 * @param aIS Input stream.
 * @return Line of text not including the terminating carriage return.
 */
string IO::
ReadLine(istream &aIS)
{
    std::string line;
    getline(aIS, line);
    int len=(int)line.length();
    // deal with reading a DOS-format file in Linux
    if(len>0 && line[len-1]=='\r') line=line.substr(0,len-1);
    return line;
}
//_____________________________________________________________________________
/**
 * Compute the number of steps of a given size to get from some initial time to
 * some final time.
 *
 * If (aTF-aTI)/aDT has a remainder that is greater than 1% of the step size,
 * an additional step is assumed to be needed to step completely to the end
 * of the interval.
 *
 * @param aTI Initial time.
 * @param aTF Final time.
 * @param aDT Step size.  Should be greater than 0.0.
 */
int IO::
ComputeNumberOfSteps(double aTI,double aTF,double aDT)
{
    if(aDT<=0) return(0);

    double duration = aTF-aTI;
    int ns = (int)floor(duration/aDT);
    if((ns*aDT)<(duration-1.0e-2*aDT)) {
        ns += 2;
    } else {
        ns += 1;
    }

    return(ns);
}
//_____________________________________________________________________________
/**
 * Read a specified number of characters from file.
 *
 * @param aIS Input stream.
 * @param aNChar Number of characters in the description.
 * @return read string.
 */
string IO::
ReadCharacters(istream &aIS,int aNChar)
{
    char *buffer=new char[aNChar+1];
    aIS.read(buffer,aNChar);
    buffer[aIS.gcount()] = '\0';
    string str = buffer;
    delete[] buffer;
    return str;
}

bool IO::FileExists(const std::string& filePath) {
    return std::ifstream(filePath).good();
}

//_____________________________________________________________________________
/**
 * Open a file.
 */
FILE* IO::
OpenFile(const string &aFileName,const string &aMode)
{
    FILE *fp = NULL;

    // OPEN THE FILE
    fp = fopen(aFileName.c_str(),aMode.c_str());
    if(fp==NULL) {
        log_error("IO.OpenFile(const string&,const string&): "
                  "failed to open {}.", aFileName);
        return(NULL);
    }

    return(fp);
}
//_____________________________________________________________________________
/**
 * Open a file.
 */
ifstream *IO::
OpenInputFile(const string &aFileName,ios_base::openmode mode)
{
    ifstream *fs = new ifstream(aFileName.c_str(), ios_base::in | mode);
    if(!fs || !(*fs)) {
        log_error("IO.OpenInputFile(const string&,openmode mode): "
                  "failed to open {}.", aFileName);
        return(NULL);
    }

    return(fs);
}
ofstream *IO::
OpenOutputFile(const string &aFileName,ios_base::openmode mode)
{
    ofstream *fs = new ofstream(aFileName.c_str(), ios_base::out | mode);
    if(!fs || !(*fs)) {
        log_error("IO.OpenOutputFile(const string&,openmode mode): failed to "
                  "open {}.", aFileName);
        return(NULL);
    }

    return(fs);
}
//_____________________________________________________________________________
/**
 * Create a directory. Potentially platform dependent.
  * @return int 0 on success, EEXIST or other error condition
*/
int IO::
makeDir(const string &aDirName)
{

#if defined __linux__ || defined __APPLE__
    return mkdir(aDirName.c_str(),S_IRWXU);
#else
    return _mkdir(aDirName.c_str());
#endif
}
//_____________________________________________________________________________
/**
 * Change working directory. Potentially platform dependent.
  * @return int 0 on success, error condition otherwise
*/
int IO::
chDir(const string &aDirName)
{

#if defined __linux__ || defined __APPLE__
    return chdir(aDirName.c_str()); 
#else
    return _chdir(aDirName.c_str());
#endif

}
//_____________________________________________________________________________
/**
 * Get current working directory. Potentially platform dependent.
  * @return working directory on success, NULL on error
*/
string IO::
getCwd()
{
    char buffer[PATH_MAX];
#if defined __linux__ || defined __APPLE__
    auto ptr = getcwd(buffer, PATH_MAX); (void)ptr;
#else
    _getcwd(buffer, PATH_MAX);
#endif
    return string(buffer);
}

//_____________________________________________________________________________
/**
 * Get parent directory of the passed in fileName.
 * 
*/
string IO::
getParentDirectory(const string& fileName)
{
    string  result="";

    string::size_type dirSep = fileName.rfind('/'); // Unix/Mac dir separator
    
    if (dirSep == string::npos)
        dirSep = fileName.rfind('\\'); // DOS dir separator
    
    if (dirSep != string::npos) // if '_fileName' contains path information...
        result = fileName.substr(0,dirSep+1); // include trailing slashes

    return result;
}

//_____________________________________________________________________________
/**
 * Get filename part of a passed in URI (also works if a DOS/Unix path is passed in)
 * 
*/
string IO::
GetFileNameFromURI(const string& aURI)
{
    string  result=aURI;

    string::size_type dirSep = aURI.rfind('/'); // Unix/Mac dir separator
    
    if (dirSep == string::npos)
        dirSep = aURI.rfind('\\'); // DOS dir separator
    
    if (dirSep != string::npos) // if aURI contains path information...
        result = aURI.substr(dirSep+1);

    return result;
}

// TODO: account for '\n' inside comments
string IO::
formatText(const string& aComment,const string& leadingWhitespace,int width,const string& endlineTokenToInsert)
{
    string formatted;
    int count = 0;
    string::size_type i = 0;
    for(;;) {
        string::size_type pos = aComment.find_first_not_of(" \t\n", i);
        if(pos==string::npos) break;
        string whitespace = aComment.substr(i, pos-i);
        int newlineCount = 0;
        for(string::size_type j=0;j<whitespace.size();j++) {
            if (whitespace[j]=='\t') whitespace[j]=' ';
            else if (whitespace[j]=='\n') newlineCount++;
        }
        string::size_type pos2 = aComment.find_first_of(" \t\n", pos);
        string word;
        if(pos2==string::npos) word = aComment.substr(pos);
        else word = aComment.substr(pos, pos2-pos);
        i = pos2;
        if(!newlineCount && count+whitespace.size()+word.size()<=(string::size_type)width) {
            formatted += whitespace+word;
            count += (int)(whitespace.size()+word.size());
        } else {
            if(!formatted.empty()) {
                if(newlineCount) for(int j=0;j<newlineCount-1;j++) formatted += endlineTokenToInsert;
                formatted += endlineTokenToInsert + leadingWhitespace;
            }
            formatted += word;
            count = (int)word.size();
        }
    }
    return formatted;
}

string IO::
GetSuffix(const std::string &aStr, int aLen)
{
    const int sz = (int)aStr.size();
    return aStr.substr((sz>=aLen ? sz-aLen : 0));
}

void IO::
RemoveSuffix(std::string &rStr, int aLen)
{
    const int sz = (int)rStr.size();
    rStr.erase((sz>=aLen ? sz-aLen : 0));
}

string IO::
replaceSubstring(const std::string &aStr, const std::string &aFrom, const std::string &aTo)
{
    string result = aStr;
    for(string::size_type i = string::npos;;) {
        i = result.rfind(aFrom, i);
        if(i==string::npos) break;
        result.replace(i, aFrom.size(), aTo);
        if(i==0) break; else i--;
    }
    return result;
}

void IO::
TrimLeadingWhitespace(std::string &rStr)
{
    string::size_type front = rStr.find_first_not_of(" \t\r\n");
    rStr.erase(0, front);
}

void IO::
TrimTrailingWhitespace(std::string &rStr)
{
    string::size_type back = rStr.find_last_not_of(" \t\r\n");
    if(back < rStr.size()-1) rStr.erase(back+1);
}

std::string IO::
Lowercase(const std::string &aStr)
{
    std::string result = aStr;
    for(unsigned int i=0; i<aStr.size(); i++) result[i] = tolower(result[i]);
    return result;
}

std::string IO::
Uppercase(const std::string &aStr)
{
    std::string result = aStr;
    for(unsigned int i=0; i<aStr.size(); i++) result[i] = toupper(result[i]);
    return result;
}

bool IO::StartsWith(const std::string& string, const std::string& start) {
    // https://stackoverflow.com/questions/874134/find-if-string-ends-with-another-string-in-c
    if (string.length() >= start.length()) {
        return string.compare(0, start.length(), start) == 0;
    }
    return false;
}

bool IO::EndsWith(const std::string& string, const std::string& ending) {
    // https://stackoverflow.com/questions/874134/find-if-string-ends-with-another-string-in-c
    if (string.length() >= ending.length()) {
        return string.compare(string.length() - ending.length(),
                              ending.length(), ending) == 0;
    }
    return false;
}

bool IO::StartsWithIgnoringCase(
        const std::string& string, const std::string& start) {
    return StartsWith(IO::Lowercase(string), IO::Lowercase(start));
}

bool IO::EndsWithIgnoringCase(
        const std::string& string, const std::string& ending) {
    return EndsWith(IO::Lowercase(string), IO::Lowercase(ending));
}

void IO::eraseEmptyElements(std::vector<std::string>& list)
{
    std::vector<std::string>::iterator it = list.begin();
    while (it != list.end()) {
        if (it->empty())
            it = list.erase(it);
        else
            ++it;
    }
}

IO::CwdChanger::CwdChanger() {
}

IO::CwdChanger::CwdChanger(const std::string& newDir) :
    _existingDir{getCwd()} {

    chDir(newDir);
}

IO::CwdChanger IO::CwdChanger::noop() {
    return CwdChanger{};
}

IO::CwdChanger IO::CwdChanger::changeTo(const std::string& newDir) {
    return CwdChanger{newDir};
}

IO::CwdChanger IO::CwdChanger::changeToParentOf(const std::string& path) {
    return changeTo(getParentDirectory(path));
}

// a custom move constructor is necessary because the default move
// constructor, which effectively calls `this->_existingDir{std::move(tmp._existingDir)}`
// is specified to leave `tmp._existingDir` in an unspecified state:
//
// from: https://en.cppreference.com/w/cpp/string/basic_string/basic_string
//
// > Move constructor. Constructs the string with the contents of
// > other using move semantics. other is left in valid, but
// > unspecified state
//
// `~CwdChanger` requires that `tmp._existingDir.empty() == true`; otherwise,
// destruction of the temporary will cause a directory change.
IO::CwdChanger::CwdChanger(IO::CwdChanger&& tmp) :
    _existingDir{} {

    std::swap(this->_existingDir, tmp._existingDir);
}

IO::CwdChanger& IO::CwdChanger::operator=(CwdChanger&& tmp) {
    this->_existingDir.clear();
    std::swap(this->_existingDir, tmp._existingDir);
    return *this;
}

void IO::CwdChanger::restore() {
    chDir(_existingDir);
    _existingDir.clear();
}

void IO::CwdChanger::stay() noexcept {
    _existingDir.clear();
}

IO::CwdChanger::~CwdChanger() noexcept {
    if (!_existingDir.empty()) {
        chDir(_existingDir);
    }
}
