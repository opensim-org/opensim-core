/* -------------------------------------------------------------------------- *
 *                            OpenSim:  SimmIO.cpp                            *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2017 Stanford University and the Authors                *
 * Author(s): Peter Loan                                                      *
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

//=============================================================================
// INCLUDES
//=============================================================================
#include <time.h>
#include "SimmIO.h"
#include "SimTKcommon/Scalar.h"

//=============================================================================
// STATICS
//=============================================================================
using namespace std;
using namespace OpenSim;

//=============================================================================
// UTILITY FUNCTIONS
//=============================================================================
//_____________________________________________________________________________
/**
 * Read a string from a stream, skipping over any commented text.
 *
 * @param aStream stream to read from.
 * @param rBuffer string from stream is returned here.
 * @return True if string was read, false if not.
 */
bool OpenSim::readNonCommentStringFromStream(istream &aStream, string &rBuffer)
{
   size_t start, end;

    rBuffer = "\0";
    while (1)
    {
        if (rBuffer.empty())
            if (!readStringFromStream(aStream, rBuffer))
                return false;

        start = rBuffer.find("/*");
        if (start != rBuffer.npos)
        {
            while (1)
            {
                end = rBuffer.find("*/");
                if (end == rBuffer.npos) // if end was not found
                    readStringFromStream(aStream, rBuffer);
                else
                {
                    rBuffer.erase(start, end+2);
                    break;
                }
            }
        }
        else
            break;
    }
    return true;
}

//_____________________________________________________________________________
/**
 * Read a string from a stream.
 *
 * @param aStream stream to read from.
 * @param rBuffer string from stream is returned here.
 * @return True if string was read, false if not.
 */
bool OpenSim::readStringFromStream(istream &aStream, string &rBuffer)
{
   while (1)
   {
      aStream >> rBuffer;
      if (aStream.eof())
         return false;
      if (!rBuffer.empty())
         return true;
   }
}

//_____________________________________________________________________________
/**
 * Read a string from an input string. The input string is modified by
 * deleting the string from it after it is read.
 *
 * @param aString input string to read from.
 * @param rBuffer string that is read is returned here.
 * @return True if string was read, false if not.
 */
bool OpenSim::readStringFromString(string &aString, string &rBuffer)
{
   int i;

   if (aString.empty())
      return false;

   /* remove any initial whitespace */
   i = findFirstNonWhiteSpace(aString);
   if (i > 0)
      aString.erase(0, i);

   if (aString.empty())
      return false;
   /* find first whitespace */
   i = findFirstWhiteSpace(aString);

   /* only one string found */
   if (i == -1)
   {
      i = (int)aString.length();
      rBuffer = aString;
      aString.erase(0, i);
   }
   else if (i >= 0)
   {
      rBuffer.assign(aString, 0, i);
      aString.erase(0, i + 1); //remove string and whitespace
   }
   /* remove any whitespace after the string*/
   i = findFirstNonWhiteSpace(aString);
   if (i > 0)
      aString.erase(0, i);
   else if (i != 0)
      aString.erase(0, aString.length());

   if (rBuffer.empty())
      return false;
   return true;
}

//_____________________________________________________________________________
/**
 * Read a tab-delimited string from an input string. The input string is
 * modified by deleting the string from it after it is read. This function
 * strips all leading white space from the input string, then looks for a
 * tab, line feed, or carriage return to terminate the string.
 *
 * @param aString input string to read from.
 * @param rBuffer string that is read is returned here.
 * @return True if string was read, false if not.
 */
bool OpenSim::readTabDelimitedStringFromString(string &aString, string &rBuffer)
{
   int i;

   if (aString.empty())
      return false;

   /* remove any initial whitespace */
   i = findFirstNonWhiteSpace(aString);
   if (i > 0)
      aString.erase(0, i);

   if (aString.empty())
      return false;

   /* find first tab, line feed, or carriage return */
   i = (int)aString.find_first_of("\t\r\n", 0);

   /* no tab, whole input string is one string */
   if (i == -1)
   {
      i = (int)aString.length();
      rBuffer = aString;
      aString.erase(0, i);
   }
   else if (i >= 0)
   {
      rBuffer.assign(aString, 0, i);
      aString.erase(0, i + 1); //remove string and whitespace
   }
   /* remove any whitespace after the string */
   i = findFirstNonWhiteSpace(aString);
   if (i > 0)
      aString.erase(0, i);
   else if (i != 0)
      aString.erase(0, aString.length());

   if (rBuffer.empty())
      return false;

   return true;
}

//_____________________________________________________________________________
/**
 * Read an integer from an input string. The input string is
 * modified by deleting the integer from it after it is read.
 *
 * @param aString input string to read from.
 * @param rNumber integer that is read is returned here.
 * @return True if integer was read, false if not.
 */
bool OpenSim::readIntegerFromString(string &aString, int *rNumber)
{
   size_t i, end;
   string buffer;

   if (aString.empty())
      return false;

   /* remove any characters before the number */
   i = aString.find_first_of("0123456789-", 0);
   if (i != 0)
      aString.erase(0, i);

   /* remove number from string, copy number to buffer */
   i = aString.find_first_not_of("0123456789-eE", 0);
   end = aString.length();
   if (i != aString.npos)
   {
      buffer.assign(aString, 0, i);
      aString.erase(0, i);
   }
   else
   {
      buffer.assign(aString);
      aString.erase(0, end);
   }

   /* remove any whitespace after the string*/
   i = findFirstNonWhiteSpace(aString);
   if (i > 0)
      aString.erase(0, i);
   
   if (buffer.empty())
      return false;
   *rNumber = atoi(buffer.c_str());
   return true;
}


//_____________________________________________________________________________
/**
 * Read a double from an input string. The input string is
 * modified by deleting the double from it after it is read.
 *
 * @param aString input string to read from.
 * @param rNumber double that is read is returned here.
 * @return True if double was read, false if not.
 */
bool OpenSim::readDoubleFromString(string &aString, double *rNumber, bool allowNaNs)
{
   size_t i, end;
   string buffer;

   if (aString.empty())
      return false;
   // Remove leading spaces
    while(aString[0]==' ') aString.erase(0, 1);
   /* remove any characters before the number */
   i = aString.find_first_of("0123456789-.", 0);
   if (i != 0){
       if (allowNaNs){
           std::string NaNString = "NAN";
           std::string prefix = aString.substr(0, 3);
           std::transform(prefix.begin(), prefix.end(),prefix.begin(), ::toupper);
           if (prefix==NaNString){
                aString.erase(0, 3);
                *rNumber = SimTK::NaN;
                return true;
           }
       }
      aString.erase(0, i);
   }
   /* remove number from string, copy number to buffer */
   i = aString.find_first_not_of("0123456789-+.eE", 0);
   end = aString.length();
   if (i != aString.npos)
   {
      buffer.assign(aString, 0, i);
      aString.erase(0, i);
   }
   //if number is at end of string
   else
   {
      buffer.assign(aString);
      aString.erase(0, end);
   }
   /* remove any whitespace after the string, but don't remove any tabs */
   i = findFirstNonWhiteSpace(aString);
   if (i != aString.npos && (i > 0) && (aString[i-1] != '\t'))
      aString.erase(0, i);

   if (buffer.empty())
      return false;
   *rNumber = atof(buffer.c_str());
   return true;
}

//_____________________________________________________________________________
/**
 * Read a vector from an input string. The input string is
 * modified by deleting the double from it after it is read.
 *
 * @param aString input string to read from.
 * @param rVec vector that is read is returned here.
 * @return True if vector was read, false if not.
 */
bool OpenSim::readVectorFromString(string &aString, SimTK::Vec3 &rVec)
{
    bool ok = true;

    SimTK::Vec3& vecPtr = rVec;
    ok = ok && OpenSim::readDoubleFromString(aString, &vecPtr[0]);
    ok = ok && OpenSim::readDoubleFromString(aString, &vecPtr[1]);
    ok = ok && OpenSim::readDoubleFromString(aString, &vecPtr[2]);
    return ok;
}

//_____________________________________________________________________________
/**
 * Read a vector from an input string. The input string is
 * modified by deleting the double from it after it is read.
 *
 * @param aString input string to read from.
 * @param rVX X coordinate of vector is returned here.
 * @param rVY Y coordinate of vector is returned here.
 * @param rVZ Z coordinate of vector is returned here.
 * @return True if vector was read, false if not.
 */
bool OpenSim::readVectorFromString(string &aString, double *rVX, double *rVY, double *rVZ)
{
   bool ok = true;

   ok = ok && OpenSim::readDoubleFromString(aString, rVX);
   ok = ok && OpenSim::readDoubleFromString(aString, rVY);
   ok = ok && OpenSim::readDoubleFromString(aString, rVZ);
   return ok;
}

//_____________________________________________________________________________
/**
 * Read tab-delimited XYZ coordinate values from a string. If there are
 * 2 tabs in a row, this indicates a missing coordinate, so fill it in
 * with SimTK::NaN. The input string is modified by deleting the
 * coordinates from it after they have been read.
 *
 * @param aString input string to read from.
 * @param rVec vector of coordinates is returned here.
 * @return True if coordinates were read, false if not.
 */
bool OpenSim::readCoordinatesFromString(string &aString, double rVec[3], bool allowNaNs)
{
   int numTabs = 0, numCoords = 0;
   double value;

   while (!aString.empty())
   {
      if (aString[0] == '\t')
      {
         numTabs++;
         aString.erase(0, 1);
      }
      else
      {
         if (!OpenSim::readDoubleFromString(aString, &value, allowNaNs))
         {
            return false;
         }
         else
         {
            rVec[numCoords++] = value;
            numTabs = 0;
         }         
      }
      /* if you have 3 TABS in a row, coordinate data is missing */
      if (numTabs == 3)
      {
            rVec[0] = rVec[1] = rVec[2] = SimTK::NaN;
         numCoords = 3;
      }
      if (numCoords == 3)
         break;
   }
   if (numCoords == 3)
      return true;
   else
      return false;
}

//_____________________________________________________________________________
/**
 * Find the first non-white-space character in a string.
 *
 * @param aString input string to scan.
 * @return Position of first non-white-space character.
 */
int OpenSim::findFirstNonWhiteSpace(string &aString)
{
   return (int)aString.find_first_not_of(" \t\r\n", 0);
}

//_____________________________________________________________________________
/**
 * Find the first white-space character in a string.
 *
 * @param aString input string to scan.
 * @return Position of first white-space character.
 */
int OpenSim::findFirstWhiteSpace(string &aString)
{
   return (int)aString.find_first_of(" \t\r\n", 0);
}

//_____________________________________________________________________________
/**
 * Convert a text string into a single token that can
 * serve as a C/C++ variable name.
 *
 * @param aString input string to convert.
 * @param aPrependUnderscore whether to prepend underscore before numbers
 */
void OpenSim::convertString(string& aString, bool aPrependUnderscore)
{
   for (unsigned int i = 0; i < aString.size(); i++)
   {
      if (aString[i] >= 'a' && aString[i] <= 'z')
         continue;
      if (aString[i] >= 'A' && aString[i] <= 'Z')
         continue;
      if (aString[i] >= '0' && aString[i] <= '9')
         continue;
      aString[i] = '_';
   }

   /* If the first character is a number, prepend an underscore. */
   if (aPrependUnderscore && aString[0] >= '0' && aString[0] <= '9')
        aString.insert(0, "_");
}

string OpenSim::getCurrentTimeString()
{
   time_t t = time(NULL);

    char buf[100];
   strftime(buf, 100, "%m/%d/%Y %I:%M:%S %p", localtime(&t));

    return string(buf);
}
