#ifndef __SimmIO_h__
#define __SimmIO_h__

// SimmIO.h
// Author: Peter Loan
/*
 * Copyright (c) 2006, Stanford University. All rights reserved. 
 * Permission is hereby granted, free of charge, to any person obtaining
 * a copy of this software and associated documentation files (the
 * "Software"), to deal in the Software without restriction, including 
 * without limitation the rights to use, copy, modify, merge, publish, 
 * distribute, sublicense, and/or sell copies of the Software, and to
 * permit persons to whom the Software is furnished to do so, subject
 * to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included 
 * in all copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
 * OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
 * IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY
 * CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
 * TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
 * SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */


// INCLUDE
#include <iostream>
#include <string>
#include "SimmPoint.h"

namespace OpenSim {

bool OSIMCOMMON_API readNonCommentStringFromStream(std::istream &aStream, std::string &rBuffer);
bool OSIMCOMMON_API readStringFromStream(std::istream &aStream, std::string &rBuffer);
bool OSIMCOMMON_API readStringFromString(std::string &aString, std::string &rBuffer);
bool OSIMCOMMON_API readTabDelimitedStringFromString(std::string &aString, std::string &rBuffer);
bool OSIMCOMMON_API readIntegerFromString(std::string &aString, int *rNumber);
bool OSIMCOMMON_API readDoubleFromString(std::string &aString, double *rNumber);
bool OSIMCOMMON_API readVectorFromString(std::string &aString, SimmPoint &rVec);
bool OSIMCOMMON_API readVectorFromString(std::string &aString, double *rVX, double *rVY, double *rVZ);
bool OSIMCOMMON_API readCoordinatesFromString(std::string &aString, double rVec[3]);
int OSIMCOMMON_API findFirstNonWhiteSpace(std::string &aString);
int OSIMCOMMON_API findFirstWhiteSpace(std::string &aString);
void OSIMCOMMON_API convertString(std::string& aString, bool aPrependUnderscore);
std::string OSIMCOMMON_API getCurrentTimeString();

} // end of namespace OpenSim

#endif // __SimmIO_h__


