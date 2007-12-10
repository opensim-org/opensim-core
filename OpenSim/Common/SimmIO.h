#ifndef __SimmIO_h__
#define __SimmIO_h__

// SimmIO.h
// Author: Peter Loan
/*
 * Copyright (c)  2006, Stanford University. All rights reserved. 
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


