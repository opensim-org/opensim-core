#ifndef _simmIO_h_
#define _simmIO_h_

// simmIO.h
// Author: Peter Loan
/* Copyright (c) 2005, Stanford University and Peter Loan.
 * 
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

bool readNonCommentString(std::istream &stream, std::string &buffer);
bool readString(std::istream &stream, std::string &buffer);
bool readStringFromLine(std::string &line, std::string &buffer);
bool readTabDelimitedStringFromLine(std::string &line, std::string &buffer);
bool readIntegerFromString(std::string &line, int *x);
bool readDoubleFromString(std::string &line, double *x);
bool readVectorFromString(std::string &line, OpenSim::SimmPoint &vec);
bool readVectorFromString(std::string &line, double *v1, double *v2, double *v3);
bool readCoordinatesFromLine(std::string &line, double vec[3]);
int findFirstNonWhiteSpace(std::string &buffer);
int findFirstWhiteSpace(std::string &buffer);

#endif // __simmIO_h__


