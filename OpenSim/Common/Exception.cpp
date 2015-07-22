/* -------------------------------------------------------------------------- *
 *                          OpenSim:  Exception.cpp                           *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2012 Stanford University and the Authors                *
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
#include <iostream>
#include <string>
#include <cassert>
#include "osimCommonDLL.h"
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
Exception(const string &aMsg,const string &aFile,int aLine):
exception()
{
    setNull();

    setMessage(aMsg);
    _file = aFile;
    _line = aLine;

// make it assert false when debugging...
//#ifndef NDEBUG
//  print(cout);
//  assert(false);
//#endif
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
getMessage() const
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
print(ostream &aOut) const
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
    aOut << '\n' << endl;
}

