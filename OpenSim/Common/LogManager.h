#ifndef _LogManager_h_
#define _LogManager_h_
/* -------------------------------------------------------------------------- *
 *                           OpenSim:  LogManager.h                           *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2017 Stanford University and the Authors                *
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

#include "osimCommonDLL.h"
#include "Array.h"
#include "LogCallback.h"
#include <iostream>
#include <sstream>

namespace OpenSim {

// Excluding this from Doxygen until it has better documentation! -Sam Hamner
/// @cond

class OSIMCOMMON_API StreamLogCallback : public LogCallback
{
private:
    std::ostream *_out;
    bool _ownsStream;
    
public:
    StreamLogCallback(const std::string &aFilename);
    StreamLogCallback(std::ostream *aOut, bool aOwnsStream = true);
    ~StreamLogCallback();
    void log(const std::string &aStr) override;
};


class OSIMCOMMON_API LogBuffer : public std::stringbuf
{
public:
    LogBuffer();
    ~LogBuffer();
    bool addLogCallback(LogCallback *aLogCallback);
    bool removeLogCallback(LogCallback *aLogCallback);

private:
    Array<LogCallback*> _logCallbacks;

    int sync() override;
};
/// @endcond

// Excluding this from Doxygen until it has better documentation! -Sam Hamner
/// @cond
class OSIMCOMMON_API LogManager
{
public:
    // Expose these members so users can manipulate output formats by calling functions on LogManager::out/err
    static LogBuffer out;
    static LogBuffer err;

    // LogManager's cout and cerr act as the normal standard output and error (i.e. they write to the terminal)
    static std::ostream cout;
    static std::ostream cerr;

    LogManager();
    ~LogManager();

    static LogManager *getInstance();

    LogBuffer *getOutBuffer();
    LogBuffer *getErrBuffer();
};
/// @endcond
}

#endif
