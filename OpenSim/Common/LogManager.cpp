/* -------------------------------------------------------------------------- *
 *                          OpenSim:  LogManager.cpp                          *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2012 Stanford University and the Authors                *
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
#include "common.h"


using namespace OpenSim;

// Initialize static members
LogBuffer LogManager::out;
LogBuffer LogManager::err;
std::ostream LogManager::cout(std::cout.rdbuf()); // This cout writes to the actual standard out
std::ostream LogManager::cerr(std::cerr.rdbuf()); // This cerr writes to the actual standard error

// Initialize a static log manager to force the constructor to be called
LogManager logManager;

//=============================================================================
// FileLogCallback
//=============================================================================
StreamLogCallback::StreamLogCallback(const std::string &filename)
    : _out(new std::ofstream(filename.c_str())), _ownsStream(true)
{}

StreamLogCallback::StreamLogCallback(std::ostream *aOut, bool aOwnsStream)
    : _out(aOut), _ownsStream(aOwnsStream)
{}

StreamLogCallback::~StreamLogCallback()
{
    if(_ownsStream) delete _out;
}

void StreamLogCallback::log(const std::string &str)
{
    *_out << str << std::flush;
}

//=============================================================================
// LogBuffer
//=============================================================================
LogBuffer::LogBuffer()
{
}

LogBuffer::~LogBuffer()
{
    for(int i = 0; i < _logCallbacks.size(); i++) {
        delete _logCallbacks[i];
    }
}

// Assumes caller owns this output stream (will never be deleted)
bool LogBuffer::
addLogCallback(LogCallback *aLogCallback)
{
    if(_logCallbacks.findIndex(aLogCallback) >= 0) return false;
    _logCallbacks.append(aLogCallback); 
    return true;
}

bool LogBuffer::
removeLogCallback(LogCallback *aLogCallback)
{
    int index = _logCallbacks.findIndex(aLogCallback);
    if(index < 0) return false;
    _logCallbacks.remove(index); 
    return true;
}

int LogBuffer::
sync()
{
    // Pass current string to all log callbacks
    for(int i=0; i<_logCallbacks.getSize(); i++) _logCallbacks[i]->log(str());
    // Reset current buffer contents
    str("");
    return std::stringbuf::sync();
}

//=============================================================================
// LogManager
//=============================================================================
LogManager::LogManager()
{
    // Seems to be causing crashes in the GUI... maybe a multithreading issue.
#if 1
    // Change the underlying streambuf for the standard cout/cerr to our custom buffers
    std::cout.rdbuf(&out);
    std::cerr.rdbuf(&err);

    // Example setup: redirect output to both the terminal and an output file
    out.addLogCallback(new StreamLogCallback(&cout,false));
    out.addLogCallback(new StreamLogCallback("out.log"));
    err.addLogCallback(new StreamLogCallback(&cerr,false));
    err.addLogCallback(new StreamLogCallback("err.log"));
#endif
}

LogManager::~LogManager()
{
    std::cout << std::flush;
    std::cerr << std::flush;
}

LogManager *LogManager::getInstance()
{
    return &logManager;
}

LogBuffer *LogManager::getOutBuffer()
{
    return &out;
}

LogBuffer *LogManager::getErrBuffer()
{
    return &err;
}
