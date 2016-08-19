/* -------------------------------------------------------------------------- *
 *                            OpenSim: Path.cpp                               *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2012 Stanford University and the Authors                *
 * Author(s): Carmichael Ong                                                  *
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


// C++ INCLUDES
#include "Path.h"

using namespace OpenSim;
using namespace std;


//=============================================================================
// CONSTRUCTORS
//=============================================================================
//_____________________________________________________________________________

//_____________________________________________________________________________
/**
 * Constructors
 */
Path::Path(const std::string path,
           const char separator,
           const std::string invalidChars) :
    _separator(separator), _invalidChars(invalidChars), _isAbsolute(false)
{
    // check if this is an absolute path
    char firstChar = path.at(0);
    if (firstChar == _separator) _isAbsolute = true;
    
    // separate the path (there may be multiple separators at the front)
    size_t start = path.find_first_not_of(separator);
    while (start != std::string::npos) {
        size_t end = path.find_first_of(separator, start);
        std::string pathElement = path.substr(start, end - start + 1);
        appendPathElement(pathElement);
        start = path.find_first_not_of(separator, end + 1);
    }
}

Path* Path::getAbsolutePath() {
    Path* absPath = getCurrentPath();
    for (auto pathElement : _path) {
        absPath->appendPathElement(pathElement);
    }

    return absPath;
}

Path* Path::findRelativePath(Path* relPath) {
    
}

void Path::cleanPath() {
    size_t numPathElements = _path.size();
    size_t i = 0;
    while (i < numPathElements) {
        if (_path[i] == ".") {
            _path.erase(_path.begin() + i);
            --numPathElements;
        }
        else if (_path[i] == "..") {
            if (i != 0) {
                _path.erase(_path.begin() + i);
                _path.erase(_path.begin() + i - 1);
                numPathElements -= 2;
                --i;
            }
        }
        else {
            ++i;
        }
    }
}

std::string Path::getString() {
    std::string outString;
    if (_isAbsolute) outString.append(1, _separator);
    for (size_t i = 0; i < _path.size(); ++i) {
        outString.append(_path[i]);
        if (i != _path.size()) {
            outString.append(1, _separator);
        }
    }

    return outString;
}

bool Path::isLegalPathElement(const std::string pathElement) {
    if (pathElement.find_first_of(_invalidChars) != std::string::npos) {
        return false;
    }
    return true;
}

void Path::appendPathElement(const std::string pathElement) {
    if (!isLegalPathElement(pathElement)) {
        Exception("invalid character use");
    }
    _path.push_back(pathElement);
}