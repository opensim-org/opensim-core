/* -------------------------------------------------------------------------- *
 *                            OpenSim: Path.cpp                               *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2016 Stanford University and the Authors                *
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
    // check if path is empty
    if (path.empty()) return;

    // check if this is an absolute path
    char firstChar = path.at(0);
    if (firstChar == _separator) _isAbsolute = true;
    
    // separate the path (there may be multiple separators at the front)
    size_t start = path.find_first_not_of(separator);
    while (start != std::string::npos) {
        size_t end = path.find_first_of(separator, start);
        if (end == std::string::npos) break;
        std::string pathElement = path.substr(start, end - start);
        appendPathElement(pathElement);
        start = path.find_first_not_of(separator, end + 1);
    }
}

Path::Path(std::vector<std::string> pathVec,
           const char separator,
           const std::string invalidChars,
           bool isAbsolute) :
    _path(pathVec), _separator(separator), _invalidChars(invalidChars),
    _isAbsolute(isAbsolute)
{}

std::vector<std::string> Path::getAbsolutePathVec(Path* otherPath) {
    if (_isAbsolute) {
        return _path;
    }
    
    if (otherPath->_isAbsolute) {
        Exception("otherPath is not an absolute path");
    }

    std::vector<std::string> pathVec;
    for (auto pathElement : otherPath->_path) {
        pathVec.push_back(pathElement);
    }

    for (auto pathElement : _path) {
        pathVec.push_back(pathElement);
    }

    return pathVec;
}

std::vector<std::string> Path::findRelativePathVec(Path* otherPath) {
    if (!this->_isAbsolute || !otherPath->_isAbsolute) {
        Exception("both paths must be absolute to find relative path");
    }

    size_t thisPathLength = this->getPathLength();
    size_t otherPathLength = otherPath->getPathLength();

    // find how many elements at the head are in common
    size_t searchLength = std::min(thisPathLength, otherPathLength);
    bool match = true;
    size_t ind = 0;
    while (match && ind < searchLength) {
        if (this->getPathElement(ind) != otherPath->getPathElement(ind)) {
            match = false;
        }
        else {
            ++ind;
        }
    }

    // ind is the first index that doesn't match. first add any necessary
    // "..", then add the remainder of the pathElements to get to otherPath
    std::vector<std::string> pathVec;
    
    size_t numDotDot = thisPathLength - ind;
    for (int i = 0; i < numDotDot; ++i) {
        pathVec.push_back("..");
    }
    for (; ind < otherPathLength; ++ind) {
        pathVec.push_back(otherPath->getPathElement(ind));
    }

    return pathVec;
}

void Path::cleanPath() {
    size_t numPathElements = getPathLength();
    size_t i = 0;
    while (i < numPathElements) {
        // remove any "." element
        if (_path[i] == ".") {
            _path.erase(_path.begin() + i);
            --numPathElements;
        }

        // if it's not the first element, remove the ".." relement and
        // the previous element
        else if (_path[i] == "..") {
            if (i != 0) {
                _path.erase(_path.begin() + i);
                _path.erase(_path.begin() + i - 1);
                numPathElements -= 2;
                --i;
            }
        }

        // otherwise, move on
        else {
            ++i;
        }
    }
}

std::string Path::getString() {
    std::string outString;
    if (_isAbsolute) outString.append(1, _separator);
    for (size_t i = 0; i < getPathLength(); ++i) {
        outString.append(_path[i]);
        if (i != getPathLength()) {
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
