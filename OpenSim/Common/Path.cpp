/* -------------------------------------------------------------------------- *
 *                            OpenSim: Path.cpp                               *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2017 Stanford University and the Authors                *
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

Path::Path(const char separator, const std::string invalidChars) :
    _separator(separator), _invalidChars(invalidChars), _isAbsolute(false)
{}

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
        // if last segment (without a "/" at the end)
        if (end == std::string::npos) {
            end = path.find_last_not_of(separator, std::string::npos) + 1;
        }
        std::string pathElement = path.substr(start, end - start);
        appendPathElement(pathElement);
        start = path.find_first_not_of(separator, end + 1);
    }

    trimDotAndDotDotElements();
}

Path::Path(const std::vector<std::string> pathVec,
           const char separator,
           const std::string invalidChars,
           bool isAbsolute) :
    _path(pathVec), _separator(separator), _invalidChars(invalidChars),
    _isAbsolute(isAbsolute)
{
    if (_path.empty()) return;
    trimDotAndDotDotElements();
    if (!isLegalPathVec(_path)) {
        OPENSIM_THROW(Exception, "Invalid character used in the path");
    }
}

std::string Path::toString() const
{
    std::string outString;
    if (_isAbsolute) outString.append(1, _separator);
    for (size_t i = 0; i < getNumPathLevels(); ++i) {
        outString.append(_path[i]);
        if (i != getNumPathLevels() - 1) {
            outString.append(1, _separator);
        }
    }

    return outString;
}

void Path::insertPathElement(size_t pos, const std::string& pathElement) 
{
    if (pos > getNumPathLevels()) {
        OPENSIM_THROW(Exception, "Index is out of range of elements");
    }

    if (!isLegalPathElement(pathElement)) {
        OPENSIM_THROW(Exception, "Invalid character used in pathElement");
    }

    if (pathElement.empty()) {
        OPENSIM_THROW(Exception, "pathElement cannot be an empty string");
    }

    _path.insert(_path.begin() + pos, pathElement);
}

void Path::erasePathElement(size_t pos)
{
    if (pos > getNumPathLevels() - 1) {
        OPENSIM_THROW(Exception, "Index is out of range of elements");
    }

    _path.erase(_path.begin() + pos);
}

std::vector<std::string> Path::formAbsolutePathVec(const Path& otherPath) const
{
    if (_isAbsolute) {
        return _path;
    }
    
    if (!otherPath._isAbsolute) {
        OPENSIM_THROW(Exception, "'otherPath' must be an absolute path");
    }

    std::vector<std::string> pathVec;
    for (const auto& pathElement : otherPath._path) {
        pathVec.push_back(pathElement);
    }

    for (const auto& pathElement : _path) {
        pathVec.push_back(pathElement);
    }

    return pathVec;
}

std::vector<std::string> Path::formRelativePathVec(const Path& otherPath) const
{
    if (!this->_isAbsolute || !otherPath._isAbsolute) {
        OPENSIM_THROW(Exception, "Both paths must be absolute paths");
    }

    size_t thisPathLength = this->getNumPathLevels();
    size_t otherPathLength = otherPath.getNumPathLevels();

    // find how many elements at the head are in common
    size_t searchLength = std::min(thisPathLength, otherPathLength);
    bool match = true;
    size_t ind = 0;
    while (match && ind < searchLength) {
        if (this->_path[ind] != otherPath._path[ind]) {
            match = false;
        }
        else {
            ++ind;
        }
    }

    // ind is the first index that doesn't match. first add any necessary
    // "..", then add the remainder of the pathElements to get to otherPath
    std::vector<std::string> pathVec;
    
    size_t numDotDot = otherPathLength - ind;
    for (size_t i = 0; i < numDotDot; ++i) {
        pathVec.push_back("..");
    }
    if (thisPathLength == 0) {
        // This path points to the root, which we treat as having an empty name.
        pathVec.push_back("");
    } else {
        for (; ind < thisPathLength; ++ind) {
            pathVec.push_back(_path[ind]);
        }
    }

    return pathVec;
}


void Path::trimDotAndDotDotElements() 
{
    size_t numPathElements = getNumPathLevels();
    size_t i = 0;
    while (i < numPathElements) {
        // remove any "." element
        if (_path[i] == ".") {
            erasePathElement(i);
            --numPathElements;
        }

        // if it's not the first element, and the previous element is not ".."
        // remove the ".." element and the previous element
        else if (_path[i] == "..") {
            if (i != 0 && _path[i-1] != "..") {
                erasePathElement(i);
                erasePathElement(i - 1);
                numPathElements -= 2;
                --i;
            }
            else {
                ++i;
            }
        }

        // otherwise, move on
        else {
            ++i;
        }
    }

    if (!_path.empty() && _path[0] == ".." && _isAbsolute) {
        OPENSIM_THROW(Exception, "Absolute path cannot start with '..'");
    }
}

bool Path::isLegalPathElement(const std::string& pathElement) const
{
    if (pathElement.find_first_of(_invalidChars) != std::string::npos) {
        return false;
    }
    return true;
}

bool Path::isLegalPathVec(const std::vector<std::string>& pathVec) const
{
    for (const std::string& pathElement : pathVec) {
        if (!isLegalPathElement(pathElement)) return false;
    }
    return true;
}

void Path::appendPathElement(const std::string& pathElement) 
{
    if (!isLegalPathElement(pathElement)) {
        OPENSIM_THROW(Exception, 
            "Invalid character used in pathElement '" + pathElement + "'.");
    }

    if (pathElement.empty()) {
        OPENSIM_THROW(Exception, "pathElement cannot be an empty string");
    }

    _path.push_back(pathElement);
}
