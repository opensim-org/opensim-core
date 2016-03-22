/* -------------------------------------------------------------------------- *
 *                  OpenSim:  FileSystemPath.cpp                              *
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
#include "FileSystemPath.h"

using namespace OpenSim;
using namespace std;

//=============================================================================
// CONSTRUCTORS
//=============================================================================
//_____________________________________________________________________________

//_____________________________________________________________________________
/**
 * Default constructor.
 */
FileSystemPath::FileSystemPath() :
    _isFile(false),
    _isDir(false),
    _pathStr("")
{}

FileSystemPath::FileSystemPath(const std::string& aFileName) : FileSystemPath()
{
    _pathStr = aFileName;
}

FileSystemPath FileSystemPath::getAbsolutePathName()
{
    FileSystemPath workingDir(SimTK::Pathname::getCurrentWorkingDirectory());
    workingDir.setIsDir(true);
    return getAbsolutePathNameWithRelativeDir(workingDir);
}

FileSystemPath FileSystemPath::getAbsolutePathNameWithRelativeDir(FileSystemPath relativeDir)
{
    if (!relativeDir.isDir()) {
        throw Exception("relativeDir is not a directory.");
    }

    std::string absolutePathStr = 
        SimTK::Pathname::getAbsolutePathnameUsingSpecifiedWorkingDirectory(relativeDir.getFileString(), 
                                                                           getFileString());
    FileSystemPath absolutePath = FileSystemPath(absolutePathStr);
    return absolutePath;
}

std::string FileSystemPath::getDirString()
{
    std::string directory, fileName, extension;
    deconstructFilePathName(directory, fileName, extension);
    return directory;
}

std::string FileSystemPath::getFileString()
{
    if (!isFile())
    {
        throw Exception("getFileString(): given a path that is not a file.");
    }
    std::string directory, fileName, extension;
    deconstructFilePathName(directory, fileName, extension);
    return fileName;
}

std::string FileSystemPath::getExtString()
{
    if (!isFile())
    {
        throw Exception("getExtString(): given a path that is not a file.");
    }
    std::string directory, fileName, extension;
    deconstructFilePathName(directory, fileName, extension);
    return extension;
}

void FileSystemPath::deconstructFilePathName(std::string& directory,
                                             std::string& fileName,
                                             std::string& extension)
{
    bool dontApplySearchPath;
    SimTK::Pathname::deconstructPathname(getPathString(), 
        dontApplySearchPath, directory, fileName, extension);
}