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

#include <algorithm>

using namespace OpenSim;
using namespace std;

#ifdef WIN32
    bool isWindows = true;
#else
    bool isWindows = false;
#endif

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
    //_isDir(false),
    _pathStr("")
{}

FileSystemPath::FileSystemPath(std::string& pathName) : FileSystemPath()
{
    IO::TrimWhitespace(pathName);
    _pathStr = pathName;
}

FileSystemPath::FileSystemPath(std::string& pathName, bool isFile) :
    FileSystemPath()
{
    IO::TrimWhitespace(pathName);
    _pathStr = pathName;
    _isFile = isFile;
}

FileSystemPath FileSystemPath::getAbsolutePath()
{
    if (isFile())
    {
        return getAbsoluteFilePath();
    } 
    else {
        return getAbsoluteDirPath();
    }
}

FileSystemPath FileSystemPath::getAbsoluteFilePath()
{
    if (!isFile())
    {
        throw Exception("getAbsoluteFilePath(): not a file");
    }
    auto absPathStr = 
        SimTK::Pathname::getAbsolutePathname(getPathString());
    FileSystemPath absPath(absPathStr);

    return absPath;
}

FileSystemPath FileSystemPath::getAbsoluteDirPath()
{
    if (!isFile())
    {
        throw Exception("getAbsoluteDirPath(): not a directory");
    }
    auto absPathStr = 
        SimTK::Pathname::getAbsoluteDirectoryPathname(getPathString());
    FileSystemPath absPath(absPathStr);

    return absPath;
}

FileSystemPath FileSystemPath::getAbsolutePathWithRelativeDir(
    FileSystemPath relativeDir)
{
    if (relativeDir.isFile()) 
    {
        throw Exception("relativeDir is not a directory.");
    }

    std::string absPathStr = 
        SimTK::Pathname::getAbsolutePathnameUsingSpecifiedWorkingDirectory(relativeDir.getFileString(), 
                                                                           getFileString());
    auto absPath = FileSystemPath(absPathStr, isFile());
    return absPath;
}

FileSystemPath FileSystemPath::getRelativePathFromOtherDir(
    FileSystemPath otherDir)
{
    // Initialize thisPath and otherPath strings. Also error check.
    std::string thisPath, otherPath;
    if (otherDir.isFile())
    {
        throw Exception("otherDir is not a directory.");
    }

    else {
        otherPath = otherDir.getPathString();
    }

    if (isFile())
    {
        thisPath = getAbsoluteDirPathForFile().getPathString();
    } 
    
    else {
        thisPath = getPathString();
    }

    // Change all "\\" to "/" for consistency
    if (isWindows)
    {
        
    }

    FileSystemPath dummy;
    return dummy;
}

FileSystemPath FileSystemPath::getWorkingDirPath()
{
    FileSystemPath workingDir(
        SimTK::Pathname::getCurrentWorkingDirectory(), false);
    return workingDir;
}

FileSystemPath FileSystemPath::getAbsoluteDirPathForFile()
{
    if (!isFile())
    {
        throw Exception("getAbsoluteDirForFile(): given a directory");
    }

    std::string directory, fileName, extension;
    deconstructFilePathName(directory, fileName, extension);
    FileSystemPath dirPath(
        SimTK::Pathname::getAbsoluteDirectoryPathname(directory), false);

    return dirPath;
}

std::string FileSystemPath::getDirString()
{
    if (isFile())
    {
        std::string directory, fileName, extension;
        deconstructFilePathName(directory, fileName, extension);
        return directory;
    }
   
    return getPathString();
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