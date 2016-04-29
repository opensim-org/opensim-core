#ifndef OPENSIM_FILESYSTEMPATH_H_
#define OPENSIM_FILESYSTEMPATH_H_
/* -------------------------------------------------------------------------- *
 *                         OpenSim: FileSystemPath.h                          *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2014 Stanford University and the Authors                *
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

// INCLUDES
#include <OpenSim/Common/osimCommonDLL.h>
#include "OpenSim/Common/IO.h"
#include "OpenSim/Common/Object.h"
#include "Simbody.h"

namespace OpenSim {

//==============================================================================
//                            OPENSIM FILESYSTEMPATH
//==============================================================================
/**
* A class for handling file system paths. The class holds two variables: 1) a
* string for the path and 2) a bool _isFile to denote of the path is a file
* or a directory.
*
* For convention, the word "Path" can refer to the FileSystemPath for either
* a file or directory. "FilePath" and "DirPath" are specific to a file or
* directory, respectively.
*
* This class has methods for resolving paths with respect to some other given
* directory (i.e. not necessarily the current working directory). In order to
* resolve these cases, we use methods in SimTK::Pathname. Refer to that class's
* documentation for the rules of how these are evaluated.
*
* @author Carmichael Ong
*/
class FileSystemPath : public Object {
    OpenSim_DECLARE_CONCRETE_OBJECT(FileSystemPath, Object);

public:
    /// Default constructor
    FileSystemPath();

    /// Construct FileSystemPath from a string.
    FileSystemPath(std::string& path, bool isFile = true);

    /// Use default copy constructor and assignment operator.
    FileSystemPath(const FileSystemPath&) = default;
    FileSystemPath& operator=(const FileSystemPath&) = default;

    /// Destructor.
    ~FileSystemPath() = default;

    /// Get a FileSystemPath for the current working directory.
    FileSystemPath getWorkingDirPath();

    /// Get absolute path to a file or directory. Checks if the FileSystemPath
    /// is a file or directory first.
    FileSystemPath getAbsolutePath();

    /// Get the absolute path for the directory of a file. Checks if it is
    /// given a file first, then gives the absolute path to the directory
    /// containing the file. If the path was not an absolute path, then the
    /// directory is resolved with respect to the current working directory.
    FileSystemPath getAbsoluteDirPathForFile();

    /// Return the absolute path relative to some specified relativeDir.
    FileSystemPath getAbsolutePathWithRelativeDir(FileSystemPath relativeDir);

    /// Get relative pathname
    FileSystemPath getRelativePathFromOtherDir(FileSystemPath otherDir);

    /// Convience methods for getting parts of the path as strings.
    std::string getDirString();
    std::string getFileString();
    std::string getExtString();

    /// GET AND SET MEMBER VARIABLES
    std::string getPathString() { return _pathStr; }
    bool isFile() { return _isFile; }

    void setPathString(std::string& aFileName) 
    {
        IO::TrimWhitespace(aFileName);
        _pathStr = aFileName; 
    }
    void setIsFile(bool isFile) { _isFile = isFile; }

private:
    // Helper functions for getAbsolutePath()
    FileSystemPath getAbsoluteFilePath();
    FileSystemPath getAbsoluteDirPath();

    // Helper function for getXString() functions
    void deconstructFilePathName(std::string& directory,
        std::string& fileName,
        std::string& extension);

    // FileSystemPath variables
    std::string _pathStr;
    bool _isFile;
}; // end class FilePathName

} // end of namespace OpenSim

#endif // OPENSIM_FILESYSTEMPATH_H_
