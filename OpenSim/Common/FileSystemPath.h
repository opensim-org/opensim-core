#ifndef OPENSIM_FILESYSTEMPATH_H_
#define OPENSIM_FILESYSTEMPATH_H_
/* -------------------------------------------------------------------------- *
 *                             OpenSim: FileSystemPath.h                      *
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
#include "OpenSim/Common/Object.h"
#include "Simbody.h"

namespace OpenSim {

//==============================================================================
//                            OPENSIM FILESYSTEMPATH
//==============================================================================
/**

*
* @author Carmichael Ong
*/
class FileSystemPath : public Object {
    OpenSim_DECLARE_CONCRETE_OBJECT(FileSystemPath, Object);

public:
    //==============================================================================
    // METHODS
    //==============================================================================
    /** Default constructor **/
    FileSystemPath();

    /** Construct FileSystemPath from a string. **/
    FileSystemPath(std::string& pathName);
    FileSystemPath(std::string& pathName, bool isFile);

    /** Use default copy constructor and assignment operator. */
    FileSystemPath(const FileSystemPath&) = default;
    FileSystemPath& operator=(const FileSystemPath&) = default;

    /** Destructor. **/
    ~FileSystemPath() = default;

    // Return the absolute pathname relative to the working directory
    FileSystemPath getAbsolutePath();
    FileSystemPath getAbsoluteFileName();
    FileSystemPath getAbsoluteDir();

    // Return the absolute pathname relative to some specified relativeDir
    FileSystemPath getAbsolutePathNameWithRelativeDir(FileSystemPath relativeDir);

    // Get relative pathname
    FileSystemPath getRelativePathNameFromOtherDir(FileSystemPath otherDir);

    FileSystemPath getAbsoluteDirForFile();

    // Convience methods for getting parts of the path as strings.
    std::string getDirString();
    std::string getFileString();
    std::string getExtString();
    void deconstructFilePathName(std::string& directory,
                                 std::string& fileName,
                                 std::string& extension);

    // GET AND SET MEMBER VARIABLES
    std::string getPathString() { return _pathStr; }
    bool isFile() { return _isFile; }
    //bool isDir() { return _isDir; }

    void setPathString(std::string& aFileName) 
    {
        IO::TrimWhitespace(aFileName);
        _pathStr = aFileName; 
    }
    void setIsFile(bool isFile) { _isFile = isFile; }
    //void setIsDir(bool isDir) { _isDir = isDir; }

private:
    std::string _pathStr;
    bool _isFile;
    //bool _isDir;
}; // end class FilePathName

} // end of namespace OpenSim

#endif // OPENSIM_FILESYSTEMPATH_H_