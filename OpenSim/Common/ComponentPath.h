#ifndef OPENSIM_COMPONENT_PATH_H_
#define OPENSIM_COMPONENT_PATH_H_
/* -------------------------------------------------------------------------- *
 *                      OpenSim: ComponentPath.h                              *
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

const char separator = '/';
const std::string invalidChars = "\\/*+";

namespace OpenSim {

//==============================================================================
//                            OPENSIM COMPONENT PATH
//==============================================================================
/**
* A class for handling Paths for Components, deriving from Path. A ComponentPath
* uses a forward-slash ('/') as a separator always. It also specifies invalid
* characters in a Component name ("\\/*+").
*
* @author Carmichael Ong
*/

class OSIMCOMMON_API ComponentPath : public Path {
    OpenSim_DECLARE_CONCRETE_OBJECT(ComponentPath, Path);

public:
    /// Constructors
    ComponentPath() = default;
    ComponentPath(const std::string path);
    ComponentPath(std::vector<std::string> pathVec, bool isAbsolute);

    ComponentPath getAbsolutePath(ComponentPath* otherPath);
    ComponentPath getRelativePath(ComponentPath* otherPath);
};


} // end of namespace OpenSim
#endif // OPENSIM_COMPONENT_PATH_H_
