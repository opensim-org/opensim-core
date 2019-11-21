/* -------------------------------------------------------------------------- *
 *                      OpenSim:  ComponentSocket.cpp                         *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2017 Stanford University and the Authors                *
 * Author(s): Ajay Seth, Chris Dembia                                         *
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

#include "ComponentSocket.h"
#include "Component.h"

using namespace OpenSim;

const Property<std::string>&
AbstractSocket::getConnecteePathProp() const {
    return _owner->getProperty<std::string>(_connecteePathIndex);
}

Property<std::string>&
AbstractSocket::updConnecteePathProp() {
    auto* owner = const_cast<Component*>(_owner.get());
    // We do not want to flip the isObjectUpToDateWithProperties flag.
    const auto& prop = owner->getProperty<std::string>(_connecteePathIndex);
    return const_cast<Property<std::string>&>(prop);
}

void AbstractSocket::prependComponentPathToConnecteePath(
        const std::string& pathToPrepend) {
    for (unsigned iConn = 0u; iConn < getNumConnectees(); ++iConn) {
        ComponentPath path(getConnecteePath(iConn));
        if (path.isAbsolute()) {
            ComponentPath newPath(pathToPrepend);
            for (int iPath = 0; iPath < (int)path.getNumPathLevels();
                 ++iPath) {
                newPath.pushBack(
                        path.getSubcomponentNameAtLevel(iPath));
            }
            setConnecteePath(newPath.toString(), iConn);
        }
    }
}

void AbstractInput::prependComponentPathToConnecteePath(
        const std::string& pathToPrepend) {
    for (unsigned iConn = 0u; iConn < getNumConnectees(); ++iConn) {
        std::string connecteePath = getConnecteePath(iConn);
        std::string componentPath;
        std::string outputName;
        std::string channelName;
        std::string alias;
        AbstractInput::parseConnecteePath(connecteePath,
                                          componentPath,
                                          outputName,
                                          channelName,
                                          alias);
        ComponentPath path(componentPath);
        if (path.isAbsolute()) {
            ComponentPath newPath(pathToPrepend);
            for (int iPath = 0; iPath < (int)path.getNumPathLevels();
                 ++iPath) {
                newPath.pushBack(
                        path.getSubcomponentNameAtLevel(iPath));
            }
            std::string newConnecteePath =
                    AbstractInput::composeConnecteePath(
                            newPath.toString(),
                            outputName, channelName, alias);

            setConnecteePath(newConnecteePath, iConn);
        }
    }

}
