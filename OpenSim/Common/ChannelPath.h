#ifndef OPENSIM_CHANNEL_PATH_H_
#define OPENSIM_CHANNEL_PATH_H_
/* -------------------------------------------------------------------------- *
 *                      OpenSim: ChannelPath.h                                *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2016 Stanford University and the Authors                *
 * Author(s): Chris Dembia                                                    *
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

#include "ComponentPath.h"
#include <iostream> // TODO remove.

namespace OpenSim {

/** TODO document the syntax (copy from parseConnecteeName()). 
 TODO rename to OutputPath? yes... don't need another "C" word.
 */

class OSIMCOMMON_API ChannelPath {

public:
    ChannelPath() = default;
    
    ChannelPath(const std::string& path) {
        // TODO error checking.
        
        // TODO require an output name.
        // TODO output name cannot be empty.
        //std::cout << "DEBUG ChannelPath constructor " << path << std::endl;
        if (path.empty()) return;
        
        auto lastSlash = path.rfind("/");
        auto colon = path.rfind(":");
        auto leftParen = path.rfind("(");
        auto rightParen = path.rfind(")");
        
        _componentPath = ComponentPath(path.substr(0, lastSlash));
        _outputName = path.substr(lastSlash + 1,
                                  std::min(colon, leftParen) - lastSlash - 1);
        
        // Channel name.
        if (colon != std::string::npos) {
            _channelName = path.substr(colon + 1, leftParen - (colon + 1));
        }
        
        // Alias.
        if (leftParen != std::string::npos && rightParen != std::string::npos) {
            _alias = path.substr(leftParen + 1, rightParen - (leftParen + 1));
        }
        
        /*std::cout << "DEBUG ChannelPath constructor " << path << " ("
                  << "compPath:" << _componentPath << " "
                  << "outputName:" << _outputName << " "
                  << "_channelName:" << _channelName << " "
                  << "_alias:" << _alias << std::endl; */
        // TODO clean up naming confusion on Channel's name.
    }
    
    ChannelPath(const ComponentPath& componentPath,
                const std::string& outputName,
                const std::string& channelName = {},
                const std::string& alias = {}) :
        _componentPath(componentPath), _outputName(outputName),
        _channelName(channelName), _alias(alias) {}

    // Operators
    bool operator==(const ChannelPath& other) const
    {   return this->toString() == other.toString(); }

    bool operator!=(const ChannelPath& other) const
    {   return !operator==(other); }
    
    // TODO explain why this takes a ComponentPath, not ChannelPath.
    ChannelPath formRelativePath(const ComponentPath& otherPath) const {
        ChannelPath output(*this);
        output.setComponentPath(_componentPath.formRelativePath(otherPath));
        return output;
    }
    
    /// @name Accessors
    /// @{
    const ComponentPath& getComponentPath() const { return _componentPath; }
    const std::string& getOutputName() const { return _outputName; }
    const std::string& getChannelName() const { return _channelName; }
    const std::string& getAlias() const { return _alias; }
    ComponentPath& updComponentPath() { return _componentPath; }
    std::string& updOutputName() { return _outputName; }
    std::string& updChannelName() { return _channelName; }
    std::string& updAlias() { return _alias; }
    void setComponentPath(const ComponentPath& componentPath)
    {   _componentPath = componentPath; }
    void setOutputName(const std::string& outputName)
    {   _outputName = outputName; /* TODO cannot be empty */ }
    void setChannelName(const std::string& channelName)
    {   _channelName = channelName; }
    void setAlias(const std::string& alias) { _alias = alias; }
    /// @}
    
    
    std::string toString() const {
        std::string path = _componentPath.toString();
        if (!path.empty()) path += "/";
        path += _outputName; // TODO what if output name is empty? error?
        if (!_channelName.empty()) path += ":" + _channelName;
        if (!_alias.empty())       path += "(" + _alias + ")";
        return path;
    }
    
private:
    ComponentPath _componentPath;
    std::string _outputName;
    std::string _channelName;
    std::string _alias;
};

inline std::istream& operator>>(std::istream& in, ChannelPath& out) {
    std::string path;
    in >> path;
    out = ChannelPath(path);
    // TODO handle exceptions from ChannelPath constructor; set failbit, etc.
    return in;
}

inline std::ostream& operator<<(std::ostream& out, const ChannelPath& path) {
    out << path.toString();
    return out;
}

} // end of namespace OpenSim
#endif // OPENSIM_CHANNEL_PATH_H_
