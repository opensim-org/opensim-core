/* -------------------------------------------------------------------------- *
 *                          OpenSim:  AbstractPath.cpp                        *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2023 Stanford University and the Authors                *
 * Author(s): Nicholas Bianco, Joris Verhagen, Adam Kewley                    *
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

#include "AbstractPath.h"
#include "Appearance.h"

using namespace OpenSim;

AbstractPath::AbstractPath() : ModelComponent() {
    setAuthors("Nicholas Bianco, Joris Verhagen, Adam Kewley");

    Appearance appearance;
    appearance.set_color(SimTK::Gray);
    constructProperty_Appearance(appearance);
}

AbstractPath::AbstractPath(AbstractPath const&) = default;

AbstractPath::~AbstractPath() noexcept = default;

AbstractPath& AbstractPath::operator=(const AbstractPath&) = default;

// DEFAULTED METHODS
const SimTK::Vec3& AbstractPath::getDefaultColor() const
{
    return get_Appearance().get_color();
}

void AbstractPath::setDefaultColor(const SimTK::Vec3& color)
{
    updProperty_Appearance().setValueIsDefault(false);
    upd_Appearance().set_color(color);
}

double AbstractPath::getPreScaleLength(const SimTK::State&) const
{
    return _preScaleLength;
}

void AbstractPath::setPreScaleLength(const SimTK::State&,
        double preScaleLength)
{
    _preScaleLength = preScaleLength;
}