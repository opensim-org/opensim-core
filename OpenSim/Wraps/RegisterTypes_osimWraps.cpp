/* -------------------------------------------------------------------------- *
 *                   OpenSim:  RegisterTypes_osimWraps.cpp                    *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2017 Stanford University and the Authors                *
 * Author(s): Frank C. Anderson                                               *
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

#include <string>
#include <iostream>
#include <OpenSim/Common/Object.h>
#include "RegisterTypes_osimWraps.h"

#include "WrapEllipsoid.h"
#include "PathWrap.h"
#include "PathWrapSet.h"
#include "WrapCylinder.h"
#include "WrapSphere.h"
#include "WrapTorus.h"
#include "WrapObjectSet.h"
#include "WrapCylinderObst.h"
#include "WrapSphereObst.h"
#include "WrapDoubleCylinderObst.h"

using namespace OpenSim;
using namespace std;

static osimWrapsInstantiator instantiator;


//_____________________________________________________________________________
/**
 * The purpose of this routine is to register all class types exported by
 * the Simulation library.
 */
OSIMWRAPS_API void RegisterTypes_osimWraps()
{
  try {

        Object::registerType( PathWrap() );
        Object::registerType( PathWrapSet() );
        Object::registerType( WrapCylinder() );
        Object::registerType( WrapSphere() );
        Object::RegisterType( WrapEllipsoid());
        Object::registerType( WrapTorus() );
        Object::registerType( WrapObjectSet() );
        Object::registerType( WrapCylinderObst() );
        Object::registerType( WrapSphereObst() );
        Object::registerType( WrapDoubleCylinderObst() );


  } catch (const std::exception& e) {
    std::cerr 
        << "ERROR during osimWraps Object registration:\n"
        << e.what() << "\n";
  }
}

osimWrapsInstantiator::osimWrapsInstantiator()
{
       registerDllClasses();
}

void osimWrapsInstantiator::registerDllClasses()
{
       RegisterTypes_osimWraps();
}
