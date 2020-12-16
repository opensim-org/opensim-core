/* -------------------------------------------------------------------------- *
 *                   OpenSim:  RegisterTypes_osimCommon.cpp                   *
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


#include "Object.h"
#include "Component.h"
#include "RegisterTypes_osimCommon.h"
#include "FunctionSet.h"
#include "GCVSplineSet.h"
#include "ScaleSet.h"
#include "GCVSpline.h"

#include "Scale.h"
#include "SimmSpline.h"
#include "Constant.h"
#include "Sine.h"
#include "StepFunction.h"
#include "LinearFunction.h"
#include "PiecewiseLinearFunction.h"
#include "PiecewiseConstantFunction.h"
#include "MultiplierFunction.h"
#include "PolynomialFunction.h"
#include "MultivariatePolynomialFunction.h"

#include "SignalGenerator.h"

#include "ObjectGroup.h"

#include "Reporter.h"
#include "TableSource.h"

#include "ModelDisplayHints.h"
#include "ExperimentalSensor.h"
#include "XsensDataReaderSettings.h"

#include <string>
#include <iostream>
#include <exception>

using namespace OpenSim;
using namespace std;


//_____________________________________________________________________________
/**
 * The purpose of this routine is to register all class types exported by
 * the osimCommon library.
 */
OSIMCOMMON_API void RegisterTypes_osimCommon()
{
  try {

    //SimTK::Xml::setXmlCondenseWhiteSpace(false);
    Object::registerType( FunctionSet() );
    Object::registerType( GCVSplineSet() );
    Object::registerType( ScaleSet() );

    Object::registerType( GCVSpline() );

    Object::registerType( Scale() );
    Object::registerType( SimmSpline() );
    Object::registerType( Constant() );
    Object::registerType( Sine() );
    Object::registerType( StepFunction() );
    Object::registerType( LinearFunction() );
    Object::registerType( PiecewiseLinearFunction() );
    Object::registerType( PiecewiseConstantFunction() );
    Object::registerType( MultiplierFunction() );
    Object::registerType( PolynomialFunction() );
    Object::registerType( MultivariatePolynomialFunction() );

    Object::registerType( SignalGenerator() );

    Object::registerType( ObjectGroup() );
    
    Object::registerType( TableSource() );
    Object::registerType( TableSourceVec3() );
    Object::registerType( TableReporter() );
    Object::registerType( TableReporterVec3() );
    Object::registerType( TableReporterVector() );
    Object::registerType( ConsoleReporter() );
    Object::registerType( ConsoleReporterVec3() );

    Object::registerType(ModelDisplayHints());
    Object::registerType(ExperimentalSensor());
    Object::registerType(XsensDataReaderSettings());

    // TODO: temporarily map old NaturalCubicSpline (which wasn't a
    // natural cubic spline) to renamed SimmSpline class. Later we
    // will replace this with an actual natural cubic spline.
    Object::renameType("NaturalCubicSpline", "SimmSpline");
    // To support older type name of "natCubicSpline"
    Object::renameType("natCubicSpline", "SimmSpline");

  } catch (const std::exception& e) {
    std::cerr 
        << "ERROR during osimCommon Object registration:\n"
        << e.what() << "\n";
  }
}

