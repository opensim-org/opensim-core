// RegisterTypes_osimCommon.cpp
// author: Frank C. Anderson
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
/*
 * Copyright (c) 2007, Stanford University. All rights reserved. 
 * Redistribution and use in source and binary forms, with or without 
 * modification, are permitted provided that the following conditions
 * are met: 
 *  - Redistributions of source code must retain the above copyright 
 *    notice, this list of conditions and the following disclaimer. 
 *  - Redistributions in binary form must reproduce the above copyright 
 *    notice, this list of conditions and the following disclaimer in the 
 *    documentation and/or other materials provided with the distribution. 
 *  - Neither the name of the Stanford University nor the names of its 
 *    contributors may be used to endorse or promote products derived 
 *    from this software without specific prior written permission. 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS 
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE 
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, 
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; 
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER 
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN 
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE 
 * POSSIBILITY OF SUCH DAMAGE. 
 */


#include "Object.h"
#include "VisibleObject.h"
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
#include "MultiplierFunction.h"
#include "DisplayGeometry.h"
#include "GeometrySet.h"
#include "VisibleObject.h"
#include "ObjectGroup.h"
#include "StateFunction.h"

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
	Object::registerType( MultiplierFunction() );
	Object::registerType( DisplayGeometry() );
	Object::registerType( GeometrySet() );
	Object::registerType( VisibleObject() );
	Object::registerType( ObjectGroup() );

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

