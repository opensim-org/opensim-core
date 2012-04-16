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

#include <string>
#include <iostream>
#include "Object.h"
#include "VisibleObject.h"
#include "RegisterTypes_osimCommon.h"
#include "FunctionSet.h"
#include "GCVSplineSet.h"
#include "ScaleSet.h"
#include "GCVSpline.h"
#include "VectorGCVSplineR1R3.h"
#include "Scale.h"
#include "NaturalCubicSpline.h"
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

using namespace OpenSim;
using namespace std;


//_____________________________________________________________________________
/**
 * The purpose of this routine is to register all class types exported by
 * the osimCommon library.
 */
OSIMCOMMON_API void RegisterTypes_osimCommon()
{
	//cout<<"RegisterTypes_osimCommon  \n";

	Object::registerType( FunctionSet() );
	Object::registerType( GCVSplineSet() );
	Object::registerType( ScaleSet() );

	Object::registerType( GCVSpline() );
	Object::registerType( VectorGCVSplineR1R3() );
	Object::registerType( Scale() );
	Object::registerType( NaturalCubicSpline() );
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

	// To support old type name of "natCubicSpline"
	Object::renameType("natCubicSpline", "NaturalCubicSpline");
}

