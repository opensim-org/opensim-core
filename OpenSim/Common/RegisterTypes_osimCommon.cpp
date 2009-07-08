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
#include "Transform.h"
#include "VectorGCVSplineR1R3.h"
#include "Scale.h"
#include "NatCubicSpline.h"
#include "Constant.h"
#include "StepFunction.h"
#include "LinearFunction.h"
#include "VisibleObject.h"
#include "ObjectGroup.h"

using namespace OpenSim;
using namespace std;

#ifndef STATIC_OSIM_LIBS
static osimCommonInstantiator instantiator; 
#endif

//_____________________________________________________________________________
/**
 * The purpose of this routine is to register all class types exported by
 * the osimCommon library.
 */
OSIMCOMMON_API void RegisterTypes_osimCommon()
{
	//cout<<"RegisterTypes_osimCommon  \n";

	Object::RegisterType( FunctionSet() );
	Object::RegisterType( GCVSplineSet() );
	Object::RegisterType( ScaleSet() );

	Object::RegisterType( GCVSpline() );
	// Object::RegisterType( Transform() );
	Object::RegisterType( VectorGCVSplineR1R3() );
	//Object::RegisterType( RootSolver() );
	Object::RegisterType( Scale() );
	Object::RegisterType( NatCubicSpline() );
	Object::RegisterType( Constant() );
	Object::RegisterType( StepFunction() );
	Object::RegisterType( LinearFunction() );
	Object::RegisterType( VisibleObject() );
	Object::RegisterType( ObjectGroup() );
}

osimCommonInstantiator::osimCommonInstantiator() 
{ 
        registerDllClasses(); 
} 
    
void osimCommonInstantiator::registerDllClasses() 
{ 
        RegisterTypes_osimCommon(); 
}
