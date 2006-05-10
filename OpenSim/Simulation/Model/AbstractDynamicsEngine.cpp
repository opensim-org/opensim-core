// AbstractDynamicsEngine.cpp
// Authors: Frank C. Anderson, Ayman Habib, Peter Loan
/*
 * Copyright (c) 2005, Stanford University. All rights reserved. 
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

//=============================================================================
// INCLUDES
//=============================================================================
#include <iostream>
#include <string>
#include <math.h>
#include <float.h>
#include <OpenSim/Tools/rdMath.h>
#include <OpenSim/Tools/Mtx.h>
#include <OpenSim/Tools/Memory.h>
#include "AbstractDynamicsEngine.h"



using namespace OpenSim;
using namespace std;


//=============================================================================
// CONSTANTS
//=============================================================================



//=============================================================================
// CONSTRUCTOR(S) AND DESTRUCTOR
//=============================================================================
//_____________________________________________________________________________
/**
 * Destructor.
 */
AbstractDynamicsEngine::~AbstractDynamicsEngine()
{
}

//_____________________________________________________________________________
/**
 * Default constructor.
 */
AbstractDynamicsEngine::AbstractDynamicsEngine()
{
	// NULL
	setNull();
}

//_____________________________________________________________________________
/**
 * Constructor from an XML Document
 */
AbstractDynamicsEngine::AbstractDynamicsEngine(const string &aFileName) :
	Object(aFileName)
{
	// NULL
	setNull();

	// DESERIALIZE
	updateFromXMLNode();
}

//_____________________________________________________________________________
/**
 * Constructor from an XML node
 */
AbstractDynamicsEngine::AbstractDynamicsEngine(DOMElement *aElement) :
	Object(aElement)
{
	// NULL
	setNull();

	// DESERIALIZE
	updateFromXMLNode();
}

//_____________________________________________________________________________
/**
 * Copy constructor.
 */
AbstractDynamicsEngine::AbstractDynamicsEngine(const AbstractDynamicsEngine& aKE)
{
	// NULL
	setNull();
}

AbstractDynamicsEngine& AbstractDynamicsEngine::operator=(const AbstractDynamicsEngine &aKE)
{
	// BASE CLASS
	Object::operator=(aKE);

	return(*this);
}

//=============================================================================
// CONSTRUCTION METHODS
//=============================================================================
//_____________________________________________________________________________
/**
 * Set NULL values for all the variable members of this class.
 */
void AbstractDynamicsEngine::setNull()
{
	setupProperties();
	setType("AbstractDynamicsEngine");
}


//=============================================================================
// TYPE REGISTRATION
//=============================================================================
///_____________________________________________________________________________
/**
 * Register the types of objects used by AbstractDynamicsEngine and that need to be registered 
 * for xml serialization purposes
 *
 */
void AbstractDynamicsEngine::setupProperties()
{	
}
