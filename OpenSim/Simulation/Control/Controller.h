#ifndef _Controller_h_
#define _Controller_h_
// Controller.h
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
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

/* Note: This code was originally developed by Realistic Dynamics Inc. 
 * Author: Frank C. Anderson 
 */
//============================================================================


#include "ControlSet.h"


const int ControllerNAME_LENGTH = 64;
const int ControllerDESCRIP_LENGTH = 8192;


//=============================================================================
//=============================================================================
/**
 * An abstract class for specifying the interface for controlling an
 * Model.
 *
 * @author Frank C. Anderson
 * @version 1.0
 */
namespace OpenSim { 

class AbstractModel;

class OSIMSIMULATION_API Controller  
{

//=============================================================================
// DATA
//=============================================================================
public:
	static const int NAME_LENGTH;
	static const int DESCRIP_LENGTH;
protected:
	/** Model. */
	AbstractModel *_model;
	/** Flag to indicate on or off state. */
	bool _on;
	/** Name. */
	char _name[ControllerNAME_LENGTH];
	/** Description. */
	char _descrip[ControllerDESCRIP_LENGTH];

//=============================================================================
// METHODS
//=============================================================================
	//--------------------------------------------------------------------------
	// CONSTRUCTION
	//--------------------------------------------------------------------------
public:
	Controller(AbstractModel *aModel);
	virtual ~Controller();
	void setNull();

	//--------------------------------------------------------------------------
	// GET AND SET
	//--------------------------------------------------------------------------
	// MODEL
	AbstractModel* getModel();
	// ON/OFF
	void setOn(bool aTrueFalse);
	bool getOn();
	// NAME
	void setName(const char *aName);
	const char* getName();
	// DESCRIPTION
	void setDescription(const char *aDescrip);
	const char* getDescription();

	//--------------------------------------------------------------------------
	// CONTROL
	//--------------------------------------------------------------------------
	virtual void
		computeControls(double &rDT,double aT,const double *aY,
		ControlSet &rX) = 0;

//=============================================================================
};	// END of class Controller

}; //namespace
//=============================================================================
//=============================================================================

#endif // __Controller_h__


