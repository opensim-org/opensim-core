#ifndef _Controller_h_
#define _Controller_h_
// Controller.h
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
/*
* Copyright (c)  2005, Stanford University. All rights reserved. 
* Use of the OpenSim software in source form is permitted provided that the following
* conditions are met:
* 	1. The software is used only for non-commercial research and education. It may not
*     be used in relation to any commercial activity.
* 	2. The software is not distributed or redistributed.  Software distribution is allowed 
*     only through https://simtk.org/home/opensim.
* 	3. Use of the OpenSim software or derivatives must be acknowledged in all publications,
*      presentations, or documents describing work in which OpenSim or derivatives are used.
* 	4. Credits to developers may not be removed from executables
*     created from modifications of the source.
* 	5. Modifications of source code must retain the above copyright notice, this list of
*     conditions and the following disclaimer. 
* 
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY
*  EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
*  OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT
*  SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
*  TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; 
*  HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
*  OR BUSINESS INTERRUPTION) OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY
*  WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
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

class Model;

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
	Model *_model;
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
	Controller(Model *aModel);
	virtual ~Controller();
	void setNull();

	//--------------------------------------------------------------------------
	// GET AND SET
	//--------------------------------------------------------------------------
	// MODEL
	Model* getModel();
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


