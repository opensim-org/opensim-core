#ifndef _Callback_h_
#define _Callback_h_
// Callback.h
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


// INCLUDES
#include <OpenSim/Simulation/osimSimulationDLL.h>
#include <OpenSim/Common/Object.h>

#include <OpenSim/Common/PropertyBool.h>
#include <OpenSim/Common/PropertyDbl.h>

//=============================================================================
//=============================================================================
/**
 * An abstract class for specifying a callback.
 *
 * Callbacks provide low-level access to aspects of an integration.
 *
 * @author Frank C. Anderson
 * @version 1.0
 */
namespace OpenSim { 

class AbstractModel;

class OSIMSIMULATION_API Callback : public Object
{

//=============================================================================
// DATA
//=============================================================================
protected:
	/** Model. */
	AbstractModel *_model;
private:
	/** On, off flag. */
	PropertyBool _onProp;
	bool &_on;
	/** Start time for the callback in normalized time. */
	PropertyDbl _startTimeProp;
	double &_startTime;
	/** End time for the callback in normalized time. */
	PropertyDbl _endTimeProp;
	double &_endTime;

//=============================================================================
// METHODS
//=============================================================================
	//--------------------------------------------------------------------------
	// CONSTRUCTION
	//--------------------------------------------------------------------------
public:
	Callback(AbstractModel *aModel=NULL);
	Callback(const Callback &aCallback);
	Callback(const std::string &aFileName);
	virtual ~Callback();
	virtual Object* copy() const;
	//--------------------------------------------------------------------------
	// OPERATORS
	//--------------------------------------------------------------------------
#ifndef SWIG
	Callback& operator=(const Callback &aCallback);
#endif
private:
	void setNull();
	void setupProperties();

	//--------------------------------------------------------------------------
	// GET AND SET
	//--------------------------------------------------------------------------
public:
	// MODEL
	virtual void setModel(AbstractModel *);
	AbstractModel* getModel() const;
	// ON,OFF
	void setOn(bool aTrueFalse);
	bool getOn() const;
	// START,END
	void setStartTime(double aStartTime);
	double getStartTime() const;
	void setEndTime(double aEndTime);
	double getEndTime() const;

//=============================================================================
};	// END of class Callback

}; //namespace
//=============================================================================
//=============================================================================

#endif // __Callback_h__


