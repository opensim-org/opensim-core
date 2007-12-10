#ifndef _Callback_h_
#define _Callback_h_
// Callback.h
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

class Model;

class OSIMSIMULATION_API Callback : public Object
{

//=============================================================================
// DATA
//=============================================================================
protected:
	/** Model. */
	Model *_model;
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
	Callback(Model *aModel=NULL);
	Callback(const Callback &aCallback);
	Callback(const std::string &aFileName, bool aUpdateFromXMLNode = true);
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
	virtual void setModel(Model *);
	Model* getModel() const;
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


