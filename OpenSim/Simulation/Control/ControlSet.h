#ifndef _ControlSet_h_
#define _ControlSet_h_
// ControlSet.h
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
#include "Control.h"
#include <OpenSim/Common/Set.h>
#include <OpenSim/Common/Storage.h>



//=============================================================================
//=============================================================================
/**
 * A class for holding and managing a set of controls for a dynamic
 * simulation.
 *
 * @author Frank C. Anderson
 * @version 1.0
 */
namespace OpenSim { 

class OSIMSIMULATION_API ControlSet : public Set<Control>
{

//=============================================================================
// DATA
//=============================================================================
protected:
	/** Map from parameters to controls. */
	Array<int> _ptcMap;
	/** Map from set parameters to control parameters. */
	Array<int> _ptpMap;


//=============================================================================
// METHODS
//=============================================================================
	//--------------------------------------------------------------------------
	// CONSTRUCTION
	//--------------------------------------------------------------------------
public:
	ControlSet();
	ControlSet(const std::string &aFileName);
	ControlSet(const ControlSet &aSet);
	virtual ~ControlSet();
	virtual Object* copy() const;
private:
	void setNull();
	void setupProperties();
	

	//--------------------------------------------------------------------------
	// OPERATORS
	//--------------------------------------------------------------------------
public:
#ifndef SWIG
	ControlSet& operator=(const ControlSet &aSet);
#endif
	//--------------------------------------------------------------------------
	// GET AND SET
	//--------------------------------------------------------------------------
	// SIZE
	int getSize(bool aForModelControls=true) const;
	// CONTROL LIST
	//int getControlIndex(const char *aName) const;
	void getControlList(const char *aType,Array<int> &rList,
		bool aForModelControls=true) const;
	// CONTROL VALUES
	void getControlValues(double aT,double rX[],
			bool aForModelControls=true) const;
	void getControlValues(double aT,Array<double> &rX,
			bool aForModelControls=true) const;
	void setControlValues(double aT,const double aX[],
			bool aForModelControls=true);
	void setControlValues(double aT,const Array<double> &aX,
			bool aForModelControls=true);
	// PARAMETERS
	int getNumParameters(bool aForModelControls=true) const;
	void getParameterList(Array<int> &rList,
			bool aForModelControls=true) const;
	void getParameterList(double aT,Array<int> &rList,
			bool aForModelControls=true) const;
	void getParameterList(double aTLower,double aTUpper,Array<int> &rList,
			bool aForModelControls=true) const;
	void getParameterMins(Array<double> &rMins,
			const Array<int> *aList=NULL) const;
	void getParameterMaxs(Array<double> &rMaxs,
			const Array<int> *aList=NULL) const;
	void getParameterValues(double rP[],
			const Array<int> *aList=NULL) const;
	void getParameterValues(Array<double> &rP,
			const Array<int> *aList=NULL) const;
	void setParameterValues(const double aP[],
			const Array<int> *aList=NULL);
	void setParameterValues(const Array<double> &aP,
			const Array<int> *aList=NULL);

	//--------------------------------------------------------------------------
	// UTILITY
	//--------------------------------------------------------------------------
	void simplify(const PropertySet &aProperties);
	void filter(double aT);
	Storage*
		constructStorage(int aN,double aT1,double aT2,bool aForModelControls);
	int mapParameterToControl(int aIndex) const;
	int mapParameterToParameter(int aIndex) const;
	void generateParameterMaps();

//=============================================================================
};	// END of class ControlSet

}; //namespace
//=============================================================================
//=============================================================================


#endif // __ControlSet_h__


