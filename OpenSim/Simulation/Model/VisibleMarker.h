#ifndef _VisibleMarker_h_
#define _VisibleMarker_h_
// VisibleMarker.h
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

/*  
 * Author:  
 */
#include <OpenSim/Simulation/osimSimulationDLL.h>
#include <OpenSim/Common/Object.h>
#include <OpenSim/Common/PropertyStr.h>
#include <OpenSim/Common/PropertyDblArray.h>
#include <OpenSim/Common/PropertyDbl.h>
#include <OpenSim/Common/VisibleObject.h>

//=============================================================================
//=============================================================================
/*
 * A Class representing a marker that lives on a SIMM model
 *
 * @author Ayman Habib
 * @version 1.0
 */
#ifdef SWIG
	#ifdef OSIMSIMULATION_API
		#undef OSIMSIMULATION_API
		#define OSIMSIMULATION_API
	#endif
#endif

namespace OpenSim { 

class OSIMSIMULATION_API VisibleMarker : public VisibleObject
{
protected:
	// PROPERTIES
	/** Location of marker in respective segment/body */
	PropertyDblArray	_propMarkerLocation;
	/** Name of Segment/VisibleBody on which marker lives */
	PropertyStr		_propReferenceSegmentName;
	/** Weight of marker to be used by Inverse Kinematics */
	PropertyDbl		_propMarkerWeight;

	// REFERENCES
	Array<double>&	_markerLocation;
	std::string&		_referenceSegmentName;
	double&				_markerWeight;

private:
	// Reference body/segment for the marker. Saved as an index since SDFast (and current model API) is index based
	int	_refSegmentForMarker;
public:
//=============================================================================
// METHODS
//=============================================================================
	//--------------------------------------------------------------------------
	// CONSTRUCTION
	//--------------------------------------------------------------------------
	VisibleMarker();
	VisibleMarker(const VisibleMarker &aMarker);
	virtual ~VisibleMarker(void);
	virtual Object* copy() const;

	//--------------------------------------------------------------------------
	// OPERATORS
	//--------------------------------------------------------------------------
#ifndef SWIG
	VisibleMarker& operator=(const VisibleMarker &aMarker);
#endif	
private:
	void setNull();
	void setupProperties();

public:
	//--------------------------------------------------------------------------
	// GET AND SET
	//--------------------------------------------------------------------------
	const double getWeight() const;
	void setWeight(const double aWeight);
	const std::string& getReferenceSegmentName() const;
	const int getRefSegment() const;
	void setRefSegment(const int aBodyIndex);
	void getLocation(Array<double>& aLocation) const;
	void setLocation(Array<double>& aLocation);
	// Scaling support 
	void scaleBy(Array<double>& aScales);
	virtual void update(const Object& aObject, Event& aEvent);

};

}; //namespace
#endif
