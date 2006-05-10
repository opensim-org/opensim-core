#ifndef _Marker_h_
#define _Marker_h_
// Marker.h
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
#include <OpenSim/Simulation/rdSimulationDLL.h>
#include <OpenSim/Tools/Object.h>
#include <OpenSim/Tools/PropertyStr.h>
#include <OpenSim/Tools/PropertyDblArray.h>
#include <OpenSim/Tools/PropertyDbl.h>
#include <OpenSim/Tools/VisibleObject.h>

//=============================================================================
//=============================================================================
/*
 * A Class representing a marker that lives on a SIMM model
 *
 * @author Ayman Habib
 * @version 1.0
 */
#ifdef SWIG
	#ifdef RDSIMULATION_API
		#undef RDSIMULATION_API
		#define RDSIMULATION_API
	#endif
#endif

namespace OpenSim { 

class RDSIMULATION_API Marker : public VisibleObject
{
protected:
	// PROPERTIES
	/** Location of marker in respective segment/body */
	PropertyDblArray	_propMarkerLocation;
	/** Name of Segment/Body on which marker lives */
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
	Marker();
	Marker(const Marker &aMarker);
	Marker(DOMElement *aElement);
	virtual ~Marker(void);
	virtual Object* copy(DOMElement *aElement) const;
	virtual Object* copy() const;

	//--------------------------------------------------------------------------
	// OPERATORS
	//--------------------------------------------------------------------------
#ifndef SWIG
	Marker& operator=(const Marker &aMarker);
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
