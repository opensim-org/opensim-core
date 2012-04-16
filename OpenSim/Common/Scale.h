#ifndef _Scale_h_
#define _Scale_h_
// Scale.h
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

/*  
 * Author:  
 */


#include "osimCommonDLL.h"
#include "Object.h"
#include "PropertyStr.h"
#include "PropertyDblArray.h"
#include "PropertyDbl.h"
#include "PropertyBool.h"
#include "PropertyDblVec.h"

//=============================================================================
/*
 * A Class representing scale factors for an object
 *
 * @author Ayman Habib
 * @version 1.0
 */
namespace OpenSim { 

class OSIMCOMMON_API Scale : public Object {
OpenSim_DECLARE_CONCRETE_OBJECT(Scale, Object);

//=============================================================================
// DATA
//=============================================================================
protected:
	// PROPERTIES
	/** A list of 3 scale factors */
	PropertyDblVec3	_propScaleFactors;
	/** Name of object to scale */
	PropertyStr		_propSegmentName;
	/** Whether or not to apply this scale */
	PropertyBool		_propApply;

	// REFERENCES
	SimTK::Vec3&	_scaleFactors;
	std::string&		_segmentName;
	bool&					_apply;

//=============================================================================
// METHODS
//=============================================================================
public:
	//--------------------------------------------------------------------------
	// CONSTRUCTION
	//--------------------------------------------------------------------------
	Scale();
	Scale(const Scale &aMarker);
	Scale( const std::string& scaleFileName);
	virtual ~Scale(void);

	//--------------------------------------------------------------------------
	// OPERATORS
	//--------------------------------------------------------------------------
#ifndef SWIG
	Scale& operator=(const Scale &aMarker);
#endif	
private:
	void setNull();
	void setupProperties();

public:
	//--------------------------------------------------------------------------
	// SET AND GET
	//--------------------------------------------------------------------------
	const std::string& getSegmentName() const;
	void setSegmentName(const std::string& aSegmentName);

	void getScaleFactors(SimTK::Vec3& aScaleFactors) const;
	SimTK::Vec3& getScaleFactors() { return _scaleFactors; }
	void getScaleFactors(double rScaleFactors[]){	// A variant that uses basic types for use by GUI
		getScaleFactors(SimTK::Vec3::updAs(rScaleFactors));
	}

	void setScaleFactors(const SimTK::Vec3& aScaleFactors);
	void setScaleFactors(const double aScaleFactors[]){	// A variant that uses basic types for use by GUI
		setScaleFactors(SimTK::Vec3::getAs(aScaleFactors));
	}

	bool getApply(void) const { return _apply; }
	void setApply(bool state) { _apply = state; }
};

}; //namespace
#endif
