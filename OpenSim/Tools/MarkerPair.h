#ifndef _MarkerPair_h_
#define _MarkerPair_h_

// MarkerPair.h
// Author: Peter Loan
/* Copyright (c)  2005, Stanford University and Peter Loan.
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


// INCLUDE
#include "osimToolsDLL.h"
#include <OpenSim/Common/Object.h>
#include <OpenSim/Common/PropertyStrArray.h>

#include <iostream>
#include <cmath>

//=============================================================================
//=============================================================================
/**
 * A class for holding the names of a pair of markers (for making
 * measurements on a model).
 *
 * @author Peter Loan
 * @version 1.0
 */
namespace OpenSim { 

class OSIMTOOLS_API MarkerPair : public Object {
OpenSim_DECLARE_CONCRETE_OBJECT(MarkerPair, Object);

//=============================================================================
// DATA
//=============================================================================
private:

protected:
	PropertyStrArray _markerNamesProp;
	Array<std::string>& _markerNames;

//=============================================================================
// METHODS
//=============================================================================
	//--------------------------------------------------------------------------
	// CONSTRUCTION
	//--------------------------------------------------------------------------
public:
	MarkerPair();
	MarkerPair(const MarkerPair &aMarkerPair);
	MarkerPair(const std::string &aName1, const std::string &aName2);
	virtual ~MarkerPair();

#ifndef SWIG
	MarkerPair& operator=(const MarkerPair &aMarkerPair);
#endif
	void copyData(const MarkerPair &aMarkerPair);

	void getMarkerNames(std::string& aName1, std::string& aName2) const;
	const std::string &getMarkerName(int i) const { return _markerNames.get(i); }
	void setMarkerName(int i, const std::string &aName) { _markerNames.set(i,aName); }

protected:

private:
	void setNull();
	void setupProperties();
//=============================================================================
};	// END of class MarkerPair

}; //namespace
//=============================================================================
//=============================================================================

#endif // __MarkerPair_h__


