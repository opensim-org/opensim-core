#ifndef __CoordinateSet_h__
#define __CoordinateSet_h__

// CoordinateSet.h
// Author: Peter Loan
/*
 * Copyright (c)  2006, Stanford University. All rights reserved. 
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

#include <OpenSim/Simulation/osimSimulationDLL.h>
#include <OpenSim/Simulation/Model/ModelComponentSet.h>
#include <OpenSim/Simulation/SimbodyEngine/Coordinate.h>


namespace OpenSim {

class Model;
//=============================================================================
//=============================================================================
/**
 * A class for holding a set of coordinates.
 *
 * @authors Peter Loan, Ajay Seth
 * @version 2.0
 */

class OSIMSIMULATION_API CoordinateSet : public ModelComponentSet<Coordinate> {
OpenSim_DECLARE_CONCRETE_OBJECT(CoordinateSet, ModelComponentSet<Coordinate>);

private:
	void setNull();
public:
	CoordinateSet();
	CoordinateSet(Model& model) : Super(model) {}
	CoordinateSet(Model& model, const std::string &aFileName, 
                  bool aUpdateFromXMLNode=true)
    :   Super(model, aFileName, aUpdateFromXMLNode) {}
	CoordinateSet(const CoordinateSet& aCoordinateSet);
	~CoordinateSet(void);

	/**
     * Populate this flat list of Coordinates given a Model that has been setup
     */
	void populate(Model& model);

	/** Perform any setup on all of the Coordinates contained in this set. */
	virtual void setup(Model& model);
	//--------------------------------------------------------------------------
	// OPERATORS
	//--------------------------------------------------------------------------
#ifndef SWIG
	CoordinateSet& operator=(const CoordinateSet &aCoordinateSet);
#endif
	void getSpeedNames(OpenSim::Array<std::string> &rNames ) const
{
	for(int i=0;i<_objects.getSize();i++) {
		Coordinate *obj = _objects[i];
		if(obj==NULL) {
			rNames.append("NULL");
		} else {
			rNames.append(obj->getSpeedName());
		}
	}
}
//=============================================================================
};	// END of class CoordinateSet
//=============================================================================
//=============================================================================

} // end of namespace OpenSim

#endif // __CoordinateSet_h__
