#ifndef __CoordinateSet_h__
#define __CoordinateSet_h__
/* -------------------------------------------------------------------------- *
 *                         OpenSim:  CoordinateSet.h                          *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2012 Stanford University and the Authors                *
 * Author(s): Peter Loan, Ajay Seth                                           *
 *                                                                            *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may    *
 * not use this file except in compliance with the License. You may obtain a  *
 * copy of the License at http://www.apache.org/licenses/LICENSE-2.0.         *
 *                                                                            *
 * Unless required by applicable law or agreed to in writing, software        *
 * distributed under the License is distributed on an "AS IS" BASIS,          *
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.   *
 * See the License for the specific language governing permissions and        *
 * limitations under the License.                                             *
 * -------------------------------------------------------------------------- */

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
     * Populate this flat list of Coordinates given a Model that has already
     * been setup.
     */
	void populate(Model& model);

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
