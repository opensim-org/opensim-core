#ifndef OPENSIM_EXTERNAL_LOADS_H_
#define OPENSIM_EXTERNAL_LOADS_H_
/* -------------------------------------------------------------------------- *
 *                         OpenSim:  ExternalLoads.h                          *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2017 Stanford University and the Authors                *
 * Author(s): Ajay Seth, Ayman Habib                                          *
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


// INCLUDES
#include "ModelComponentSet.h"
#include "ExternalForce.h"
#include "OpenSim/Common/PropertyStr.h"
#include "OpenSim/Common/PropertyDbl.h"

namespace OpenSim {

class Model;

//=============================================================================
//=============================================================================
/**
 * A convenience class for managing ExternaForce(s) to be applied to a model.
 * This includes creating instances and manipulating the data source
 * of individual ExternalForces so that they satisfy conditions imposed
 * by particular Tools. For example, ForwardTool, CMC/RRA, achieve better
 * tracking (slower divergence) if the ground reaction forces are applied
 * to a point that is expressed in the foot frame according to "ideal"
 * kinematics. ExternalLoads provides convenience methods to perform this
 * "mapping" which is beyond the scope of an individual ExternalForce, but is
 * too much detail to have each Tool implement.
 *
 * An individual ExternalForce has a property for its data source name, but 
 * under the management of ExternalLoads, the data source identified by
 * ExternalLoads is used to set the data source on each ExternalForce. 
 * If multiple data sources are required for different groups of external forces
 * then use multiple ExternalLoads.
 *
 * @authors Ajay Seth, Ayman Habib 
 */

//=============================================================================
class OSIMSIMULATION_API ExternalLoads 
:   public ModelComponentSet<ExternalForce> {
OpenSim_DECLARE_CONCRETE_OBJECT(ExternalLoads, 
                                ModelComponentSet<ExternalForce>);

//=============================================================================
// DATA
//=============================================================================
protected:
    /** Data source for all forces in this ExternalLoads, where individual 
     *  external forces identify which subsets of the data they will access.*/
    PropertyStr _dataFileNameProp;
    std::string &_dataFileName;

private:
    /* If point of applications for external forces must be re-expressed
       then build new storages to be assigned to the individual ExternalForces
       with the transformed point data. Hang-on to them so we can delete them. */
    std::vector<std::shared_ptr<Storage>> _storages;

    // TODO: Replace with a Path property type that remembers where a file
    // was loaded from.
    std::string _loadedFromFile;

//=============================================================================
// METHODS
//=============================================================================
    //--------------------------------------------------------------------------
    // CONSTRUCTION
    //--------------------------------------------------------------------------
public:
    ExternalLoads();

    /**  Construct an actuator set from file.
    * @param fileName Name of the file.
    * @param aUpdateFromXMLNode Should the ExternalLoads be updated from the
    * file? */
    ExternalLoads(const std::string &fileName, bool aUpdateFromXMLNode);

    ExternalLoads(const ExternalLoads &aExternalLoads);
    virtual ~ExternalLoads();

    void copyData(const ExternalLoads &otherExternalLoads);

    /** Override of the default implementation to account for versioning. */
    void updateFromXMLNode(SimTK::Xml::Element& aNode, int versionNumber=-1) override;

    // Connect all ExternalForces inside this ExternalLoads collection to
    // their Model. Overrides ModelComponentSet method.
    void extendConnectToModel(Model& aModel) override;

    const std::string& getDataFileName() const { return _dataFileName;};
    void setDataFileName(const std::string& aNewFile) { _dataFileName = aNewFile; };

    void transformPointsExpressedInGroundToAppliedBodies(const Storage &kinematics, double startTime = -SimTK::Infinity, double endTime = SimTK::Infinity);
    ExternalForce* transformPointExpressedInGroundToAppliedBody(const ExternalForce &exForce, const Storage &kinematics, double startTime, double endTime);

    /// ExternalLoads remembers the file it was loaded from, even after being
    /// copied. This file path is used to find the datafile relative to the
    /// location of the ExternalLoads file itself. This function can clear
    /// the memory of the file that the original ExternalLoads came from.
    /// In general, users should not need to use this function.
    void clearLoadedFromFile() { _loadedFromFile = ""; }

private:
    void setNull();
    void setupSerializedMembers();
    std::string createIdentifier(OpenSim::Array<std::string>&oldFunctionNames, const Array<std::string>& labels);

    //--------------------------------------------------------------------------
    // OPERATORS
    //--------------------------------------------------------------------------
public:
#ifndef SWIG
    ExternalLoads& operator=(const ExternalLoads &otherExternalLoads);
#endif


//=============================================================================
};  // END of class ExternalLoads
//=============================================================================
//=============================================================================

} // end of namespace OpenSim


#endif // OPENSIM_EXTERNAL_LOADS_H_


