#ifndef OPENSIM_EXTERNAL_FORCE_H_
#define OPENSIM_EXTERNAL_FORCE_H_
/* -------------------------------------------------------------------------- *
 *                         OpenSim:  ExternalForce.h                          *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2012 Stanford University and the Authors                *
 * Author(s): Ajay Seth                                                       *
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
// INCLUDE
#include "Force.h"

namespace OpenSim {

class Model;
class Storage;
class Function;

/**
 * An ExternalForce is a Force class specialized at applying an external force 
 * and/or torque to a body as described by arrays (columns) of a Storage object.
 * The source of the Storage may be experimental sensor recording or user 
 * generated data. The Storage must be able to supply (1) an array of time, (2) 
 * arrays for the x,y,z, components of force and/or torque in time. Optionaly,
 * (3) arrays for the point of force application in time. An ExternalForce 
 * must specify the identifier (e.g. Force1.x Force1.y Force1.z) for the force
 * components (columns) listed in the Storage either by individual labels or
 * collectively (e.g. as "Force1"). Similarly, identifiers for the applied
 * torque and optionally the point of force application must be specified. 
 *
 * If an identifier is supplied and it cannot uniquely identify the force data 
 * (e.g. the force, torque, or point) in the Storage, then an Excpetion is 
 * thrown.
 *
 * An ExternalForce must apply at least a force or a torque and therefore both 
 * identifiers cannot be empty. 
 *
 * @author Ajay Seth
 */
class OSIMSIMULATION_API ExternalForce : public Force {
OpenSim_DECLARE_CONCRETE_OBJECT(ExternalForce, Force);
public:
//==============================================================================
// PROPERTIES
//==============================================================================
    /** @name Property declarations
    These are the serializable properties associated with this class. **/
    /**@{**/
    OpenSim_DECLARE_PROPERTY(applied_to_body, std::string,
		"Name of the body the force is applied to.");
	OpenSim_DECLARE_PROPERTY(force_expressed_in_body, std::string,
		"Name of the body the force is expressed in (default is ground).");
	OpenSim_DECLARE_OPTIONAL_PROPERTY(point_expressed_in_body, std::string,
		"Name of the body the point is expressed in (default is ground).");
	OpenSim_DECLARE_OPTIONAL_PROPERTY(force_identifier, std::string,
		"Identifier (string) to locate the force to be applied in the data source.");
	OpenSim_DECLARE_OPTIONAL_PROPERTY(point_identifier, std::string,
		"Identifier (string) to locate the point to be applied in the data source.");
	OpenSim_DECLARE_OPTIONAL_PROPERTY(torque_identifier, std::string,
		"Identifier (string) to locate the torque to be applied in the data source.");
	OpenSim_DECLARE_OPTIONAL_PROPERTY(data_source_name, std::string,
		"Name of the data source (Storage) that will supply the force data.");
    /**@}**/


//==============================================================================
// PUBLIC METHODS
//==============================================================================
	/**
	 * Default Construct of an ExternalForce. 
	 * By default ExternalForce has data source identified by name.
	 * By setup() time, Tool or modeler must setDataSource() on this Force for
	 * it to be able to apply any force. Otherwise, an exception is thrown.
	 * 
	 */
	ExternalForce();
	/**
	 * Convenience Constructor of an ExternalForce. 
	 * 
 	 * @param dataSource		a storage containing the pertinent force data through time
	 * @param forceIdentifier   string used to access the force data in the dataSource
	 * @param pointIdentifier   string used to access the point of application of the force in dataSource
	 * @param torqueIdentifier  string used to access the force data in the dataSource
	 * @param appliedToBodyName			string used to specify the body to which the force is applied
	 * @param forceExpressedInBodyName  string used to define in which body the force is expressed
	 * @param pointExpressedInBodyName  string used to define the body in which the point is expressed
	 */
	ExternalForce(const Storage& dataSource, 
                  const std::string& forceIdentifier="force", 
                  const std::string& pointIdentifier="point", 
                  const std::string& torqueIdentifier="torque",
		          const std::string& appliedToBodyName="", 
                  const std::string& forceExpressedInBodyName="ground", 
                  const std::string& pointExpressedInBodyName="ground");
	explicit ExternalForce(SimTK::Xml::Element& aNode);

    // Uses default (compiler-generated) destructor, copy constructor, copy 
    // assignment operator.

    // Copy properties from XML into member variables
	virtual void updateFromXMLNode(SimTK::Xml::Element& aNode, int versionNumber=-1);

	// ACCESS METHODS
	/**
	 *  Associate the data source from which the force, point and/or torque data
	 *  is to be extracted.
	 */
	void setDataSource(const Storage& dataSource);

    /** Get the name of the data source for the force data. **/ 
	const std::string& getDataSourceName() const 
    {   return get_data_source_name(); }

	/**
	 *  Specify or obtain the body to which the force will be applied
	 */
	void setAppliedToBodyName(const std::string& applyToName) 
    {   set_applied_to_body(applyToName); }
	const std::string& getAppliedToBodyName() const 
    {   return get_applied_to_body(); }

	/**
	 *  Specify or obtain the body in which the point of application is expressed
	 */
	void setPointExpressedInBodyName(const std::string& pointInBodyName) 
    {   set_point_expressed_in_body(pointInBodyName); }
	const std::string& getPointExpressedInBodyName() const 
    {   return get_point_expressed_in_body(); }

	/**
	 *  Specify or obtain the body in which the force is expressed
	 */
	void setForceExpressedInBodyName(const std::string &forceInBodyName) 
    {   set_force_expressed_in_body(forceInBodyName); }
	const std::string& getForceExpressedInBodyName() const 
    {   return get_force_expressed_in_body(); }

	/**
	 * Identifiers
	 */
	void setForceIdentifier(const std::string aForceIdentifier) 
    {   set_force_identifier(aForceIdentifier); }
	void setPointIdentifier(const std::string aPointIdentifier) 
    {   set_point_identifier(aPointIdentifier); }
	void setTorqueIdentifier(const std::string aTorqueIdentifier) 
    {   set_torque_identifier(aTorqueIdentifier); }

	const std::string& getForceIdentifier() const 
    {   return get_force_identifier(); }
	const std::string& getPointIdentifier() const 
    {   return get_point_identifier(); }
	const std::string& getTorqueIdentifier() const 
    {   return get_torque_identifier(); }
	/**
	 * Convenience methods to access external forces at a given time
	 */
	SimTK::Vec3 getForceAtTime(double aTime) const;
	SimTK::Vec3 getPointAtTime(double aTime) const;
	SimTK::Vec3 getTorqueAtTime(double aTime) const;

	/**
	 * Methods used for reporting.
	 * First identify the labels for individual components
	 */
	virtual OpenSim::Array<std::string> getRecordLabels() const;
	/**
	 * Given SimTK::State object extract all the values necessary to report 
     * forces, application location frame, etc. used in conjunction with 
     * getRecordLabels and should return same size Array.
	 */
	virtual OpenSim::Array<double> getRecordValues(const SimTK::State& state) const;
	/**
	 * Methods to query the force properties to find out if it's a body vs. 
     * point force and/or if it applies a torque. 
	 */
	bool appliesForce() const { 
		if(getProperty_force_identifier().size() < 1)
			return false;
        const std::string &forceIdentifier = get_force_identifier(); 
        return !((forceIdentifier.find_first_not_of(" \t")==std::string::npos) 
                  || (forceIdentifier == "Unassigned"));
    }
	bool specifiesPoint() const {
		if(getProperty_point_identifier().size() < 1)
			return false;
        const std::string &pointIdentifier = get_point_identifier(); 
        return !((pointIdentifier.find_first_not_of(" \t")==std::string::npos) 
                  || (pointIdentifier == "Unassigned"));
    }
	bool appliesTorque() const {
		if(getProperty_torque_identifier().size() < 1)
			return false;
        const std::string &torqueIdentifier = get_torque_identifier(); 
        return !((torqueIdentifier.find_first_not_of(" \t")==std::string::npos) 
                  || (torqueIdentifier == "Unassigned"));
    }


protected:

	/**  ModelComponent interface */ 
	void connectToModel(Model& model) OVERRIDE_11;

	/**
	 * Compute the force.
	 */
	virtual void computeForce(const SimTK::State& state, 
							  SimTK::Vector_<SimTK::SpatialVec>& bodyForces, 
							  SimTK::Vector& generalizedForces) const;

private:
	void setNull();
	void constructProperties();


//==============================================================================
// DATA
//==============================================================================

	/** Pointer to the body that force is applied to */
	SimTK::ReferencePtr<const Body> _appliedToBody;

	/** Pointer to the body that force is expressed in */
	SimTK::ReferencePtr<const Body> _forceExpressedInBody;

	/** Pointer to the body that point is expressed in */
	SimTK::ReferencePtr<const Body> _pointExpressedInBody;

	/** Pointer to the data source owned by the caller/creator of this force. 
	    Note that it is not a RefPtr because we want to point to the same data
		source when the ExternalForce is copied, without copying the data. */
	const Storage* _dataSource;

	/** characterize the force/torque being applied */
	bool _appliesForce;
	bool _specifiesPoint;
	bool _appliesTorque;

	/** force data as a function of time used internally */
	ArrayPtrs<Function> _forceFunctions;
	ArrayPtrs<Function> _torqueFunctions;
	ArrayPtrs<Function> _pointFunctions;

	friend class ExternalLoads;
//==============================================================================
};	// END of class ExternalForce
//==============================================================================
//==============================================================================

} // end of namespace OpenSim

#endif // OPENSIM_EXTERNAL_FORCE_H_
