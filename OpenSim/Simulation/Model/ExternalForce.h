#ifndef OPENSIM_EXTERNAL_FORCE_H_
#define OPENSIM_EXTERNAL_FORCE_H_
// ExternalForce.h
// Author: Ajay Seth
/*
 * Copyright (c)  2011, Stanford University. All rights reserved. 
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
#include "OpenSim/Common/PropertyStr.h"
#include "Force.h"

namespace OpenSim {

class Model;
class Storage;
class Function;

/**
 * This applies a force and/or torque to a body according describe by arrays contained in a Storage
 * The source of the Storage may be experimental sensor recording or user generated data.
 * The Storage must be able to supply (1) array of time, (3) arrays for the x,y,z, components of  
 * force and/or torque in time.  Optionaly (3) arrays for the point of force application in time.
 * This Force must identify a force identifier (e.g. Force1.x Force1.y Force1.y) may be individual
 * labels for force components but they are collectively identified (as "Force1"). Similarly,  
 * identifiers for torque and point are expected. 
 *
 * If an identifier is supplied and it cannot uniquely identify the quantity (force, torque, point) in 
 * the Storage an Excpetion is thrown.
 *
 * An ExternalForce must apply either a force or a torque so both identifiers cannot be empty. 
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
    OpenSim_DECLARE_OPTIONAL_PROPERTY(applied_to_body, std::string,
		"Name of the body the force is applied to.");
	OpenSim_DECLARE_OPTIONAL_PROPERTY(force_expressed_in_body, std::string,
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
	 * @param pointExpressedInBodyName  string used to define the body in which the the point is expressed
	 */
	ExternalForce(const Storage& dataSource, 
                  const std::string& forceIdentifier="force", 
                  const std::string& PointIdentifier="point", 
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
	void setDataSource(const Storage *dataSource);

    /** Get the name of the data source for the force data. **/ 
	const std::string& getDataSourceName() const 
    {   return get_data_source_name(); }

	/**
	 *  Specify or obtain the body to which the force will be applied
	 */
	void setAppliedToBodyName(const std::string &applyToName) 
    {   set_applied_to_body(applyToName); }
	const std::string& getAppliedToBodyName() const 
    {   return get_applied_to_body(); }

	/**
	 *  Specify or obtain the body in which the point of application is expressed
	 */
	void setPointExpressedInBodyName(const std::string &pointInBodyName) 
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
        const std::string &forceIdentifier = get_force_identifier(); 
        return !((forceIdentifier.find_first_not_of(" \t")==std::string::npos) 
                  || (forceIdentifier == "Unassigned"));
    }
	bool specifiesPoint() const { 
        const std::string &pointIdentifier = get_point_identifier(); 
        return !((pointIdentifier.find_first_not_of(" \t")==std::string::npos) 
                  || (pointIdentifier == "Unassigned"));
    }
	bool appliesTorque() const { 
        const std::string &torqueIdentifier = get_torque_identifier(); 
        return !((torqueIdentifier.find_first_not_of(" \t")==std::string::npos) 
                  || (torqueIdentifier == "Unassigned"));
    }


protected:

	/**  ModelComponent interface */ 
	virtual void setup(Model& model);
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
	Body *_appliedToBody;

	/** Pointer to the body that force is expressed in */
	Body *_forceExpressedInBody;

	/** Pointer to the body that point is expressed in */
	Body *_pointExpressedInBody;

	/** Pointer to the data source owned by the caller/creator of this force */
	const Storage *_dataSource;

	/** characterize the force/torque being applied */
	bool _appliesForce;
	bool _specifiesPoint;
	bool _appliesTorque;

	/** force data as a function of time used internally */
	SimTK::Array_<Function*> _forceFunctions;
	SimTK::Array_<Function*> _torqueFunctions;
	SimTK::Array_<Function*> _pointFunctions;

	friend class ExternalLoads;
//==============================================================================
};	// END of class ExternalForce
//==============================================================================
//==============================================================================

} // end of namespace OpenSim

#endif // OPENSIM_EXTERNAL_FORCE_H_
