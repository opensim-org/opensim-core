#ifndef __ExternalForce_h__
#define __ExternalForce_h__
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
class OSIMSIMULATION_API ExternalForce : public Force
{

//=============================================================================
// DATA
//=============================================================================
protected:
	/** The body that the external force(torque) is applied to. */
	PropertyStr _appliedToBodyNameProp;
	std::string& _appliedToBodyName;

	/** The body (reference frame) that the external force(torque) is expressed in. */
	PropertyStr _forceExpressedInBodyNameProp;
	std::string& _forceExpressedInBodyName;

	/** The body (reference frame) that the point of force application is expressed. */
	PropertyStr _pointExpressedInBodyNameProp;
	std::string& _pointExpressedInBodyName;

	/** The name of the data source (Storage) that contains the force as data points in time. */
	PropertyStr _dataSourceNameProp;
	std::string& _dataSourceName;

	/** The identifier (as part of its label) of the force data in the dataSource necessary to retrieve 
	    the force data from the dataSource. */
	PropertyStr _forceIdentifierProp;
	std::string& _forceIdentifier;

	/** The identifier of the point data in the dataSource necessary to retieve the point
	    data from the dataSource. */
	PropertyStr _pointIdentifierProp;
	std::string& _pointIdentifier;

	/** The identifier for the torque data in the dataSource necessary to retieve the torque
	    data from the dataSource. */
	PropertyStr _torqueIdentifierProp;
	std::string& _torqueIdentifier;

private:

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

//=============================================================================
// METHODS
//=============================================================================
public:
	// CONSTRUCTION
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
	ExternalForce(const Storage &dataSource, std::string forceIdentifier="force", std::string PointIdentifier="point", std::string torqueIdentifier="torque",
		std::string appliedToBodyName="", std::string forceExpressedInBodyName="ground", std::string pointExpressedInBodyName="ground");
	ExternalForce(const ExternalForce& force);
	ExternalForce(DOMElement* aNode);
	~ExternalForce();
	virtual Object* copy() const;

	// Copy properties from XML into member variables
	virtual void updateFromXMLNode();

	// ACCESS METHODS
	/**
	 *  Associate the data source from which the force, point and/or torque data
	 *  is to be extracted.
	 */
	void setDataSource(const Storage *dataSource);

	/**
	 *  Specify or obtain the body to which the force will be applied
	 */
	void setAppliedToBodyName(const std::string &applyToName) const { _appliedToBodyName =applyToName; }
	const std::string& getAppliedToBodyName() const { return (_appliedToBodyName); }

	/**
	 *  Specify or obtain the body in which the point of application is expressed
	 */
	void setPointExpressedInBodyName(const std::string &pointInBodyName) const { _pointExpressedInBodyName =pointInBodyName; }
	const std::string& getPointExpressedInBodyName() const { return (_pointExpressedInBodyName); }

	/**
	 *  Specify or obtain the body in which the force is expressed
	 */
	void setForceExpressedInBodyName(const std::string &forceInBodyName) const { _forceExpressedInBodyName =forceInBodyName; }
	const std::string& getForceExpressedInBodyName() const { return (_forceExpressedInBodyName); }

	/**
	 * Identifiers
	 */
	void setForceIdentifier(const std::string aForceIdentifier) { _forceIdentifier = aForceIdentifier; }
	void setPointIdentifier(const std::string aPointIdentifier) { _pointIdentifier = aPointIdentifier; }
	void setTorqueIdentifier(const std::string aTorqueIdentifier) { _torqueIdentifier = aTorqueIdentifier; }
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
	 * Given SimTK::State object extract all the values necessary to report forces, application location
	 * frame, etc. used in conjunction with getRecordLabels and should return same size Array
	 */
	virtual OpenSim::Array<double> getRecordValues(const SimTK::State& state) const;

#ifndef SWIG
	ExternalForce& operator=(const ExternalForce &aForce);
#endif
	OPENSIM_DECLARE_DERIVED(ExternalForce, Force);

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
	void setupProperties();
	void copyData(const ExternalForce& orig);
//=============================================================================
};	// END of class ExternalForce
//=============================================================================
//=============================================================================

} // end of namespace OpenSim

#endif // __ExternalForce_h__
