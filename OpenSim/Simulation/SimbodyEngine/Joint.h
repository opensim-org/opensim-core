#ifndef OPENSIM_JOINT_H_
#define OPENSIM_JOINT_H_
/* -------------------------------------------------------------------------- *
 *                            OpenSim:  Joint.h                               *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2012 Stanford University and the Authors                *
 * Author(s): Frank C. Anderson, Peter Loan, Ajay Seth                        *
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
#include <OpenSim/Simulation/Model/ModelComponent.h>
#include <OpenSim/Simulation/Model/CoordinateSet.h>
#include <OpenSim/Common/Function.h>

namespace OpenSim {

class Body;
class ScaleSet;

//=============================================================================
//=============================================================================
/**
 * An OpenSim Joint is an OpenSim::ModelComponent which connects two Bodies 
 * together and speficies their relative permissible motion as described in  
 * internal coordinates. The base Joint specifies the two frames (one each body),
 * which the joint spans. The relative motion (including the # of coordinates)
 * are defined by concrete Joints, which specify the permissible kinematics of
 * a child joint frame (on a child body) with respect to a parent joint frame
 * (on a parent body). The designation of parent and child are used only to
 * identify the directionality of the joint and in which frame the joint
 * coordinates are expressed. For example, A PinJoint between a parent, P, 
 * and a child body, B, frames has a coordinate value of zero when the two 
 * frames are aligned and positive coordinate values are the angle between the 
 * frames' X-axes given a positive Z-rotation of the child frame about the  
 * coincident Z-axis in the parent frame.
 *
 * Concrete Joints can specify relative translations and even coupled 
 * rotations and translations (@see EllipsoidJoint and CustomJoint).
 *
 * @author Ajay Seth
 */
class OSIMSIMULATION_API Joint : public ModelComponent {
OpenSim_DECLARE_ABSTRACT_OBJECT(Joint, ModelComponent);

public:

//==============================================================================
// PROPERTIES
//==============================================================================
    /** @name Property declarations 
    These are the serializable properties associated with a Joint. **/
    /**@{**/
	OpenSim_DECLARE_PROPERTY(location_in_parent, SimTK::Vec3, 
		"Location of the joint in the parent body specified in the parent "
		"reference frame. Default is (0,0,0).");

	OpenSim_DECLARE_PROPERTY(orientation_in_parent, SimTK::Vec3, 
		"Orientation of the joint in the parent body specified in the parent "
		"reference frame. Euler XYZ body-fixed rotation angles are used to "
		"express the orientation. Default is (0,0,0).");

	OpenSim_DECLARE_PROPERTY(location_in_child, SimTK::Vec3, 
		"Location of the joint in the child body specified in the child "
		"reference frame. For SIMM models, this vector is always the zero "
		"vector (i.e., the body reference frame coincides with the joint). ");

	OpenSim_DECLARE_PROPERTY(orientation_in_child, SimTK::Vec3, 
		"Orientation of the joint in the child body specified in the child body "
		"reference frame.  Euler XYZ body-fixed rotation angles are used to "
		"express the orientation. " );

	OpenSim_DECLARE_UNNAMED_PROPERTY(CoordinateSet, 
		"Set holding the generalized coordinates (q's) that parmeterize this joint." );

    OpenSim_DECLARE_PROPERTY(reverse, bool, 
		"Whether the joint transform defines parent->child or child->parent."); 
    /**@}**/

//=============================================================================
// METHODS
//=============================================================================

	//--------------------------------------------------------------------------
	// CONSTRUCTION
	//--------------------------------------------------------------------------
	/** DEFAULT CONSTRUCTION */
	Joint();

	/** Convenience Constructor */
	/** Create a Joint where the parenet and body are specified as well as the
	    joint frames in the child and parent bodies in terms of their location
		and oriendtation in the respective bodies. 
		TODO remove reverse option and determine direction from tree structure.*/
	Joint(const std::string &name, const Body &parent, 
		  const SimTK::Vec3& locationInParent, const SimTK::Vec3& orientationInParent,
		  const OpenSim::Body& child,
		  const SimTK::Vec3& locationInchild, const SimTK::Vec3& orientationInChild, 
		  bool reverse = false);

	virtual ~Joint();

	// GET & SET

	void setChildBodyName(const std::string& name);
	const std::string& Joint::getChildBodyName() const;

	/**
	 * Set the child body that this joint connects.
	 *
	 * @param child Body reference.
	 */
	void setChildBody(OpenSim::Body& child);

	/**
	 * Get the child body that this joint connects to.
	 *
	 * @return const Body reference.
	 */
	const OpenSim::Body& getChildBody() const;

	void setLocationInChild(const SimTK::Vec3& aLocation);
	const SimTK::Vec3& getLocationInChild() const;
	void setOrientationInChild(const SimTK::Vec3& aOrientation);
	const SimTK::Vec3& getOrientationInChild() const;

	// Relating to the parent body
	void setParentBodyName(const std::string& aName);
	const std::string& getParentBodyName() const;

	/**
	* Set the parent body that this joint connects.
	*
	* @param parent Body.
	*/
	void setParentBody(OpenSim::Body& parent);
	/**
	 * Get the parent body to which this joint attaches.
	 *
	 * @return const ref to parent Body.
	 */
	const OpenSim::Body& getParentBody() const;

	void setLocationInParent(const SimTK::Vec3& aLocation);
	const SimTK::Vec3& getLocationInParent() const;
	void setOrientationInParent(const SimTK::Vec3& aOrientation);
	const SimTK::Vec3& getOrientationInParent() const;

	/** Get the Joint frames expressed as body transforms. Only available after 
	    connectToModel() has been called on the Joint. */
	const SimTK::Transform& getParentTransform() const
		{ return _jointFrameInParent; }
	const SimTK::Transform& getChildTransform() const
		{ return _jointFrameInChild; }

	// Coordinate Set
	const CoordinateSet& getCoordinateSet() const { return get_CoordinateSet(); }

	bool getReverse() const { return get_reverse(); }

	//Model building
	virtual int numCoordinates() const = 0;


	// Utility
	bool isCoordinateUsed(const Coordinate& aCoordinate) const;
	
	// Computation
	/** Given some system mobility (generalized) forces, calculate the equivalent spatial body force for this Joint. 
	Keep in mind that there are typically nm < 6 mobilities per joint with an infinte set of solutions that can map
	nm gen forces to 6 spatial force components (3 for torque + 3 for force). The solution returned provides the
	"most" effective force and torque in the joint frame. This means the smallest magnituded force and/or 
	torque that will result in the same generalized force. If a generalized force is defined along/about a joint
	axis, then this should be evident in the reported results as a force or torque on the same axis. 
	NOTE: Joints comprised of multiple mobilizers and/or constraints, should override this method and account for multiple
	      internal components 
	@param s containing the generalized coordinate and speed values 
	@param mobilityForces for the system as computed by inverse dynamics, for example 
	@return spatial force, FB_G, acting on the body connected by this joint at its location B, expressed in ground.  */
	virtual SimTK::SpatialVec calcEquivalentSpatialForce(const SimTK::State &s, const SimTK::Vector &mobilityForces) const;


	/** Joints in general do not contribute power since the reaction space forces
	    are orthogonal to the mobility space. However, when joint motion is prescribed, 
		the internal forces that move the joint will do work. In this case, the power is
		non-zero */
	virtual double calcPower(const SimTK::State &s) const;

	// SCALE
	/**
	* Scale a joint based on XYZ scale factors for the bodies.
	* Generic behavior is to scale the locations on parent and on the body 
	* according to scale factors of the bodies upon which they are located.
	*
	* Joint subclasses should invoke this method before scaling joint specific 
	* properties
	* 
	* @param aScaleSet Set of XYZ scale factors for the bodies.
	*/
	virtual void scale(const ScaleSet& aScaleSet);
    // ModelComponent interface.
	
	/**
	* Perform some set up functions that happen after the
	* object has been deserialized or copied.
	*
	* @param aModel OpenSim model containing this Joint.
	*/
	void connectToModel(Model& aModel) OVERRIDE_11;

protected:
    // TODO: child overrides must invoke Joint::addToSystem()
    // *after* they create the MobilizedBody. This is an API bug
    // since we want to have children invoke parent first.
    void addToSystem(SimTK::MultibodySystem& system) const OVERRIDE_11;
    void initStateFromProperties(SimTK::State& s) const OVERRIDE_11;
    void setPropertiesFromState(const SimTK::State& state) OVERRIDE_11;

	/** Construct coordinates according to the mobilities of the Joint */
	void constructCoordinates();

	// Methods that allow access for Joint subclasses to data members of objects that
	// Joint befriends like Body, Coordinate and SimbodyEngine
	const SimTK::MobilizedBodyIndex getMobilizedBodyIndex(const Body& body) const; 

	void setChildMobilizedBodyIndex(SimTK::MobilizedBodyIndex index) const;
	void setCoordinateMobilizedBodyIndex(Coordinate *aCoord, SimTK::MobilizedBodyIndex index) const {aCoord->_bodyIndex = index;}
	void setCoordinateMobilizerQIndex(Coordinate *aCoord, int index) const
		{ aCoord->_mobilizerQIndex = SimTK::MobilizerQIndex(index);}
	void setCoordinateModel(Coordinate *aCoord, Model *aModel) const {aCoord->_model = aModel;}

	/** Updating XML formating to latest revision */
	void updateFromXMLNode(SimTK::Xml::Element& aNode, int versionNumber);

	/** Calculate the equivalent spatial force, FB_G, acting on a mobilized body specified by index 
	   acting at its mobilizer frame B, expressed in ground.  */
	SimTK::SpatialVec calcEquivalentSpatialForceForMobilizedBody(const SimTK::State &s, const SimTK::MobilizedBodyIndex mbx, const SimTK::Vector &mobilityForces) const;

	/** Utility method for creating the underlying MobilizedBody of the desired 
	    type of the concrete Joint. It is templatized by the MobilizedBody type. 
		Concrete class cannot override this method but can customize addToSystem()
		instead of using this service. It assumes that the MobilizedBody is the child
		body and the parent body correspond to those of the Joint. For more
		granularity as to which bodies are being interconnected internally, use
		createMobilizedBody(MobilizedBody& parent, const SimTK::Transform& parentTransform,
		Body& child, const SimTK::Transform& childTransform).*/
	template <typename T>
	T createMobilizedBody(const SimTK::Transform& parentTransform,
		const SimTK::Transform& childTransform) const {
		SimTK::MobilizedBody& parent = 
			_model->updMatterSubsystem().updMobilizedBody(getParentBody().getIndex());
		SimTK::Body child = SimTK::Body::Rigid(getChildBody().getMassProperties());

		int beginAtIndex = 0;
		return createMobilizedBody<T>(parent, parentTransform, 
			                           child, childTransform, beginAtIndex);
	}
	/** Utility method for creating an underlying MobilizedBody of the desired
	type. Method is templatized by the MobilizedBody. Unlike the previous method, 
	the parent and child of the mobilized body are not assumed to be those of the
	Joint. This enables Joint component makers to introduce intermediate MobilizedBodies
	for the purpose of creating more complex Joints. If more than one MobilizedBody
	is being created, it is up to the caller to supply the corresponding 
	coordinateIndex for the puprose of automatically assigning the sysetm and allocation
	indices necessary for the Coordinates of this Joint to access coordinate and 
	speed values. As a convenience the startingCoorinateIndex is updated so 
	that sequential calls will increment correctly.
	*/
	template <typename T>
	T createMobilizedBody(SimTK::MobilizedBody& parent, 
		                const SimTK::Transform& parentTransform,
		                const SimTK::Body& child, 
						const SimTK::Transform& childTransform, 
						int& startingCoorinateIndex) const {
		// CREATE MOBILIZED BODY
		T simtkBody(parent,	parentTransform, child,	childTransform);

		SimTK::MobilizedBodyIndex mbix = simtkBody.getMobilizedBodyIndex();
		
		const CoordinateSet& coords = get_CoordinateSet();
		int nc = numCoordinates();

		SimTK_ASSERT1(nc == coords.getSize(), "%s list of coordinates does not match number of mobilities.",
			getConcreteClassName());

		// Assumes nq = nu for the MobilizedBody
		// TODO: generalize so that we handle nq >= nu
		int nq = getNumMobilities<T>(simtkBody);

		int j = startingCoorinateIndex;
		for (int iq = 0; iq < nq; ++iq){
			if (j < nc){ // assign
				coords[j]._mobilizerQIndex = SimTK::MobilizerQIndex(iq);
				coords[j]._bodyIndex = simtkBody.getMobilizedBodyIndex();
				j++;
			}
			else{
				string msg = getConcreteClassName() +
					" creating MobilizedBody with more mobilities than declared Coordinates.";
				throw Exception(msg);
			}
		}
		//update the startingCoorinateIndex
		startingCoorinateIndex = j;

		setChildMobilizedBodyIndex(mbix);
		
		return simtkBody;
	}

private:
	void setNull();

	/** Construct the infrastructure of the Joint component.
	    Begin with its properties. */
	void constructProperties() OVERRIDE_11;

	/** Next define its structural dependencies on other components.
		These will be the parent and child bodies of the Joint.*/
	void constructStructuralConnectors() OVERRIDE_11;

	//=========================================================================
	// DATA
	//=========================================================================
	// Hold complete transforms for the joint frame's in connected bodies
	SimTK::Transform _jointFrameInChild;
	SimTK::Transform _jointFrameInParent;

	/** Utility method for accessing the number of mobilities provided by 
	    an underlying MobilizedBody */
	template <typename T>
	int getNumMobilities(const T& mobod) const 
	{
		return mobod.getDefaultQ().size();
	}

    friend class JointSet;

//=============================================================================
};	// END of class Joint
//=============================================================================
//=============================================================================

// Specializations
template <>
inline int Joint::getNumMobilities(const SimTK::MobilizedBody::Pin& mobod) const
{
	return 1;
}

template <>
inline int Joint::getNumMobilities(const SimTK::MobilizedBody::Slider& mobod) const
{
	return 1;
}

template <>
inline int Joint::getNumMobilities(const SimTK::MobilizedBody::Weld& mobod) const
{
	return 0;
}

template <>
inline int Joint::getNumMobilities(const SimTK::MobilizedBody::Universal& mobod) const
{
	return 2;
}

template <>
inline int Joint::getNumMobilities(const SimTK::MobilizedBody::Ball& mobod) const
{
	return 3;
}

template <>
inline int Joint::getNumMobilities(const SimTK::MobilizedBody::Ellipsoid& mobod) const
{
	return 3;
}

template <>
inline int Joint::getNumMobilities(const SimTK::MobilizedBody::Free& mobod) const
{
	return 6;
}


} // end of namespace OpenSim

#endif // OPENSIM_JOINT_H_


