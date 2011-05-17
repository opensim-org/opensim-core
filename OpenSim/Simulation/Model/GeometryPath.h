#ifndef __GeometryPath_h__
#define __GeometryPath_h__

// GeometryPath.h
// Author: Peter Loan
/*
 * Copyright (c)  2009, Stanford University. All rights reserved. 
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
#include <iostream>
#include <math.h>
#include <OpenSim/Simulation/osimSimulationDLL.h>
#include "OpenSim/Simulation/Model/ModelComponent.h"
#include <OpenSim/Common/PropertyInt.h>
#include <OpenSim/Common/PropertyObj.h>
#include <OpenSim/Common/ScaleSet.h>
#include <OpenSim/Common/VisibleObject.h>
#include "PathPointSet.h"
#include <OpenSim/Simulation/Wrap/PathWrapSet.h>


#ifdef SWIG
	#ifdef OSIMSIMULATION_API
		#undef OSIMSIMULATION_API
		#define OSIMSIMULATION_API
	#endif
#endif

namespace OpenSim {

class Coordinate;
class WrapResult;
class WrapObject;
class PointForceDirection;

//=============================================================================
//=============================================================================
/**
 * A base class representing a path (muscle, ligament, etc.).
 *
 * @author Peter Loan
 * @version 1.0
 */
class OSIMSIMULATION_API GeometryPath : public ModelComponent  
{
//=============================================================================
// DATA
//=============================================================================
protected:
	// the set of points defining the path
	PropertyObj _pathPointSetProp;
	PathPointSet &_pathPointSet;

	// used to display the path in the 3D window
	PropertyObj _displayerProp;
	VisibleObject &_displayer;

	// the wrap objects that are associated with this path
	PropertyObj _pathWrapSetProp;
	PathWrapSet &_pathWrapSet;

	// used for scaling tendon and fiber lengths
	double _preScaleLength;

	// object that owns this GeometryPath object
	Object* _owner;


//=============================================================================
// METHODS
//=============================================================================
	//--------------------------------------------------------------------------
	// CONSTRUCTION
	//--------------------------------------------------------------------------
public:
	GeometryPath();
	GeometryPath(const GeometryPath &aPath);
	virtual ~GeometryPath();
	virtual Object* copy() const;

	void setName(const std::string &aName);
#ifndef SWIG
	GeometryPath& operator=(const GeometryPath &aPath);
#endif
	void copyData(const GeometryPath &aPath);
	const PathPointSet& getPathPointSet() const { return _pathPointSet; }
	PathPointSet& updPathPointSet() const { return _pathPointSet; }
	PathWrapSet& getWrapSet() const { return _pathWrapSet; }

	//--------------------------------------------------------------------------
	// UTILITY
	//--------------------------------------------------------------------------
	PathPoint* addPathPoint(const SimTK::State& s, int aIndex, OpenSim::Body& aBody);
	PathPoint* appendNewPathPoint(const std::string& proposedName, 
								OpenSim::Body& aBody, const SimTK::Vec3& aPositionOnBody);
	bool canDeletePathPoint( int aIndex);
	bool deletePathPoint(const SimTK::State& s, int aIndex);
	void addPathWrap(const SimTK::State& s, WrapObject& aWrapObject);
	void moveUpPathWrap(const SimTK::State& s, int aIndex);
	void moveDownPathWrap(const SimTK::State& s, int aIndex);
	void deletePathWrap(const SimTK::State& s, int aIndex);
	bool replacePathPoint(const SimTK::State& s, PathPoint* aOldPathPoint, PathPoint* aNewPathPoint); 

    //--------------------------------------------------------------------------
    // GET
    //--------------------------------------------------------------------------
	Object* getOwner() const { return _owner; }
	void setOwner(Object *anObject) {_owner = anObject; };

	virtual double getLength( const SimTK::State& s) const;
	virtual void setLength( const SimTK::State& s, double length) const;
	virtual double getPreScaleLength( const SimTK::State& s) const;
	virtual void setPreScaleLength( const SimTK::State& s, double preScaleLength);
	virtual const Array<PathPoint*>& getCurrentPath( const SimTK::State& s) const;

	virtual const Array<PathPoint*>& getCurrentDisplayPath(const SimTK::State& s) const;
	
	/** get the the path as PointForceDirections directions */
	void getPointForceDirections(const SimTK::State& s, OpenSim::Array<PointForceDirection*> *rPFDs) const;

	virtual double getLengtheningSpeed(const SimTK::State& s) const;
	virtual void setLengtheningSpeed( const SimTK::State& s, double speed ) const;

	//--------------------------------------------------------------------------
	// COMPUTATIONS
	//--------------------------------------------------------------------------
private:
	void computePath(const SimTK::State& s ) const;
	void computeLengtheningSpeed(const SimTK::State& s) const;
	void applyWrapObjects(const SimTK::State& s, Array<PathPoint*>& path ) const;
	double _calc_path_length_change(const SimTK::State& s, WrapObject& wo, WrapResult& wr, const Array<PathPoint*>& path) const; 
	virtual double calcLengthAfterPathComputation(const SimTK::State& s, const Array<PathPoint*>& currentPath) const;
public:
	virtual double computeMomentArm(const SimTK::State& s, const Coordinate& aCoord);

	//--------------------------------------------------------------------------
	// SCALING
	//--------------------------------------------------------------------------
	virtual void preScale(const SimTK::State& s, const ScaleSet& aScaleSet);
	virtual void scale(const SimTK::State& s, const ScaleSet& aScaleSet);
	virtual void postScale(const SimTK::State& s, const ScaleSet& aScaleSet);
	virtual int getNumStateVariables() const { return 0;};

protected:

	virtual void setup(Model& aModel);
	virtual void initState(SimTK::State& s) const;
	virtual void createSystem(SimTK::MultibodySystem& system) const;
	virtual void setDefaultsFromState(const SimTK::State& state) {};

public:
	//--------------------------------------------------------------------------
	// Visible Object Support
	//--------------------------------------------------------------------------
	virtual VisibleObject* getDisplayer() const { 
		return &_displayer; 
	}
	virtual void updateDisplayer(const SimTK::State& s);
	OPENSIM_DECLARE_DERIVED(GeometryPath, Object);
	// Update the geometry attached to the path (location of path points and connecting segments
	//  all in global/interial frame)
	virtual void updateGeometry(const SimTK::State& s);

private:
	void setNull();
	void setupProperties();
	void updateDisplayPath(const SimTK::State& s);
	void updateGeometrySize(const SimTK::State& );
	void updateGeometryLocations(const SimTK::State& s);
	void namePathPoints(int aStartingIndex);
    void placeNewPathPoint(const SimTK::State& s, SimTK::Vec3& aOffset, int aIndex, const OpenSim::Body& aBody);

//=============================================================================
};	// END of class GeometryPath
//=============================================================================
//=============================================================================

} // end of namespace OpenSim

#endif // __GeometryPath_h__


