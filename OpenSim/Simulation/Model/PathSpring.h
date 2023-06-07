#ifndef OPENSIM_PATH_SPRING_H_
#define OPENSIM_PATH_SPRING_H_
/* -------------------------------------------------------------------------- *
 *                            OpenSim:  PathSpring.h                          *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2017 Stanford University and the Authors                *
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


//=============================================================================
// INCLUDES
//=============================================================================
#include "Force.h"

namespace OpenSim {

class GeometryPath;
class ScaleSet;

//=============================================================================
//=============================================================================
/**
 * A class implementing a PathSpring. The path of the PathSpring is
 * determined by a GeometryPath object. A PathSpring is a massless Force
 * element which applies tension along a path connected to bodies and can wrap
 * over surfaces.  The tension is proportional to its stretch beyond its
 * resting length and the amount of dissipation scales with amount of stretch,
 * such that tension = (K*s)*(1+D*ldot) where stretch, s = l-lo for l > lo, and 
 * 0 otherwise. l is the path length of the spring and lo is its rest length.
 * K is the linear stiffness and D is the dissipation factor.
 * When l < lo the spring applies no tension to the bodies and considered
 * to be slack.
 *
 * @author Ajay Seth
 */
class OSIMSIMULATION_API PathSpring : public Force {
OpenSim_DECLARE_CONCRETE_OBJECT(PathSpring, Force);
public:
//=============================================================================
// PROPERTIES
//=============================================================================
    OpenSim_DECLARE_PROPERTY(resting_length, double,
        "The resting length (m) of the PathSpring");
    OpenSim_DECLARE_PROPERTY(stiffness, double,
        "The linear stiffness (N/m) of the PathSpring");
    OpenSim_DECLARE_PROPERTY(dissipation, double,
        "The dissipation factor (s/m) of the PathSpring");
    OpenSim_DECLARE_UNNAMED_PROPERTY(GeometryPath, 
        "The GeometryPath defines the set of points and wrapping surface" 
        "interactions that form the path of action of the PathSpring");

//=============================================================================
// OUTPUTS
//=============================================================================
    OpenSim_DECLARE_OUTPUT(length, double, getLength,
        SimTK::Stage::Position);
    OpenSim_DECLARE_OUTPUT(stretch, double, getStretch,
        SimTK::Stage::Position);
    OpenSim_DECLARE_OUTPUT(lengthening_speed, double, getLengtheningSpeed,
        SimTK::Stage::Velocity);
    OpenSim_DECLARE_OUTPUT(tension, double, getTension,
        SimTK::Stage::Dynamics);

//==============================================================================
// PUBLIC METHODS
//==============================================================================
    
    /** Construct a PathSpring with default parameters. Users should note
    that the default values for resting_length, stiffness, and dissipation
    are `NaN` so they must be set before simulating. */
    PathSpring();

    /** Convenience constructor with PathSpring parameters
     * @param name          name of this %PathSpring Object
     * @param restLength    the spring's resting length
     * @param stiffness     the spring stiffness
     * @param dissipation   the dissipation factor */
    PathSpring(const std::string& name, 
        double restLength, double stiffness, double dissipation);


    // Uses default (compiler-generated) destructor, copy constructor, and copy
    // assignment operator.

    //--------------------------------------------------------------------------
    //  <B> Get and set PathSpring properties </B>
    //--------------------------------------------------------------------------
    // Properties
    /** get/set the resting length */
    double getRestingLength() const 
    {   return get_resting_length(); }
    void setRestingLength(double restingLength);

    /** get/set the stiffness */
    double getStiffness() const 
    {   return get_stiffness(); }
    void setStiffness(double stiffness);

    /** get/set the dissipation factor */
    double getDissipation() const 
    {   return get_dissipation(); }
    void setDissipation(double dissipation);

    /** Access the GeometryPath to update connection points and
        specify wrap objects the path can interact with. */
    const GeometryPath& getGeometryPath() const 
    {   return get_GeometryPath(); }
    GeometryPath& updGeometryPath() 
    {   return upd_GeometryPath(); }

    //--------------------------------------------------------------------------
    //  <B> State dependent values </B>
    //--------------------------------------------------------------------------
    /** get the length of the PathSpring. Accessible at Stage::Position*/
    double getLength(const SimTK::State& s) const;
    /** get the stretch in the PathSpring. The value of the stretch 
        can only be obtained after the system as be realized to Stage::Position*/
    double getStretch(const SimTK::State& s) const;
    /** get the length of the PathSpring. Accessible at Stage::Velocity*/
    double getLengtheningSpeed(const SimTK::State& s) const;
    /** get the tension generated by the PathSpring. The value of the tension 
        can only be obtained after the system as be realized to Stage::Dynamics */
    double getTension(const SimTK::State& s) const;

    //--------------------------------------------------------------------------
    // COMPUTATIONS
    //--------------------------------------------------------------------------
    /** compute the moment-arm of the PathSpring about a coordinate of interest. */
    double computeMomentArm(const SimTK::State& s, const Coordinate& aCoord) const;

    //--------------------------------------------------------------------------
    // SCALE
    //--------------------------------------------------------------------------

    /** Adjust the resting length of the path spring after the model has been
        scaled. The `resting_length` property is multiplied by the quotient of
        the current path length and the path length before scaling. */
    void extendPostScale(const SimTK::State& s,
                         const ScaleSet& scaleSet) override;

protected:
    /** Implementation of Force component virtual method */
    void computeForce(const SimTK::State& s, 
                              SimTK::Vector_<SimTK::SpatialVec>& bodyForces, 
                              SimTK::Vector& generalizedForces) const override; 

    /** Implement ModelComponent interface. */
    void extendFinalizeFromProperties() override;

    //Force reporting
    /** 
     * Methods to query a Force for the value actually applied during simulation
     * The names of the quantities (column labels) is returned by this first function
     * getRecordLabels()
     */
    OpenSim::Array<std::string> getRecordLabels() const override {
        OpenSim::Array<std::string> labels("");
        labels.append(getName()+"_tension");
        return labels;
    }
    /**
     * Given SimTK::State object extract all the values necessary to report forces, application location
     * frame, etc. used in conjunction with getRecordLabels and should return same size Array
     */
    OpenSim::Array<double> getRecordValues(const SimTK::State& state) const override {
        OpenSim::Array<double> values(1);
        values.append(getTension(state));
        return values;
    }

private:
    void constructProperties();

//=============================================================================
};  // END of class PathSpring
//=============================================================================
//=============================================================================
} // end of namespace OpenSim

#endif // OPENSIM_PATH_SPRING_H_
