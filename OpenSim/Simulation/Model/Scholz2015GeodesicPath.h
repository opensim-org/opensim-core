#ifndef OPENSIM_SCHOLZ2015_GEODESIC_PATH_H
#define OPENSIM_SCHOLZ2015_GEODESIC_PATH_H
/* -------------------------------------------------------------------------- *
 *                     OpenSim: Scholz2015GeodesicPath.h                      *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2024 Stanford University and the Authors                *
 * Author(s): Nicholas Bianco, Pepijn van den Bos, Andreas Scholz             *
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

#include "OpenSim/Simulation/Model/AbstractGeometryPath.h"
#include "Station.h"
#include <OpenSim/Common/Exception.h>

namespace OpenSim {

// TODO temporary, for compilation checks only
struct Geodesic {


};

class Geodesics : public SimTK::AbstractValue {

    Geodesics* clone() const override {return new Geodesics(*this);}

    SimTK::String getTypeName() const override { return "Geodesics"; }

    SimTK::String getValueAsString() const override { return "Geodesics"; }

    static bool isA(const AbstractValue& value)
    {   return dynamic_cast<const Geodesics*>(&value) != nullptr; }

    bool isCompatible(const AbstractValue& value) const override
    {   return isA(value); }

    void compatibleAssign(const AbstractValue& value) override {
        if (!isA(value)) {
            OPENSIM_THROW(Exception,
                    "Geodesics::compatibleAssign(): incompatible value.");
        }

        *this = dynamic_cast<const Geodesics&>(value);

    }


};

/**
 * TODO
 */
class OSIMSIMULATION_API GeodesicPathSegment : public Component {
    OpenSim_DECLARE_CONCRETE_OBJECT(GeodesicPathSegment, Component);

public:

//=============================================================================
// SOCKETS
//=============================================================================
    OpenSim_DECLARE_SOCKET(origin, Station,
            "TODO");
    OpenSim_DECLARE_SOCKET(insertion, Station,
            "TODO");

//=============================================================================
// PROPERTIES
//=============================================================================

protected:
    void extendRealizeTopology(SimTK::State&) const override;
    mutable SimTK::DiscreteVariableIndex m_discreteVarIndex;

private:

    // NaturalGeodesicWrapSolver _solver;
};

/**
 * TODO
 */
class OSIMSIMULATION_API Scholz2015GeodesicPath : public AbstractGeometryPath {
OpenSim_DECLARE_CONCRETE_OBJECT(Scholz2015GeodesicPath, AbstractGeometryPath);

public:
//=============================================================================
// PROPERTIES
//=============================================================================
    OpenSim_DECLARE_LIST_PROPERTY(path_segments, GeodesicPathSegment,
            "TODO");

//=============================================================================
// METHODS
//=============================================================================
    // CONSTRUCTION
    Scholz2015GeodesicPath();

    // ABSTRACT GEOMETRY PATH INTERFACE
    double getLength(const SimTK::State& s) const override;
    double getLengtheningSpeed(const SimTK::State& s) const override;
    void addInEquivalentForces(const SimTK::State& state,
            const double& tension,
            SimTK::Vector_<SimTK::SpatialVec>& bodyForces,
            SimTK::Vector& mobilityForces) const override;
    double computeMomentArm(const SimTK::State& s,
            const Coordinate& aCoord) const override;
    bool isVisualPath() const override;

private:
    // MODEL COMPONENT INTERFACE
    void extendFinalizeFromProperties() override;
    void extendConnectToModel(Model& model) override;
    void extendAddToSystem(SimTK::MultibodySystem& system) const override;

    // CONVENIENCE METHODS
    void constructProperties();
    SimTK::Vector computeCoordinateValues(const SimTK::State& s) const;
    SimTK::Vector computeCoordinateDerivatives(const SimTK::State& s) const;
    void computeLength(const SimTK::State& s) const;
    void computeMomentArms(const SimTK::State& s) const;
    void computeLengtheningSpeed(const SimTK::State& s) const;

    // MEMBER VARIABLES

    // CACHE VARIABLES
    mutable CacheVariable<double> _lengthCV;
    mutable CacheVariable<SimTK::Vector> _momentArmsCV;
    mutable CacheVariable<double> _lengtheningSpeedCV;

}; // class Scholz2015GeodesicPath

} // namespace OpenSim


#endif // OPENSIM_SCHOLZ2015_GEODESIC_PATH_H
