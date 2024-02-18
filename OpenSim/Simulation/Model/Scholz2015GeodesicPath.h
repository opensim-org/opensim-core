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

#include <OpenSim/Simulation/Model/AbstractGeometryPath.h>
#include <OpenSim/Simulation/Model/Station.h>
#include <OpenSim/Common/Exception.h>
#include <OpenSim/Simulation/GeodesicWrapping/GeodesicWrapSurface.h>
//#include <OpenSim/Simulation/GeodesicWrapping/GeodesicWrapSolver.h>

namespace OpenSim {

// TODO: temporary, only used to check compilation.
class Geodesic : public Component {
    OpenSim_DECLARE_CONCRETE_OBJECT(Geodesic, Component);
public:
    std::string getValueAsString() const {
        return "TODO";
    }
};
struct GeodesicWrappingPath {
};
struct GeodesicWrapSolver {
};
struct GeodesicWrapObject {
};

/**
 * A helper class to allow storing a collection of geodesics belonging to a
 * GeodesicPathSegment in a discrete variable.
 */
 // TODO: probably move to wherever we define Geodesic.
class Geodesics : public SimTK::AbstractValue {

public:
    Geodesics() = default;

    void set(const std::vector<Geodesic>& geodesics) {
        _geodesics = geodesics;
    }

    const std::vector<Geodesic>& get() const {
        return _geodesics;
    }

    std::vector<Geodesic>& upd() {
        return _geodesics;
    }

    Geodesics* clone() const override {
        return new Geodesics(*this);
    }

    SimTK::String getTypeName() const override {
        return "Geodesics";
    }

    SimTK::String getValueAsString() const override {
         std::stringstream ss;
         for (const auto& geodesic : _geodesics) {
             ss << geodesic.getValueAsString() << std::endl;
         }
         return ss.str();
    }

    static bool isA(const AbstractValue& value) {
        return dynamic_cast<const Geodesics*>(&value) != nullptr;
    }

    bool isCompatible(const AbstractValue& value) const override {
        return isA(value);
    }

    void compatibleAssign(const AbstractValue& value) override {
        if (!isA(value)) {
            OPENSIM_THROW(Exception,
                    "Geodesics::compatibleAssign(): incompatible value.");
        }

        *this = dynamic_cast<const Geodesics&>(value);
    }

private:
    std::vector<Geodesic> _geodesics;

    // Disable public usage of the getValue() and setValue() methods, which are
    // not relevant to this class (they depend on Value<T>).
    using SimTK::AbstractValue::getValue;
    using SimTK::AbstractValue::updValue;
};


 // TODO this class might deserve its own file.
class OSIMSIMULATION_API GeodesicPathSegment : public ModelComponent {
    OpenSim_DECLARE_CONCRETE_OBJECT(GeodesicPathSegment, ModelComponent);

public:
//=============================================================================
// SOCKETS
//=============================================================================
    OpenSim_DECLARE_SOCKET(origin, Station,
            "TODO");
    OpenSim_DECLARE_SOCKET(insertion, Station,
            "TODO");
    OpenSim_DECLARE_LIST_SOCKET(surfaces, GeodesicWrapSurface,
            "TODO");

//=============================================================================
// SOCKETS
//=============================================================================
    OpenSim_DECLARE_LIST_PROPERTY(default_geodesics, Geodesic,
            "TODO must match number of 'surfaces' and be in the same order.");

//=============================================================================
// METHODS
//=============================================================================
    // CONSTRUCTION
    GeodesicPathSegment();

    void addWrapObstacle(const GeodesicWrapSurface& surface,
                         Geodesic defaultGeodesic);

    double getLength(const SimTK::State& s) const;
    double getLengtheningSpeed(const SimTK::State& s) const;

    void addInEquivalentForces(const SimTK::State& state,
            const double& tension,
            SimTK::Vector_<SimTK::SpatialVec>& bodyForces,
            SimTK::Vector& mobilityForces) const;

protected:
    void extendAddToSystem(SimTK::MultibodySystem& system) const override;
    void extendRealizeTopology(SimTK::State&) const override;
    void extendConnectToModel(Model& model) override;

private:

    // CONVENIENCE METHODS
    void calcWrappingPath(const SimTK::State& s) const;

    // MEMBER VARIABLES
    std::vector<GeodesicWrapObject> _wrapObjects;
    GeodesicWrapSolver _solver;

    // CACHE VARIABLES
    // TODO store path results and geodesics in the same struct?
    mutable CacheVariable<GeodesicWrappingPath> _pathCV;
    mutable SimTK::DiscreteVariableIndex m_discreteVarIndex;
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
    bool isVisualPath() const override;
    double getLength(const SimTK::State& s) const override;
    double getLengtheningSpeed(const SimTK::State& s) const override;
    void addInEquivalentForces(const SimTK::State& state,
            const double& tension,
            SimTK::Vector_<SimTK::SpatialVec>& bodyForces,
            SimTK::Vector& mobilityForces) const override;
    double computeMomentArm(const SimTK::State& s,
            const Coordinate& aCoord) const override;

private:
    // MODEL COMPONENT INTERFACE
    void extendFinalizeFromProperties() override;
    void extendConnectToModel(Model& model) override;
    void extendAddToSystem(SimTK::MultibodySystem& system) const override;

    // CONVENIENCE METHODS
    void constructProperties();
    void computeLength(const SimTK::State& s) const;
    void computeLengtheningSpeed(const SimTK::State& s) const;

    // MEMBER VARIABLES

    // CACHE VARIABLES
    mutable CacheVariable<double> _lengthCV;
    mutable CacheVariable<double> _lengtheningSpeedCV;

}; // class Scholz2015GeodesicPath

} // namespace OpenSim


#endif // OPENSIM_SCHOLZ2015_GEODESIC_PATH_H
