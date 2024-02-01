/* -------------------------------------------------------------------------- *
 *                 OpenSim:  testStationDefinedFrame.cpp                      *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2024 Stanford University and the Authors                *
 * Author(s): Adam Kewley                                                     *
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

#include <OpenSim/Simulation/Model/StationDefinedFrame.h>

#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Simulation/Model/Station.h>
#include <OpenSim/Simulation/SimbodyEngine/WeldJoint.h>

#include <catch2/catch_all.hpp>
#include <Simbody.h>

#include <functional>
#include <memory>
#include <type_traits>
#include <utility>

using OpenSim::ArrayPtrs;
using OpenSim::Body;
using OpenSim::Joint;
using OpenSim::Model;
using OpenSim::ModelComponent;
using OpenSim::Object;
using OpenSim::PhysicalOffsetFrame;
using OpenSim::Station;
using OpenSim::StationDefinedFrame;
using OpenSim::WeldJoint;

// helper functions
namespace {
    template<typename T, typename MemberFunc, typename... Args>
    T& EmplaceGeneric(Model& model, MemberFunc adder, Args&&... args)
    {
        static_assert(std::is_constructible<T, Args&&...>::value, "Args must be able to construct the T");

        auto p = std::make_unique<T>(std::forward<Args>(args)...);
        T* ptr = p.get();
        adder(model, p.release());
        return *ptr;
    }

    template<typename T, typename... Args>
    T& EmplaceModelComponent(Model& model, Args&&... args)
    {
        static_assert(std::is_base_of<ModelComponent, T>::value, "T must inherit from ModelComponent");
        return EmplaceGeneric<T>(model, std::mem_fn(&Model::addModelComponent), std::forward<Args>(args)...);
    }

    template<typename T = Body, typename... Args>
    T& EmplaceBody(Model& model, Args&&... args)
    {
        static_assert(std::is_base_of<Body, T>::value, "T must inherit from Body");
        return EmplaceGeneric<T>(model, std::mem_fn(&Model::addBody), std::forward<Args>(args)...);
    }

    template<typename T = Joint, typename... Args>
    T& EmplaceJoint(Model& model, Args&&... args)
    {
        static_assert(std::is_base_of<Joint, T>::value, "T must inherit from Joint");
        return EmplaceGeneric<T>(model, std::mem_fn(&Model::addJoint), std::forward<Args>(args)...);
    }
}

TEST_CASE("StationDefinedFrame_IsRegistered")
{
    // sanity check: `StationDefinedFrame` should be part of the general `Object` registry
    ArrayPtrs<StationDefinedFrame> objs;
    Object::getRegisteredObjectsOfGivenType<StationDefinedFrame>(objs);
    REQUIRE(objs.size() == 1);
}

TEST_CASE("StationDefinedFrame_CanBeDefaultConstructed")
{
    // sanity check: if this doesn't work then something _quite_ strange has happened :>
    REQUIRE_NOTHROW(StationDefinedFrame{});
}

TEST_CASE("StationDefinedFrame_CanCreateAModelContainingAStandaloneStationDefinedFrame")
{
    Model model;

    // add stations to the model
    auto& p1 = EmplaceModelComponent<Station>(model, model.getGround(), SimTK::Vec3{-1.0, -1.0, 0.0});
    auto& p2 = EmplaceModelComponent<Station>(model, model.getGround(), SimTK::Vec3{-1.0,  1.0, 0.0});
    auto& p3 = EmplaceModelComponent<Station>(model, model.getGround(), SimTK::Vec3{ 1.0,  0.0, 0.0});
    auto& origin = p1;

    // add a `StationDefinedFrame` that uses the stations to the model
    EmplaceModelComponent<StationDefinedFrame>(
        model,
        SimTK::CoordinateAxis::XCoordinateAxis(),
        SimTK::CoordinateAxis::YCoordinateAxis(),
        p1,
        p2,
        p3,
        origin
    );

    // the resulting model should finalize etc. fine
    model.buildSystem();
    auto& state = model.initializeState();
    model.realizeReport(state);
}

TEST_CASE("StationDefinedFrame_CanCreateAModelContainingAStationDefinedFrameAsAChild")
{
    Model model;

    // add stations to the model
    auto& p1 = EmplaceModelComponent<Station>(model, model.getGround(), SimTK::Vec3{-1.0, -1.0, 0.0});
    auto& p2 = EmplaceModelComponent<Station>(model, model.getGround(), SimTK::Vec3{-1.0,  1.0, 0.0});
    auto& p3 = EmplaceModelComponent<Station>(model, model.getGround(), SimTK::Vec3{ 1.0,  0.0, 0.0});
    auto& origin = p1;

    // add a `StationDefinedFrame` that uses the stations to the model
    auto& sdf = EmplaceModelComponent<StationDefinedFrame>(
        model,
        SimTK::CoordinateAxis::XCoordinateAxis(),
        SimTK::CoordinateAxis::YCoordinateAxis(),
        p1,
        p2,
        p3,
        origin
    );

    // add a `Body` that will act as the child of a `Joint`
    auto& body = EmplaceBody(
        model,
        "name",
        1.0,
        SimTK::Vec3{0.0, 0.0, 0.0},
        SimTK::Inertia{1.0, 1.0, 1.0}
    );

    // add a `Joint` between the `StationDefinedFrame` and the `Body`
    EmplaceJoint<WeldJoint>(model, "weld", sdf, body);

    // the model should initialize etc. fine
    model.buildSystem();
    auto& state = model.initializeState();
    model.realizeReport(state);
}

TEST_CASE("StationDefinedFrame_CanCreateModelContainingStationDefinedFrameAsParentOfOffsetFrame")
{
    Model model;

    // add stations to the model
    auto& p1 = EmplaceModelComponent<Station>(model, model.getGround(), SimTK::Vec3{-1.0, -1.0, 0.0});
    auto& p2 = EmplaceModelComponent<Station>(model, model.getGround(), SimTK::Vec3{-1.0,  1.0, 0.0});
    auto& p3 = EmplaceModelComponent<Station>(model, model.getGround(), SimTK::Vec3{ 1.0,  0.0, 0.0});
    auto& origin = p1;

    // add a `StationDefinedFrame` that uses the stations to the model
    auto& sdf = EmplaceModelComponent<StationDefinedFrame>(
        model,
        SimTK::CoordinateAxis::XCoordinateAxis(),
        SimTK::CoordinateAxis::YCoordinateAxis(),
        p1,
        p2,
        p3,
        origin
    );

    EmplaceModelComponent<PhysicalOffsetFrame>(model, sdf, SimTK::Transform{});

    // the model should initialize etc. fine
    model.buildSystem();  // TODO: broken by OpenSim/Simulation/Model/Model.cpp:958: only considers PoFs for finalization order
    auto& state = model.initializeState();
    model.realizeReport(state);
}
