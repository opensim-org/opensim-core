/* ------------------------------------------------------------------------- *
*             OpenSim:  futureOrientationInverseKinematics.cpp               *
* -------------------------------------------------------------------------- *
* The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
* See http://opensim.stanford.edu and the NOTICE file for more information.  *
* OpenSim is developed at Stanford University and supported by the US        *
* National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
* through the Warrior Web program.                                           *
*                                                                            *
* Copyright (c) 2005-2017 Stanford University and the Authors                *
* Author(s): Dimitar Stanev                                                  *
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

#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Common/MarkerData.h>
#include <OpenSim/Analyses/Kinematics.h>
#include <Simbody.h>
#include <iostream>

/*
Demonstrates how orientation sensors can be used for tracking the movements
of the arm and to perform the inverse kinematics. Two IMUs are attached, one 
on the humerus and the other on the radius. The recordings are stored in a 
.trc file. The model is an extended version of the arm26. Currently, OpenSim
does not provide direct tools for employing inverse kinematics based on the
orientation sensors, while Simbody provides these facilities. This code
uses the Simbody interface to execute the inverse kinematics for the specific 
problem and it is not a generic solution.
*/

//#define DEBUG

class OrientationIK
{
    /************************************************************************/
    /* Path variables                                                       */
    /************************************************************************/
    std::string m_model_path;
    std::string m_imu_path;
    std::string m_ik_result_path;

    /************************************************************************/
    /* Name of the bodies where the imus are attached                       */
    /************************************************************************/
    std::string m_humerus_body_name;
    std::string m_radius_body_name;
    
    /************************************************************************/
    /* Number of observations used                                          */
    /************************************************************************/
    static const int OSENSORS = 2;

    /************************************************************************/
    /* IMU indexes to point to the allocation space                         */
    /************************************************************************/
    SimTK::OrientationSensors::OSensorIx m_humerus_mx;
    SimTK::OrientationSensors::ObservationIx m_humerus_ox;
    SimTK::OrientationSensors::OSensorIx m_radius_mx;
    SimTK::OrientationSensors::ObservationIx m_radius_ox;


public:

    OrientationIK(
        std::string model_path, 
        std::string imu_path,
        std::string result_path,
        std::string humerus_body_name, std::string radius_body_name)
    : m_model_path(model_path), 
    m_imu_path(imu_path),
    m_ik_result_path(result_path),
    m_humerus_body_name(humerus_body_name), m_radius_body_name(radius_body_name)
    {
            
    }

    ~OrientationIK()
    {

    }

    /************************************************************************/
    /* Runs the inverse kinematics                                          */
    /************************************************************************/
    void run()
    {
        
        // load model and data
        OpenSim::Model model(m_model_path);

        OpenSim::MarkerData imu_trc(m_imu_path);

        SimTK::State& state = model.initSystem();

        // setup
        SimTK::Assembler ik(model.updMultibodySystem());
        ik.setAccuracy(1e-5);
        SimTK::Markers* markers = new SimTK::Markers();
        SimTK::OrientationSensors* imus = new SimTK::OrientationSensors();
    
        // add markers
        addCustomMarkers(model, *markers, *imus);   
    
        // result storage
        OpenSim::Kinematics kinematics(&model);
        kinematics.setRecordAccelerations(true);

        // move to initial target
        ik.adoptAssemblyGoal(imus);
        imus->moveOneObservation(m_humerus_ox, SimTK::Rotation(
            SimTK::BodyOrSpaceType::BodyRotationSequence,
            imu_trc.getFrame(0).getMarker(0)[0], SimTK::XAxis,
            imu_trc.getFrame(0).getMarker(0)[1], SimTK::YAxis,
            imu_trc.getFrame(0).getMarker(0)[2], SimTK::ZAxis));
        imus->moveOneObservation(m_radius_ox, SimTK::Rotation(
            SimTK::BodyOrSpaceType::BodyRotationSequence,
            imu_trc.getFrame(0).getMarker(1)[0], SimTK::XAxis,
            imu_trc.getFrame(0).getMarker(1)[1], SimTK::YAxis,
            imu_trc.getFrame(0).getMarker(1)[2], SimTK::ZAxis));

        // setup inverse kinematics
        state.setTime(imu_trc.getFrame(0).getFrameTime());
        ik.initialize(state);
        ik.assemble(state);
        kinematics.begin(state);

        // loop for every observation frame (!!!must be same length)
        for (int i = 1; i < imu_trc.getNumFrames(); ++i)
        {        
            OpenSim::MarkerFrame osensor_frame = imu_trc.getFrame(i);
            SimTK::Vec3 humerus_vec = osensor_frame.getMarker(0);
            imus->moveOneObservation(m_humerus_ox, SimTK::Rotation(
                SimTK::BodyOrSpaceType::BodyRotationSequence,
                humerus_vec[0], SimTK::XAxis,
                humerus_vec[1], SimTK::YAxis,
                humerus_vec[2], SimTK::ZAxis));
            SimTK::Vec3 radius_vec = osensor_frame.getMarker(1);
            imus->moveOneObservation(m_radius_ox, SimTK::Rotation(
                SimTK::BodyOrSpaceType::BodyRotationSequence,
                radius_vec[0], SimTK::XAxis,
                radius_vec[1], SimTK::YAxis,
                radius_vec[2], SimTK::ZAxis));

            // track
            state.updTime() = osensor_frame.getFrameTime();
            ik.track(state.getTime());
            ik.updateFromInternalState(state);

            // report
    #ifdef DEBUG
            std::cout << "Frame: " << i << " (t=" << state.getTime() << ")\n";
            std::cout << "Error: " << ik.calcCurrentErrorNorm() << "\n";
            std::flush(std::cout);
    #endif

            // store
            kinematics.step(state, i);
        }
        kinematics.end(state);

        // store results
        kinematics.printResults(m_ik_result_path, "");
    }

private:

    /************************************************************************/
    /* Adds the markers for the line test problem                           */
    /************************************************************************/
    void addCustomMarkers(OpenSim::Model& model, 
        SimTK::Markers& markers, SimTK::OrientationSensors& imus)
    {
        // add orientation osensor
        m_humerus_mx = imus.addOSensor(
            "humerus",
            model.updBodySet().get(m_humerus_body_name).getMobilizedBodyIndex(),
            SimTK::Rotation(SimTK::BodyOrSpaceType::BodyRotationSequence,
            -SimTK::Pi / 2, SimTK::ZAxis,
            SimTK::Pi / 2, SimTK::XAxis),
            1);
    
        m_radius_mx = imus.addOSensor(
            "radius",
            model.updBodySet().get(m_radius_body_name).getMobilizedBodyIndex(),
            SimTK::Rotation(SimTK::BodyOrSpaceType::BodyRotationSequence,
            -SimTK::Pi / 2, SimTK::ZAxis,
            SimTK::Pi , SimTK::XAxis),
            1);
        
    
        // finalize observation order (to allocate ObservationIx)
        static const char* osensor_observation_order[OSENSORS] = {"humerus", "radius"};
        imus.defineObservationOrder(OSENSORS, osensor_observation_order);
    
        // get all ObservationIx
        m_humerus_ox = imus.getObservationIxForOSensor(m_humerus_mx);
        m_radius_ox = imus.getObservationIxForOSensor(m_radius_mx);
    }
};


/**
* Main function
*/
int main()
{
    try {
      
        OrientationIK ik(
            "futureOrientationInverseKinematics.osim",
            "futureOrientationInverseKinematics.trc",
            "futureOrientationInverseKinematics",
            "humerus_r", 
            "radius_r");
        ik.run();

    }
    catch (const std::exception& ex)
    {
        std::cout << "Exception: " << ex.what() << std::endl;
        return 1;
    }
    catch (...)
    {
        std::cout << "Unrecognized exception " << std::endl;
        return 1;
    }
    
    //system("pause");
    
    return 0;
}
