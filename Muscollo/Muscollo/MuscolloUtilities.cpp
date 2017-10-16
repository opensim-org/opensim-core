/* -------------------------------------------------------------------------- *
 * OpenSim Muscollo: MuscolloUtilities.cpp                                               *
 * -------------------------------------------------------------------------- *
 * Copyright (c) 2017 Stanford University and the Authors                     *
 *                                                                            *
 * Author(s): Christopher Dembia                                              *
 *                                                                            *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may    *
 * not use this file except in compliance with the License. You may obtain a  *
 * copy of the License at http://www.apache.org/licenses/LICENSE-2.0          *
 *                                                                            *
 * Unless required by applicable law or agreed to in writing, software        *
 * distributed under the License is distributed on an "AS IS" BASIS,          *
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.   *
 * See the License for the specific language governing permissions and        *
 * limitations under the License.                                             *
 * -------------------------------------------------------------------------- */

#include "MuscolloUtilities.h"

#include <OpenSim/Common/TimeSeriesTable.h>
#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Simulation/StatesTrajectory.h>
#include <Simbody/internal/Visualizer_InputListener.h>

using namespace OpenSim;

Storage OpenSim::convertTableToStorage(const TimeSeriesTable& table) {
    Storage sto;
    OpenSim::Array<std::string> labels("", (int)table.getNumColumns() + 1);
    labels[0] = "time";
    for (int i = 0; i < (int)table.getNumColumns(); ++i) {
        labels[i + 1] = table.getColumnLabel(i);
    }
    sto.setColumnLabels(labels);
    const auto& times = table.getIndependentColumn();
    for (unsigned i_time = 0; i_time < table.getNumRows(); ++i_time) {
        auto rowView = table.getRowAtIndex(i_time);
        sto.append(times[i_time], SimTK::Vector(rowView.transpose()));
    }
    return sto;
}

// Based on code from simtk.org/projects/predictivesim SimbiconExample/main.cpp.
void OpenSim::visualize(Model model, Storage statesSto) {
    model.setUseVisualizer(true);
    model.initSystem();

    const double dataRate = 90; // Hz.
    const double frameRate = 30; // Hz.
    // TODO rename.
    // const double speedMultiplier = dataRate / frameRate;

    // Prepare data.
    // -------------
    statesSto.resample(1.0 / dataRate, 4 /* degree */);
    auto statesTraj =
            StatesTrajectory::createFromStatesStorage(model, statesSto);
    OPENSIM_THROW_IF(!statesTraj.isCompatibleWith(model), Exception,
            "Model is not compatible with the provided StatesTrajectory.");

    // Set up visualization.
    // ---------------------
    model.updMatterSubsystem().setShowDefaultGeometry(true);
    auto& viz = model.updVisualizer().updSimbodyVisualizer();
    std::string modelName = model.getName().empty() ? "<unnamed>"
                                                    : model.getName();
    std::string motionName = statesSto.getName().empty() ? "<unnamed>"
                                                         : statesSto.getName();
    viz.setWindowTitle("Visualizing model '" + modelName + "' with motion '" +
            motionName + "'.");
    viz.setMode(SimTK::Visualizer::PassThrough);
    viz.setDesiredBufferLengthInSec(0);
    viz.setDesiredFrameRate(frameRate);
    viz.setShowSimTime(true);
    auto& silo = model.updVisualizer().updInputSilo();


    // BodyWatcher to control camera.
    // TODO


    const int numFrames = (int)statesTraj.getSize();
    /*
    const SimTK::Real initialTime = statesTraj.front().getTime();
    const SimTK::Real finalTime = statesTraj.back().getTime();
     */

    /*
    // Add sliders to control playback.
    const int speedSliderIndex = 1;
    const double minSpeed = 0;
    const double maxSpeed = 4;
    viz.addSlider("Speed", speedSliderIndex, minSpeed, maxSpeed, 1.0);
    const int timeSliderIndex = 2;
    viz.addSlider("Time", timeSliderIndex, initialTime, finalTime, initialTime);

    double speed = 1.0 * speedMultiplier;
    SimTK::Real time = initialTime;
    double prevSpeed = speed;
    int prevFrame = -1;
     */

    int iframe = 0;
    while (true) {
        /*
        // TODO rename frame.
        for (double frame = 0; frame < numFrames; frame += speed) {
            time = frame / numFrames;

            // Slider input.
            int sliderIndex;
            double sliderValue;
            if (silo.takeSliderMove(sliderIndex, sliderValue)) {
                if (sliderIndex == speedSliderIndex) {
                    speed = speedMultiplier * sliderValue;
                }
                else if (sliderIndex == timeSliderIndex) {
                    time = sliderValue;
                    frame = time * dataRate;
                } else {
                    std::cout << "Internal error: unrecognized slider." <<
                            std::endl;
                }
            }

            // Key input.
            unsigned key, modifiers;
            if (silo.takeKeyHit(key, modifiers)) {
                // Exit.
                if (key == SimTK::Visualizer::InputListener::KeyEsc) {
                    std::cout << "Exiting visualization." << std::endl;
                    return;
                }
                // Smart zoom.
                else if (key == 'r') {
                    viz.zoomCameraToShowAllGeometry();
                }
                // Pause.
                else if (key == ' ') {
                    if (speed != 0.0) {
                        prevSpeed = speed;
                        speed = 0.0;
                    } else {
                        speed = prevSpeed;
                    }
                }
            }

            if (frame < 0) frame = 0;
            if ((int)frame == numFrames) frame = numFrames - 1;

            // TODO rename to realTimeFactor.
            viz.setSliderValue(speedSliderIndex, speed / speedMultiplier);
            viz.setSliderValue(timeSliderIndex, time);

            viz.report(statesTraj[(int)frame]);
            prevFrame = frame;
        }
        */
        if (iframe == numFrames) iframe = 0;
        // std::cout << "DEBUG " << iframe << std::endl;

        // User input.

        viz.report(statesTraj[iframe]);

        ++iframe;
    }

}
