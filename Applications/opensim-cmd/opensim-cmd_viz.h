#ifndef OPENSIM_CMD_VIZ_H_
#define OPENSIM_CMD_VIZ_H_
/* -------------------------------------------------------------------------- *
 *                      OpenSim:  opensim-cmd_viz.h                           *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2020 Stanford University and the Authors                *
 * Author(s): Chris Dembia, Ayman Habib                                       *
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

#include "parse_arguments.h"
#include <docopt.h>
#include <iostream>

#include <OpenSim/OpenSim.h>
#include <OpenSim/Simulation/VisualizerUtilities.h>

static const char HELP_VIZ[] =
R"(Show a model, motion, or marker/orientation data with the Simbody Visualizer.

Usage:
  opensim-cmd [options]... viz [--geometry=<path>] model <model-file> [<states-file>]
  opensim-cmd [options]... viz [--geometry=<path>] data <data-file> [--model=<model-file>] [--layout=<layout>] [--rotate=<rotations>]
  opensim-cmd viz -h | --help

Options:
  -L <path>, --library <path>  Load a plugin.
  -o <level>, --log <level>  Logging level.
  -g <path>, --geometry <path> Search for geometry mesh files in this path.
  -m <path>, --model <model-file> Visualize data based on a model.
  -a <layout>, --layout <layout> Visualize orientations in a circle, line, etc.
  -r <rotations>, --rotate <rotations> Rotate orientations using specified angles.

Description of subcommands:
  model  Visualize a model, and optionally, animate the model using the provided
         states file.
  data   Visualize experimental marker data or body orientation (inertial
         measurement unit) data. Any file type supported by OpenSim's file
         adapters can be provided. If the provided data file contains a
         TimeSeriesTableVec3 (e.g., TRC or C3D), then the
         data are visualized as markers. If the file contains a
         TimeSeriesTableQuaternion, then the data are visualized as body
         orientations using x-y-z triads, and the --model and --layout
         flags can be used.

Examples:
  opensim-cmd viz model lowerlimb.osim
  opensim-cmd viz --geometry C:/MyGeometry model lowerlimb.osim
  opensim-cmd viz model lowerlimb.osim states.sto
  opensim-cmd viz data markers.trc
  opensim-cmd viz data markers.c3d
  opensim-cmd viz data orientations.sto -a circle --r 1.57,0,0
  opensim-cmd viz -g C:/MyGeometry data orientations.sto -m arm.osim
  opensim-cmd --library C:\Plugins\osimMyCustomForce.dll viz model arm.osim
)";
/* Block below was removed from help text to address bug in VisualStudio handling of regular expressions opensim-core issue #2807

Description of options:
  g, geometry  Search for geometry mesh files in this path.
  m, model     If visualizing orientation data and --layout is 'model',
               the orientation data will be visualized according to the default
               pose of this model.
  a, layout    If visualizing orientation data, this option specifies the layout
               of x-y-z triads as either in a 'line' (the default), in a
               'circle', or according to the default pose of a 'model'. If the
               value is 'model', then the --model option must be provided.
  r, rotate    If visualizing orientation data, this option specifies the space
               fixed Euler angles to apply to data specified in radians.
*/

SimTK::Vec3 parseRotationsString(const std::string& rotationString) {
    SimTK::Vec3 returnVec3{0};
    std::stringstream ss(rotationString);
    ss >> returnVec3;
    return returnVec3;
}

int viz(int argc, const char** argv) {

    using namespace OpenSim;

    std::map<std::string, docopt::value> args = OpenSim::parse_arguments(
            HELP_VIZ, { argv + 1, argv + argc },
            true); // show help if requested

    if (args["--geometry"]) {
        ModelVisualizer::addDirToGeometrySearchPaths(
                args["--geometry"].asString());
    }

    if (args["model"].asBool()) {
        Model model(args["<model-file>"].asString());
        if (args["<states-file>"]) {
            TimeSeriesTable states(args["<states-file>"].asString());
            VisualizerUtilities::showMotion(model, states);
        } else {
            VisualizerUtilities::showModel(model);
        }

    } else if (args["data"].asBool()) {

        const std::string dataFileName = args["<data-file>"].asString();
        auto tables = FileAdapter::createAdapterFromExtension(dataFileName)
                ->read(dataFileName);

        OPENSIM_THROW_IF(tables.size() < 1, Exception,
                         "<data-file> provided zero tables.");

        std::shared_ptr<AbstractDataTable> table;
        if (tables.count("markers")) {
            table = tables.at("markers");
        } else {
            table = tables.begin()->second;
        }

        using TableVec3 = TimeSeriesTableVec3;
        using TableQuat = TimeSeriesTableQuaternion;
        if (auto tableVec3 = std::dynamic_pointer_cast<TableVec3>(table)) {
            log_info("Detected TableSeriesTableVec3...visualizing as "
                     "marker data.");
            if (args["--layout"]) {
                log_warn("Argument --layout is ignored when visualizing marker "
                         "data.");
            }
            if (args["--model"]) {
                log_warn("Argument --model is ignored when visualizing marker "
                         "data.");
            }
            VisualizerUtilities::showMarkerData(*tableVec3);

        } else if (auto tableQuat = std::dynamic_pointer_cast<TableQuat>(table))
        {
            log_info("Detected TableSeriesTableQuaternion...visualizing as "
                     "orientation data.");

            std::unique_ptr<Model> model;
            if (args["--model"]) {
                OPENSIM_THROW_IF(
                        !args["--layout"] ||
                                args["--layout"].asString() != "model",
                        Exception,
                        "--model provided but --layout=model not provided.");
            }

            std::string layout = "line";
            if (args["--layout"]) {
                layout = args["--layout"].asString();
            }
            if (args["--rotate"]) {
                std::string rotationString = "";
                rotationString = args["--rotate"].asString();
                const SimTK::Vec3& sensor_to_opensim_rotations =
                        parseRotationsString(rotationString);
                SimTK::Rotation sensorToOpenSim = SimTK::Rotation(
                        SimTK::BodyOrSpaceType::SpaceRotationSequence,
                        sensor_to_opensim_rotations[0], SimTK::XAxis,
                        sensor_to_opensim_rotations[1], SimTK::YAxis,
                        sensor_to_opensim_rotations[2], SimTK::ZAxis);
                // Rotate data
                OpenSenseUtilities::rotateOrientationTable(
                        *tableQuat, sensorToOpenSim);
            }
            if (layout == "model") {
                OPENSIM_THROW_IF(!args["--model"], Exception,
                        "Expected --model argument if --layout=model, but "
                        "--model was not provided.");
                model.reset(new Model(args["--model"].asString()));
                model->initSystem();
            }

            VisualizerUtilities::showOrientationData(
                    *tableQuat, layout, model.get());
        } else {
            OPENSIM_THROW(Exception,
                          "Expected the <data-file> to contain a "
                          "TimeSeriesTableVec3 or TimeSeriesTableQuaternion, "
                          "but the <data-file> contained a different type of "
                          "table.");
        }

    } else {
        OPENSIM_THROW(Exception, "Subcommand 'model' or 'data' not provided.");
    }
    return EXIT_SUCCESS;
}


#endif // OPENSIM_CMD_VIZ_H_
