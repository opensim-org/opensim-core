#ifndef OPENSIM_CMD_VISUALIZE_H_
#define OPENSIM_CMD_VISUALIZE_H_
/* -------------------------------------------------------------------------- *
 *                      OpenSim:  opensim-cmd_visualize.h                     *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2017 Stanford University and the Authors                *
 * Author(s): Frank C. Anderson, Ayman Habib, Chris Dembia                    *
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

static const char HELP_VISUALIZE[] =
R"(Show a model, motion, or marker/orientation data with the Simbody Visualizer.

Usage:
  opensim-cmd [options]... visualize [--geometry=<path>]
                                model <model-file> [<states-file>]
  opensim-cmd [options]... visualize [--geometry=<path>]
                                markers <data-file>
  opensim-cmd [options]... visualize [--geometry=<path>]
                                orientations <data-file> [--layout=<layout>]
  opensim-cmd visualize -h | --help

Options:
  -L <path>, --library <path>  Load a plugin.
  -g <path>, --geometry <path> Search for geometry mesh files in this path.
  -a <layout>, --layout <layout> TODO.

Description:
  TODO

Examples:
  opensim-cmd visualize model lowerlimb.osim
  opensim-cmd visualize --geometry C:/MyGeometry model lowerlimb.osim
  opensim-cmd visualize model lowerlimb.osim states.sto
  opensim-cmd visualize markers markers.trc
  opensim-cmd visualize markers markers.c3d
  opensim-cmd visualize orientations orientations.sto -a line
  opensim-cmd visualize -g C:/MyGeometry
                    orientations orientations.sto -a model.osim
  opensim-cmd --library C:\Plugins\osimMyCustomForce.dll visualize arm.osim
)";

int visualize(int argc, const char** argv) {

    using namespace OpenSim;

    std::map<std::string, docopt::value> args = OpenSim::parse_arguments(
            HELP_VISUALIZE, { argv + 1, argv + argc },
            true); // show help if requested

    if (args["--geometry"]) {
        ModelVisualizer::addDirToGeometrySearchPaths(
                args["--geometry"].asString());
    }

    if (args["model"].asBool()) {
        Model model(args["<model-file>"].asString());
        if (args["<states-file>"]) {
            Storage states(args["<states-file>"].asString());
            VisualizerUtilities::showMotion(model, states);
        } else {
            VisualizerUtilities::showModel(model);
        }

    } else if (args["markers"].asBool()) {
        const std::string dataFileName = args["<data-file>"].asString();
        auto tables = FileAdapter::createAdapterFromExtension(dataFileName)
                ->read(dataFileName);

        // This ideally supports reading markers from C3D files.
        OPENSIM_THROW_IF(tables.size() < 1, Exception,
                         "<data-file> provided zero tables.");

        std::shared_ptr<AbstractDataTable> table;
        if (tables.count("markers")) {
            table = tables.at("markers");
        } else {
            table = tables.begin()->second;
        }

        auto tableVec3 = std::dynamic_pointer_cast<TimeSeriesTableVec3>(table);
        OPENSIM_THROW_IF(!tableVec3, Exception,
                      "Could not load <data-file> as TimeSeriesTableVec3.");

        VisualizerUtilities::showMarkerData(*tableVec3);

    } else if (args["orientations"].asBool()) {

        TimeSeriesTableQuaternion table(args["<data-file>"].asString());

        std::unique_ptr<Model> model;

        std::string layout = "line";
        if (args["--layout"]) {
            layout = args["--layout"].asString();
            if (IO::GetSuffix(layout, 5) == ".osim") {
                model.reset(new Model(layout));
                layout = "model";
            }
        }

        VisualizerUtilities::showOrientationData(table, layout, model.get());

    } else {
        OPENSIM_THROW(Exception, "Subcommand 'model' or 'data' not provided.");
    }
    return EXIT_SUCCESS;
}


#endif // OPENSIM_CMD_VISUALIZE_H_
