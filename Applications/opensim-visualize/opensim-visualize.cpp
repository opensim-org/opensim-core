/* -------------------------------------------------------------------------- *
 *                    OpenSim:  VisualizeModel.cpp                            *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2020 Stanford University and the Authors                *
 * Author(s): Ayman Habib                                                     *
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

/* 
 *  Below is an example of an OpenSim application that loads an OpenSim model
 *  and visualize it in the API visualizer.
 */

// Author:  Ayman Habib

//==============================================================================
//==============================================================================
#include <OpenSim/Common/IO.h>
#include <OpenSim/Simulation/VisualizerUtilities.h>
#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Common/LoadOpenSimLibrary.h>
#include <OpenSim/Simulation/OpenSense/OpenSenseUtilities.h>

using namespace OpenSim;
using namespace SimTK;
using namespace std;

SimTK::Vec3 parseRotationsString(const std::string& rotationString) {
    Vec3 returnVec3{0};
    std::stringstream ss(rotationString);
    ss >> returnVec3;
    return returnVec3;
}

static void PrintUsage(const char* aProgName, ostream& aOStream);
//______________________________________________________________________________
/**
 * First exercise: create a model that does nothing. 
 */
int main(int argc, char** argv) {
    LoadOpenSimLibrary("osimActuators");
    try {
        // Create an OpenSim model and set its name
        if (argc < 2) {
            PrintUsage(argv[0], cout);
            return 1;
        }
        string option = argv[1];
        if ((option == "-help") || (option == "-h") || (option == "-Help") ||
                (option == "-H") || (option == "-usage") || (option == "-u") ||
                (option == "-Usage") || (option == "-U")) {
            PrintUsage(argv[0], cout);
            return 0;
        } else {
            string modelFileName = "";
            string geomertySearchPath = "";
            string dataFileName = "";
            string statesFileName = "";
            string layout = "model";
            string rotationString = "";
            for (int i = 1; i <= (argc - 1); i++) {
                option = argv[i];
                if (option == "-M" || option == "-Model") {
                    if (argc < i + 2) {
                        cout << "No model file specified after option -M";
                        PrintUsage(argv[0], cout);
                        exit(-1);
                    }
                    modelFileName = string(argv[i + 1]);
                }
                else if (option == "-G" || option == "-Geom") {
                    if (argc < i + 2) {
                        cout << "No Geometry search path specified after option -G";
                        PrintUsage(argv[0], cout);
                        exit(-1);
                    }
                    geomertySearchPath = string(argv[i + 1]);
                }
                else if (option == "-S") { 
                     if (argc < i + 2) {
                        cout << "No sto file specified after option -S";
                        PrintUsage(argv[0], cout);
                        exit(-1);
                    }
                    statesFileName = string(argv[i + 1]); 
                }
                else if (option == "-D") { 
                      if (argc < i + 2) {
                        cout << "No data file specified after option -D";
                        PrintUsage(argv[0], cout);
                        exit(-1);
                    }
                    dataFileName = string(argv[i + 1]); 
                } else if (option == "-A") {
                    if (argc < i + 2) {
                        cout << "No layout specified after option -A";
                        PrintUsage(argv[0], cout);
                        exit(-1);
                    }
                    layout = string(argv[i + 1]);
                }
                if (option == "-R") {
                    if (argc < i + 2) {
                        cout << "No Rotation specified after option -R";
                        PrintUsage(argv[0], cout);
                        exit(-1);
                    }
                    rotationString = string(argv[i + 1]);
                }
            }
            option = argv[1];
            // Based on first argv will decide operation
            if (option == "-VM" || option == "-ViewModel") {

                // If Geometry is located in folders other than
                // where the .osim file is located or in a Geometry
                // folder adjacent to the model file, use additional
                // geomertySearchPath for the folder containing
                // geometry mesh files
                if (geomertySearchPath != "")
                    ModelVisualizer::addDirToGeometrySearchPaths(
                            geomertySearchPath);
                Model m(modelFileName);
                VisualizerUtilities::showModel(m);

            } else if (option == "-VS"){

                if (geomertySearchPath != "")
                    ModelVisualizer::addDirToGeometrySearchPaths(
                            geomertySearchPath);
                Model m(modelFileName);
                Storage storage(statesFileName);
                VisualizerUtilities::showMotion(m, storage);
                
            } else if ((option == "-VD") || (option == "-ViewData")) {

                std::string::size_type extSep = dataFileName.rfind(".");
                if (extSep == std::string::npos) {
                    throw OpenSim::Exception("Input file '" +
                                             string(dataFileName) +
                                             "' does not have an extension.");
                }
                std::string extension = dataFileName.substr(extSep);
                if (extension == ".sto") {

                    TimeSeriesTableQuaternion quatTable(dataFileName);
                    if (rotationString != "") { 
                        const SimTK::Vec3& sensor_to_opensim_rotations =
                                parseRotationsString(rotationString);
                        SimTK::Rotation sensorToOpenSim = SimTK::Rotation(
                                SimTK::BodyOrSpaceType::SpaceRotationSequence,
                                sensor_to_opensim_rotations[0], SimTK::XAxis,
                                sensor_to_opensim_rotations[1], SimTK::YAxis,
                                sensor_to_opensim_rotations[2], SimTK::ZAxis);
                        // Rotate data so Y-Axis is up
                        OpenSenseUtilities::rotateOrientationTable(
                                quatTable, sensorToOpenSim);
                    }
                    Model model(modelFileName);
                    model.initSystem();
                    VisualizerUtilities::showOrientationData(
                            quatTable, layout, &model);

                } else if (extension == ".trc") {

                    TimeSeriesTableVec3 markerData(dataFileName);
                    VisualizerUtilities::showMarkerData(markerData);

                } else {
                    cout << "Unrecognized data file " << dataFileName
                         << "on command line... Ignored" << endl;
                    PrintUsage(argv[0], cout);
                    return (0);
                }

            } else {
                cout << "Unrecognized option " << option
                     << " on command line... Ignored" << endl;
                PrintUsage(argv[0], cout);
                return (0);
            }
        }
    }
    catch (const std::exception& ex)
    {
        std::cout << ex.what() << std::endl;
        return 1;
    }
    catch (...)
    {
        std::cout << "UNRECOGNIZED EXCEPTION" << std::endl;
        return 1;
    }

    return 0;
}
//_____________________________________________________________________________
/**
 * Print the usage for this application
 */
void PrintUsage(const char* aProgName, ostream& aOStream) {
    string progName = IO::GetFileNameFromURI(aProgName);
    aOStream << "\n\n" << progName << ":\n\n";
    aOStream << "Option             Argument                            Action / Notes\n";
    aOStream << "------             --------                            --------------\n";
    aOStream << "-Help, -H                                              "
                "Print the command-line options for " << progName << ".\n";
    aOStream << "-VM,-ViewModel -M model.osim -G geomSearchPath         "
                "Visualize model from file, with optional geometry search path.\n";
    aOStream << "-VS -M model.osim -G geomSearchPath -S states.sto      "
                "Visualize model with optional geometry search path, \n"
                "                                                       "
                "and apply states from the specified states.sto file.\n";
    aOStream << "-VD,-ViewData -M mdl.osim -A layout -D file.{sto,trc}  "
                "Visualize data from mocap (.trc) or orientations(.sto)\n"
                "                                                       "
                "layout is one of {line, circle, model}, If model      \n"
                "                                                       "
                "is specified, it's used to layout data on screen.     \n";
}