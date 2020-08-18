/* --------------------------------------------------------------------------*
*                                  opensense                                 *
* -------------------------------------------------------------------------- *
* Command line application for running an IMUInverseKinematicsTool with IMU  *
* supplied as quaternions and registered onto a subject via labeled markers  *
* OR, by registering IMU rotations on to a model in the calibration pose.    *
*                                                                            *
* Developed by AMJR Consulting under a contract and in a                     *
* collaborative effort with The Johns Hopkins University Applied Physics     *
* Laboratory for a project sponsored by the United States Army Natick Soldier*
* Research Development and Engineering Center and supported by a the NAVAL   *
* SEA SYSTEMS COMMAND (NAVSEA) under Contract No. N00024-13-D-6400,          *
* Task Order #VKW02. Any opinions, findings, conclusions or recommendations  *
* expressed in this material are those of the author(s) and do not           *
* necessarily reflect the views of NAVSEA.                                   *
*                                                                            *
* Copyright (c) 2017-2019 AMJR Consulting and the Authors                    *
* Author(s): Ajay Seth & Ayman Habib                                         *
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

// INCLUDES
#include <cctype>
#include <string>
#include <OpenSim/version.h>
#include <OpenSim/Common/IO.h>
#include <OpenSim/Common/LoadOpenSimLibrary.h>
#include <OpenSim/Common/STOFileAdapter.h>
#include <OpenSim/Common/APDMDataReader.h>
#include <OpenSim/Common/XsensDataReader.h>
#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Simulation/MarkersReference.h>
#include <OpenSim/Simulation/InverseKinematicsSolver.h>
#include <OpenSim/Simulation/OpenSense/OpenSenseUtilities.h>
#include <OpenSim/Tools/IMUInverseKinematicsTool.h>

#include <ctime>  // clock(), clock_t, CLOCKS_PER_SEC

using namespace std;
using namespace OpenSim;
using namespace SimTK;

void addImuFramesFromMarkers(const string& modelFile, const string& markerFile);

TimeSeriesTable_<SimTK::Quaternion> readRotationsFromXSensFiles(const std::string& directory,
    const std::string& readerSetupFile);

TimeSeriesTable_<SimTK::Quaternion> readRotationsFromAPDMFile(const std::string& file,
    const std::string& readerSetupFile);

static void PrintUsage(const char *aProgName, ostream &aOStream);
//______________________________________________________________________________
/**
*  Program to perform IMU-based IK
*
* @param argc Number of command line arguments (should be 1).
* @param argv Command line arguments:  simmReadXML inFile
*/
int main(int argc, char **argv)
{
    //----------------------
    // Surrounding try block
    //----------------------
    try {
        //----------------------
        // PARSE COMMAND LINE
        string option = "";
        string setupFileName;
        if (argc < 2)
        {
            PrintUsage(argv[0], cout);
            exit(-1);
        }
        else {// Don't know maybe user needs help or have special needs
              // Load libraries first
            LoadOpenSimLibraries(argc, argv);

            int i;
            for (i = 1; i <= (argc - 1); i++) {
                option = argv[i];

                // PRINT THE USAGE OPTIONS
                if ((option == "-help") || (option == "-h") || (option == "-Help") || (option == "-H") ||
                    (option == "-usage") || (option == "-u") || (option == "-Usage") || (option == "-U")) {
                    PrintUsage(argv[0], cout);
                    return 0;
                }
                else if ((option == "-ReadXsens") || (option == "-RX")) {
                    if (argc < 4) {
                        log_error("Both the directory containing Xsens data "
                                  "files and the reader settings file are "
                                  "necessary to read Xsens data. Please retry "
                                  "with these inputs.");
                        PrintUsage(argv[0], cout);
                        exit(-1);
                    }
                    std::string directory{ argv[i + 1] };
                    std::string settingsFile{ argv[i + 2] };
                    TimeSeriesTable_<SimTK::Quaternion> rotationsTable =
                                                        readRotationsFromXSensFiles(directory, settingsFile);
                    log_info("Done.");
                    return 0;
                }
                else if ((option == "-ReadAPDM") || (option == "-RA")) {
                    if (argc < 4) {
                        log_error("Both the data file (.csv) with APDM "
                                  "formatted data and the reader settings "
                                  "file are necessary to read APDM data. "
                                  "Please retry with these inputs.");
                        PrintUsage(argv[0], cout);
                        exit(-1);
                    }
                    std::string dataFile{ argv[i + 1] };
                    std::string settingsFile{ argv[i + 2] };
                    TimeSeriesTable_<SimTK::Quaternion> rotationsTable =
                        readRotationsFromAPDMFile(dataFile, settingsFile);
                    log_info("Done.");
                    return 0;
                }
                else if ((option == "-Transform") || (option == "-T")) {
                    if (argc < 3) {
                        log_error("Marker file is needed for this option. "
                                  "Please fix and retry.");
                        PrintUsage(argv[0], cout);
                        exit(-1);
                    }                   
                    std::string markerFile{ argv[i + 1] };
                    TimeSeriesTable_<SimTK::Quaternion> quaternions =
                        OpenSenseUtilities::createOrientationsFileFromMarkers(markerFile);
                    log_info("Done.");
                    return 0;
                }
                else if ((option == "-AddIMUs") || (option == "-A")) {
                    if (argc < 4) {
                        log_error("Both a model (.osim) file and marker data "
                                  "(e.g. .trc) file are necessary to add IMU "
                                  "frames to the model based-on marker data.");
                        PrintUsage(argv[0], cout);
                        exit(-1);
                    }
                    std::string modelFile{ argv[i + 1] };
                    std::string markersFile{ argv[i + 2] };
                    addImuFramesFromMarkers(modelFile, markersFile);

                    log_info("Done.");
                    return 0;
                }
                else if ((option == "-Calibrate") || (option == "-C")) {
                    if (argc < 3) {
                        log_error("Calibration specification file is needed. "
                                  "Please fix and retry.");
                        PrintUsage(argv[0], cout);
                        exit(-1);
                    }
                    std::string modelCalibrationSetupFile{ argv[i + 1] };
                    IMUPlacer imuPlacer{modelCalibrationSetupFile};
                    imuPlacer.run();
                    Model model = imuPlacer.getCalibratedModel();
                    // If output_model was specified it will be written, otherwise preserve model by writing to "calibrated_"
                    if (imuPlacer.get_output_model_file().empty()) {
                        auto filename =
                                model.getName() + "_calibrated" + ".osim";
                        log_info("Wrote calibrated model to file: {}.",
                                 filename);
                        model.print(filename);
                    }
                    log_info("Done.");
                    return 0;
                }
                else if ((option == "-InverseKinematics") || (option == "-IK")) {
                    if (argc < 3) {
                        log_error("An inverse kinematics settings (.xml) file "
                                  "was expected but no file was provided.");
                        PrintUsage(argv[0], cout);
                        exit(-1);
                    }
                    setupFileName = argv[i + 1];
                    break;
                }
                else if ((option == "-PrintSetup") || (option == "-PS")) {
                    IMUInverseKinematicsTool *imuIKTool = new IMUInverseKinematicsTool();
                    imuIKTool->setName("new");
                    Object::setSerializeAllDefaults(true);
                    std::string setupFile = 
                        "new_Setup_IMUInverseKinematicsTool.xml";
                    imuIKTool->print(setupFile);
                    Object::setSerializeAllDefaults(false);
                    log_info("Created file {} with default setup.", setupFile);
                    return 0;

                    // PRINT PROPERTY INFO
                }
                else if ((option == "-PropertyInfo") || (option == "-PI")) {
                    if ((i + 1) >= argc) {
                        Object::PrintPropertyInfo(cout, "");

                    }
                    else {
                        char *compoundName = argv[i + 1];
                        if (compoundName[0] == '-') {
                            Object::PrintPropertyInfo(cout, "");
                        }
                        else {
                            Object::PrintPropertyInfo(cout, compoundName);
                        }
                    }
                    return(0);

                    // UNRECOGNIZED
                }
                else {
                    log_warn("Unrecognized option {} on command line... "
                              "Ignored", option);
                    PrintUsage(argv[0], cout);
                    return(0);
                }
            }
        }

        // ERROR CHECK
        if (setupFileName == "") {
            log_error("opensense.exe: A setup file must be specified.");
            PrintUsage(argv[0], cout);
            return(-1);
        }

        // CONSTRUCT
        log_info("Constructing tool from setup file {}.", setupFileName);
        IMUInverseKinematicsTool ik(setupFileName);

        // start timing
        std::clock_t startTime = std::clock();

        // RUN
        ik.run();

        auto timeInMilliseconds = 1.e3*(std::clock() - startTime) / CLOCKS_PER_SEC;
        log_info("opensense compute time = {} ms", timeInMilliseconds);

        //----------------------------
        // Catch any thrown exceptions
        //----------------------------
    }
    catch (const std::exception& x) {
        log_error("Exception in opensense: {}", x.what());
        return -1;
    }
    //----------------------------

    return(0);
}

//_____________________________________________________________________________
/**
* Print the usage for this application
*/
void PrintUsage(const char *aProgName, ostream &aOStream)
{
    string progName = IO::GetFileNameFromURI(aProgName);
    aOStream << "\n\n" << progName << ":\n\n";
    aOStream << "Option             Argument               Action / Notes\n";
    aOStream << "------             --------               --------------\n";
    aOStream << "-Help, -H                                 Print the command-line options for " << progName << ".\n";
    aOStream << "-PrintSetup, -PS                          Create a template inverse kinematics settings file that can be customized.\n";
    aOStream << "-PropertyInfo, -PI                        Print help information for properties in setup files.\n";
    aOStream << "-ReadXsens, -RX directory settings.xml    Parse Xsens exported files from directory using settings.xml.\n";
    aOStream << "                                          Creates a storage file with the orientation data for each sensor, \n";
    aOStream << "                                          where each column in the storage file is named\n"; 
    aOStream << "                                          according to the Frame in the corresponding OpenSim model.\n";
    aOStream << "-ReadAPDM, -RA datafile.csv settings.xml  Parse single csv file provided by APDM using specified settings.xml.\n";
    aOStream << "                                          Creates a storage file with the orientation data for each sensor, \n";
    aOStream << "                                          where each column in the storage file is named\n";
    aOStream << "                                          according to the Frame in the corresponding OpenSim model.\n";
    aOStream << "-Calibrate, -C IMUPlacer_setup.xml        Place IMUs on the model that is specified in the IMUPlacer_setup.xml file.\n";
    aOStream << "                                          The model is positioned in its default pose. IMUs are then registered to the model according to\n ";
    aOStream << "                                          their orientations in the first frame of the quaternions file that is specified in IMUPlacer_setup.xml.\n";
    aOStream << "                                          The orientations of the IMUs in the quaternions file are assumed to be in the IMU world frame.\n";
    aOStream << "                                          The resultant model with IMU frames registered is written to file if output_model_file is specified\n";
    aOStream << "                                          in IMUPlacer_setup.xml. Additional options can be specified in IMUPlacer_setup.xml to perform heading correction.\n";
    aOStream << "-InverseKinematics, -IK ik_settings.xml   Run IK using an xml settings file to define the inverse kinematics problem.\n";
    aOStream << endl;
/** Advanced options for experimental validation. Uncomment if/when ready to make public
    aOStream << "-Transform, -T markerFileWithIMUframes.trc  Transform experimental marker locations that define axes of IMUs, or the plates\n";
    aOStream << "                                        upon which they are rigidly affixed, into the orientations of the IMUs expressed \n";
    aOStream << "                                        in the motion capture(markers) lab frame.The orientations over the trial are \n";
    aOStream << "                                        written out as quaternions to <markerFileWithIMUframes>_quaternions.sto.\n";
    aOStream << "                                        Markers on IMUs(or plates) are assumed to be labeled:\n";
    aOStream << "                                        '<base>_IMU_O', '<base>_IMU_X', '<base>_IMU_Y', '<base>_IMU_D', \n";
    aOStream << "                                        where <base> is the label of the IMU, and O, X, Y and D denote\n";
    aOStream << "                                        the origin, x - axis direction, y - axis direction and diagonal, respectively.\n";
    aOStream << "                                        The resulting quaternions file as <base>_IMU as its column labels.\n";
    aOStream << "-AddIMUs, -A modelFile.osim markerFileWithIMUframes.trc Add IMUs to the provided model based on marker data.\n";
    aOStream << "                                        Marker data with the naming convention of '<base>_IMU_O', '<base>_IMU_X',\n";
    aOStream << "                                        '<base>_IMU_Y', '<base>_IMU_D' are used to compute the location \n";
    aOStream << "                                        and orientation the IMU frame which is then affixed to the same base segment(frame) \n";
    aOStream << "                                        to which the markers are attached.Before the IMU frames are attached to the model, \n";
    aOStream << "                                        the model is posed according to marker - based IK.\n";
**/
}

TimeSeriesTable_<SimTK::Quaternion> readRotationsFromXSensFiles(const std::string& directory,
                                                    const std::string& readerSetupFile)
{
    XsensDataReaderSettings readerSettings(readerSetupFile);
    XsensDataReader reader(readerSettings);
    DataAdapter::OutputTables tables = reader.read(directory);
    const TimeSeriesTableQuaternion& quatTableTyped =  reader.getOrientationsTable(tables);

    STOFileAdapter_<SimTK::Quaternion>::write(quatTableTyped, readerSettings.get_trial_prefix()+"_orientations.sto");
 
    return quatTableTyped;
}


TimeSeriesTable_<SimTK::Quaternion> readRotationsFromAPDMFile(const std::string& apdmCsvFile,
    const std::string& readerSetupFile)
{
    APDMDataReaderSettings readerSettings(readerSetupFile);
    APDMDataReader reader(readerSettings);
    DataAdapter::OutputTables tables = reader.read(apdmCsvFile);
    const TimeSeriesTableQuaternion& quatTableTyped = reader.getOrientationsTable(tables);

    STOFileAdapter_<SimTK::Quaternion>::write(quatTableTyped, 
        apdmCsvFile.substr(0, apdmCsvFile.rfind('.'))+"_orientations.sto");

    return quatTableTyped;
}

void addImuFramesFromMarkers(const string& modelFile, const string& markersFile)
{
    Model model{ modelFile };
    model.updForceSet().clearAndDestroy();

    TimeSeriesTableVec3 table =
        IMUInverseKinematicsTool::loadMarkersFile(markersFile);

    model.setUseVisualizer(true);

    State& s = model.initSystem();
    model.realizePosition(s);

    auto& times = table.getIndependentColumn();
    //auto averageRow = table->averageRow(*times.cbegin(), *times.cend());
    const auto referenceRow = table.getRowAtIndex(0);

    std::shared_ptr<MarkersReference> markersRef(new MarkersReference(table, Set<MarkerWeight>()));

    // create the IK solver based on markers only to get the static pose
    SimTK::Array_<CoordinateReference> coordinateReferences;

    InverseKinematicsSolver ikSolver(model, markersRef, coordinateReferences);
 
    s.updTime() = times[0];

    ikSolver.assemble(s);
    model.getVisualizer().show(s);

    // labels of markers including those <bodyName>O,X,Y that identify the 
    // IMU sensor placement/alignment on the body expressed in Ground
    auto labels = table.getColumnLabels();

    //auto bodies = model.getComponentList<OpenSim::Body>();
    auto markers = model.getComponentList<OpenSim::Marker>();

    std::vector<PhysicalOffsetFrame*> offsets;
    std::vector<PhysicalFrame*> bodies;

    size_t index = 0;
    Vec3 op, xp, yp, dp;

    for (auto& marker : markers) {
        const PhysicalFrame& parent = marker.getParentFrame();
        // max one imu on per body for now
        auto it = std::find(bodies.begin(), bodies.end(), &parent);
        if (it != bodies.end()) {
            continue;
        }

        auto ix = marker.getName().find("_IMU");
        string base = marker.getName().substr(0, ix);
        cout << "Processing marker " << marker.getName() << endl;

        op = xp = yp = dp = Vec3{ SimTK::NaN };
        for (auto& label : labels) {
            if (table.hasColumn(base + "_IMU_O")) {
                index = table.getColumnIndex(base + "_IMU_O");
                op = referenceRow[unsigned(index)];
            }
            if (table.hasColumn(base + "_IMU_X")) {
                index = table.getColumnIndex(base + "_IMU_X");
                xp = referenceRow[unsigned(index)];
            }
            if (table.hasColumn(base + "_IMU_Y")) {
                index = table.getColumnIndex(base + "_IMU_Y");
                yp = referenceRow[unsigned(index)];
            }
            if (table.hasColumn(base + "_IMU_D")) {
                index = table.getColumnIndex(base + "_IMU_D");
                dp = referenceRow[unsigned(index)];
            }
        }

        cout << base << " O:" << op << ", X:" << xp << ", Y:" << yp
            << ", D:" << dp << endl;

        if (op.isNaN() || xp.isNaN() || yp.isNaN()) {
            cout << "marker " << marker.getName() <<
                " is NaN and cannot be used to define IMU on " <<
                marker.getParentFrame().getName() << endl;
        }
        else {
            // Transform of the IMU formed from markers expressed in Ground
            auto X_FG = OpenSenseUtilities::formTransformFromPoints(op, xp, yp);
            // update origin location to centroid of marker points on IMU plate
            X_FG.updP() = (op + xp + yp + dp) / 4.0;

            // Transform of the body in Ground
            auto& X_BG = marker.getParentFrame().getTransformInGround(s);
            cout << "X_BG: " << X_BG << endl;

            // Transform of the IMU frame in the Body
            auto X_FB = ~X_BG*X_FG;
            cout << "X_FB: " << X_BG << endl;

            auto imuOffset =
                new PhysicalOffsetFrame(IO::Lowercase(base) + "_imu",
                    marker.getParentFrame(), X_FB);
            auto* brick = new Brick(Vec3(0.02, 0.01, 0.005));
            brick->setColor(SimTK::Orange);
            imuOffset->attachGeometry(brick);

            offsets.push_back(imuOffset);

            cout << "IMU on frame " << parent.getName() << " done." << endl;
            PhysicalFrame& body = model.updComponent<PhysicalFrame>(parent.getAbsolutePath());
            bodies.push_back(&body);
        }
    }

    // store joint initial pose from marker IK as default pose for the model. 
    model.setPropertiesFromState(s);

    for (int i = 0; i < (int)offsets.size(); ++i) {
        // add imu offset frames to the model with model taking ownership
        bodies[i]->addComponent(offsets[i]);
    }

    model.finalizeConnections();

    auto ix = markersFile.rfind("\\") + 1;
    auto jx = markersFile.rfind(".");

    string suffix = markersFile.substr(ix, jx - ix) + "_IMUs";

    model.setName(model.getName() + "_" + suffix);
    model.print(model.getName() + ".osim");
    std::cout << std::endl;
}
