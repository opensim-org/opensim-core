/* --------------------------------------------------------------------------*
*                                  opensense                                 *
* -------------------------------------------------------------------------- *
* Command line application for running an InverseKinematicsStudy with IMU    *
* supplied as quaternions and registered onto a subject via labeled markers  *
* OR, by registering IMU rotations on to a model in the calibration pose.    *
*                                                                            *
* Developed by AMJR Consulting for NSRDEC under subcontract No. XXXXXXXXX,   *
* The Johns Hopkins University Applied Physics Laboratory (APL)              *
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
#include <OpenSim/Simulation/OpenSense/InverseKinematicsStudy.h>
#include <OpenSim/Simulation/OpenSense/OpenSenseUtilities.h>

#include <ctime>  // clock(), clock_t, CLOCKS_PER_SEC

using namespace std;
using namespace OpenSim;
using namespace SimTK;

void addImuFramesFromMarkers(const string& modelFile, const string& markerFile);

SimTK::Transform formTransformFromPoints(const SimTK::Vec3& op,
    const SimTK::Vec3& xp,
    const SimTK::Vec3& yp);

TimeSeriesTable_<SimTK::Quaternion> readRotationsFromXSensFiles(const std::string& directory,
    const std::string& readerSetupFile);

TimeSeriesTable_<SimTK::Quaternion>
    createOrientationsFileFromMarkers(const std::string& markerFile);

void calibrateModelFromOrientaions(
    const std::string& modelCalibrationPoseFile,
    const std::string& calibrationOrientationsFile);

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
                else if ((option == "-Read") || (option == "-R")) {
                    std::string directory{ argv[i + 1] };
                    std::string settingsFile{ argv[i + 2] };
                    TimeSeriesTable_<SimTK::Quaternion> rotationsTable =
                                                        readRotationsFromXSensFiles(directory, settingsFile);
                    cout << "Done." << endl;
                    return 0;
                }
                else if ((option == "-Transform") || (option == "-T")) {
                    std::string markerFile{ argv[i + 1] };
                    TimeSeriesTable_<SimTK::Quaternion> quaternions =
                        createOrientationsFileFromMarkers(markerFile);
                    cout << "Done." << endl;
                    return 0;
                }
                else if ((option == "-AddIMUs") || (option == "-A")) {
                    std::string modelFile{ argv[i + 1] };
                    std::string markersFile{ argv[i + 2] };
                    addImuFramesFromMarkers(modelFile, markersFile);

                    cout << "Done." << endl;
                    return 0;
                }
                else if ((option == "-Calibrate") || (option == "-C")) {
                    std::string modelCalibrationPoseFile{ argv[i + 1] };
                    std::string calibrationOrientationsFile{ argv[i + 2] };
                    calibrateModelFromOrientaions( modelCalibrationPoseFile,
                                                   calibrationOrientationsFile);

                    cout << "Done." << endl;
                    return 0;
                }
                else if ((option == "-Setup") || (option == "-S")) {
                    setupFileName = argv[i + 1];
                    break;
                }
                else if ((option == "-PrintSetup") || (option == "-PS")) {
                    InverseKinematicsStudy *study = new InverseKinematicsStudy();
                    study->setName("new");
                    Object::setSerializeAllDefaults(true);
                    study->print("new_Setup_IMUtracking.xml");
                    Object::setSerializeAllDefaults(false);
                    cout << "Created file new_Setup_IMUtracking.xml with default setup" << endl;
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
                    cout << "Unrecognized option" << option << "on command line... Ignored" << endl;
                    PrintUsage(argv[0], cout);
                    return(0);
                }
            }
        }

        // ERROR CHECK
        if (setupFileName == "") {
            cout << "\n\nopensense.exe: ERROR- A setup file must be specified.\n";
            PrintUsage(argv[0], cout);
            return(-1);
        }

        // CONSTRUCT
        cout << "Constructing tool from setup file " << setupFileName << ".\n\n";
        InverseKinematicsStudy ik(setupFileName);

        // start timing
        std::clock_t startTime = std::clock();

        // RUN
        ik.run();

        std::cout << "opensense compute time = " << 1.e3*(std::clock() - startTime) / CLOCKS_PER_SEC << "ms\n";


        //----------------------------
        // Catch any thrown exceptions
        //----------------------------
    }
    catch (const std::exception& x) {
        cout << "Exception in opensense: " << x.what() << endl;
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
    aOStream << "Option             Argument         Action / Notes\n";
    aOStream << "------             --------         --------------\n";
    aOStream << "-Help, -H                           Print the command-line options for " << progName << ".\n";
    aOStream << "-PrintSetup, -PS                    Generates a template Setup file to customize the scaling\n";
    aOStream << "-Setup, -S         SetupFileName    Specify an xml setup file for solving an inverse kinematics problem.\n";
    aOStream << "-PropertyInfo, -PI                  Print help information for properties in setup files.\n";
}

TimeSeriesTable_<SimTK::Quaternion> readRotationsFromXSensFiles(const std::string& directory,
                                                    const std::string& readerSetupFile)
{
    XsensDataReaderSettings readerSettings(readerSetupFile);
    XsensDataReader reader(readerSettings);
    DataAdapter::OutputTables tables = reader.extendRead(directory);
    const TimeSeriesTableQuaternion& quatTableTyped =  IMUDataUtilities::getOrientationsTable(tables);

    STOFileAdapter_<SimTK::Quaternion>::write(quatTableTyped, "imuOrientations.sto");
 
    return quatTableTyped;
}

SimTK::Transform formTransformFromPoints(const Vec3& op,
                                        const Vec3& xp,
                                        const Vec3& yp)
{
    OPENSIM_THROW_IF(op.isNaN() || xp.isNaN() || yp.isNaN(),
        OpenSim::Exception,
        "formTransformFromPoints: An input point is NaN.");

    UnitVec3 ux{ xp - op };
    UnitVec3 uy{ yp - op };
    UnitVec3 uz{ ux % uy };

    Mat33 nearRot{ ux, uy, uz };
    Rotation R{ nearRot };

    SimTK::Transform X{ R, op };

    return X;
}

TimeSeriesTable_<SimTK::Quaternion> 
    createOrientationsFileFromMarkers(const std::string& markersFile)
{
    TimeSeriesTableVec3 table{ markersFile };

    // labels of markers including those <bodyName>O,X,Y that identify the 
    // IMU sensor placement/alignment on the body expressed in Ground
    auto labels = table.getColumnLabels();

    std::string suffix{ "_IMU" };

    std::vector<std::string> imuLabels;
    std::vector<std::vector<int>> imuIndices;

    auto it = labels.begin();

    size_t nImu = 0;
    size_t index;
    while (it != labels.end()) {
        std::vector<int> indices;
        auto ix = it->find(suffix);
        if (ix > it->size()) {
            it++;
            continue;
        }
        string base = it->substr(0, ix);

        imuLabels.push_back(IO::Lowercase(base + suffix));

        index = table.getColumnIndex(base + suffix + "_O");
        indices.push_back(int(index));
        it++;

        index = table.getColumnIndex(base + suffix + "_X");
        indices.push_back(int(index));
        it++;

        index = table.getColumnIndex(base + suffix + "_Y");
        indices.push_back(int(index));
        it++;

        index = table.getColumnIndex(base + suffix + "_D");
        indices.push_back(int(index));
        it++;

        // Left leg plates have additional marker to differentiate from right side.
        if (table.hasColumn(base + suffix + "_5")) it++;

        imuIndices.push_back(indices);
    }

    const auto& markerData = table.getMatrix();
    SimTK::Matrix_<SimTK::Quaternion> quatData{ markerData.nrow(), int(imuLabels.size()) };

    Vec3 op, xp, yp, dp;
    SimTK::Quaternion quat{ SimTK::NaN, SimTK::NaN, SimTK::NaN, SimTK::NaN };

    for (int row = 0; row < markerData.nrow(); ++row) {
        // reset marker and quaternion to NaN
        op = xp = yp = dp = Vec3{ SimTK::NaN };
        quat *= SimTK::NaN;

        for (size_t i = 0; i < imuLabels.size(); ++i) {
            op = markerData.getElt(row, imuIndices[i][0]);
            xp = markerData.getElt(row, imuIndices[i][1]);
            yp = markerData.getElt(row, imuIndices[i][2]);
            dp = markerData.getElt(row, imuIndices[i][3]);

            if (op.isNaN() || xp.isNaN() || yp.isNaN()) {
                cout << "marker(s) for IMU '" << imuLabels[i] <<
                    "' is NaN and orientation will also be NaN." << endl;
            }
            else {
                // Transform of the IMU formed from markers expressed in Ground
                auto X_FG = formTransformFromPoints(op, xp, yp);
                quat = X_FG.R().convertRotationToQuaternion();
            }
            quatData(row, int(i)) = quat;
        }

    }

    TimeSeriesTable_<SimTK::Quaternion> quaternions{
        table.getIndependentColumn(), quatData, imuLabels };

    auto ix = markersFile.find(".");
    string fileName = markersFile.substr(0, ix) + "_quaternions.sto";

    STOFileAdapter_<SimTK::Quaternion>::write(quaternions, fileName);

    return quaternions;
}


void addImuFramesFromMarkers(const string& modelFile, const string& markersFile)
{
    Model model{ modelFile };
    model.updForceSet().clearAndDestroy();

    TimeSeriesTableVec3 table =
        InverseKinematicsStudy::loadMarkersFile(markersFile);

    model.setUseVisualizer(true);

    State& s = model.initSystem();
    model.realizePosition(s);

    auto& times = table.getIndependentColumn();
    //auto averageRow = table->averageRow(*times.cbegin(), *times.cend());
    const auto referenceRow = table.getRowAtIndex(0);

    MarkersReference markersRef(table, nullptr);

    // create the IK solver based on markers only to get the static pose
    SimTK::Array_<CoordinateReference> coordinateReferences;

    InverseKinematicsSolver ikSolver(model, markersRef,
        coordinateReferences);
 
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
            auto X_FG = formTransformFromPoints(op, xp, yp);
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

    for (int i = 0; i < offsets.size(); ++i) {
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

void calibrateModelFromOrientaions(const string& modelCalibrationPoseFile,
    const string& calibrationOrientationsFile)
{
    Model model(modelCalibrationPoseFile);

    TimeSeriesTable_<SimTK::Quaternion> quatTable =
        STOFileAdapter_<SimTK::Quaternion>::read(calibrationOrientationsFile);

    TimeSeriesTable_<SimTK::Rotation> orientationsData =
        OpenSenseUtilities::convertQuaternionsToRotations(quatTable);

    std::cout << "Loaded orientations as quaternions from "
        << calibrationOrientationsFile << std::endl;

    auto imuLabels = orientationsData.getColumnLabels();
    auto& times = orientationsData.getIndependentColumn();

    // The rotations of the IMUs at the start time in order
    // the labels in the TimerSeriesTable of orientations
    auto rotations = orientationsData.getRowAtIndex(0);

    SimTK::State& s0 = model.initSystem();
    s0.updTime() = times[0];

    // default pose of the model defined by marker-based IK 
    model.realizePosition(s0);

    size_t imuix = 0;
    std::vector<PhysicalFrame*> bodies{ imuLabels.size(), nullptr };
    std::map<std::string, SimTK::Rotation> imuBodiesInGround;

    // First compute the transform of each of the imu bodies in ground
    for (auto& imuName : imuLabels) {
        auto ix = imuName.rfind("_imu");
        if (ix != std::string::npos) {
            auto bodyName = imuName.substr(0, ix);
            auto body = model.findComponent<PhysicalFrame>(bodyName);
            if (body) {
                bodies[imuix] = const_cast<PhysicalFrame*>(body);
                imuBodiesInGround[imuName] =
                    body->getTransformInGround(s0).R();
            }
        }
        ++imuix;
    }

    // Now cycle through each imu with a body and compute the relative
    // offset of the IMU measurement relative to the body and
    // update the modelOffset OR add an offset if none exists
    imuix = 0;
    for (auto& imuName : imuLabels) {
        cout << "Processing " << imuName << endl;
        if (imuBodiesInGround.find(imuName) != imuBodiesInGround.end()) {
            cout << "Computed offset for " << imuName << endl;
            auto R_FB = ~imuBodiesInGround[imuName] * rotations[int(imuix)];
            cout << "Offset is " << R_FB << endl;
            PhysicalOffsetFrame* imuOffset = nullptr;
            const PhysicalOffsetFrame* mo = nullptr;
            if (mo = model.findComponent<PhysicalOffsetFrame>(imuName)) {
                imuOffset = const_cast<PhysicalOffsetFrame*>(mo);
                auto X = imuOffset->getOffsetTransform();
                X.updR() = R_FB;
                imuOffset->setOffsetTransform(X);
            }
            else {
                cout << "Creating offset frame for " << imuName << endl;
                OpenSim::Body* body = dynamic_cast<OpenSim::Body*>(bodies[imuix]);
                SimTK::Vec3 p_FB(0);
                if (body) {
                    p_FB = body->getMassCenter();
                }

                imuOffset = new PhysicalOffsetFrame(imuName,
                        *bodies[imuix], SimTK::Transform(R_FB, p_FB));
                auto* brick = new Brick(Vec3(0.02, 0.01, 0.005));
                brick->setColor(SimTK::Orange);
                imuOffset->attachGeometry(brick);
                bodies[imuix]->addComponent(imuOffset);
                cout << "Added offset frame for " << imuName << endl;
            }
            cout << imuOffset->getName() << " offset computed from " <<
                imuName << " data from file." << endl;
        }
        imuix++;
    }

    model.finalizeConnections();

    auto filename = "calibrated_" + model.getName() + ".osim";
    cout << "Wrote calibrated model to file: '" << filename << "'." << endl;
    model.print(filename);

    model.setUseVisualizer(true);
    SimTK::State& s = model.initSystem();

    s.updTime() = times[0];

    // create the solver given the input data
    MarkersReference mRefs{};
    OrientationsReference oRefs(orientationsData);
    SimTK::Array_<CoordinateReference> coordRefs{};

    const double accuracy = 1e-4;
    InverseKinematicsSolver ikSolver(model, mRefs, oRefs, coordRefs);
    ikSolver.setAccuracy(accuracy);

    model.getVisualizer().getSimbodyVisualizer().setShowSimTime(true);
    ikSolver.assemble(s);
    model.getVisualizer().show(s);

    char c;
    std::cout << "Press any key and return to close visualizer." << std::endl;
    std::cin >> c;
}
