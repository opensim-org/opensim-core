
#include "InverseMuscleSolver.h"

using namespace OpenSim;

InverseMuscleSolver::InverseMuscleSolver() {
    constructProperties();
}

InverseMuscleSolver::InverseMuscleSolver(const std::string& setupFilePath) :
        Object(setupFilePath) {
    constructProperties();
}

void InverseMuscleSolver::constructProperties() {
    constructProperty_model_file("");
    constructProperty_kinematics_file("");
    constructProperty_net_generalized_forces_file("");
    constructProperty_lowpass_cutoff_frequency_for_joint_moments(-1);
    constructProperty_initial_time();
    constructProperty_final_time();
    constructProperty_create_reserve_actuators(-1);
    constructProperty_coordinates_to_include();
    constructProperty_actuators_to_include();
}

void InverseMuscleSolver::loadModelAndData(Model& model,
        TimeSeriesTable& kinematics,
        TimeSeriesTable& netGeneralizedForces) const {
    using SimTK::Pathname;
    // Get the directory containing the setup file.
    std::string setupDir;
    {
        bool dontApplySearchPath;
        std::string fileName, extension;
        Pathname::deconstructPathname(getDocumentFileName(),
                dontApplySearchPath, setupDir, fileName, extension);
    }

    // Model.
    // ------
    OPENSIM_THROW_IF_FRMOBJ(!get_model_file().empty() && _model, Exception,
            "A model has been specified via the model_file "
            "property AND via setModel(); only one of these two mechanisms "
            "can be used. Consider setting the model_file "
            "property to an empty string.");
    if (!get_model_file().empty()) {
        // The model file path may be relative to the setup dir.
        std::string modelFilePath =
                Pathname::getAbsolutePathnameUsingSpecifiedWorkingDirectory(
                        setupDir, get_model_file());
        model = Model(modelFilePath);
    } else if (_model) {
        // The user called setModel().
        model = Model(*_model.get());
    } else {
        OPENSIM_THROW_FRMOBJ(Exception, "No model specified.");
    }
    model.initSystem();

    // Kinematics.
    // -----------
    OPENSIM_THROW_IF_FRMOBJ(!get_kinematics_file().empty() && _kinematics,
            Exception,
            "Kinematics have been specified via the kinematics_file "
            "property AND via setKinematicsData(); only one of these two "
            "mechanisms can be used. Consider setting the kinematics_file "
            "property to an empty string.");
    if (!get_kinematics_file().empty()) {
        // The kinematics file path may be relative to the setup dir.
        std::string kinematicsFilePath =
                Pathname::getAbsolutePathnameUsingSpecifiedWorkingDirectory(
                        setupDir, get_kinematics_file());
        DataAdapter::OutputTables tables =
                FileAdapter::readFile(kinematicsFilePath);

        // There should only be one table.
        OPENSIM_THROW_IF_FRMOBJ(tables.size() != 1, Exception,
                "Expected kinematics file '" + get_kinematics_file() +
                "' to contain 1 table, but it contains " +
                std::to_string(tables.size()) + " tables.");

        // Get the first table.
        auto* firstTable =
                dynamic_cast<TimeSeriesTable*>(tables.begin()->second.get());
        OPENSIM_THROW_IF_FRMOBJ(!firstTable, Exception,
                "Expected kinematics file to contain a (scalar) "
                "TimeSeriesTable, but it contains a different type of table.");

        kinematics = *firstTable;
    } else if (_kinematics) {
        // The user called setKinematicsData().
        kinematics = TimeSeriesTable(*_kinematics.get());
    } else {
        OPENSIM_THROW_FRMOBJ(Exception, "No kinematics data specified.");
    }
    OPENSIM_THROW_IF_FRMOBJ(kinematics.getNumRows() == 0, Exception,
            "The provided kinematics table has no rows.");

    // Convert rotational DOFs to radians.
    if (kinematics.hasTableMetaDataKey("inDegrees") &&
            kinematics.getTableMetaDataAsString("inDegrees") == "yes") {
        model.getSimbodyEngine().convertDegreesToRadians(kinematics);
    }

    // Net generalized forces (optional).
    // ----------------------------------
    OPENSIM_THROW_IF_FRMOBJ(!get_net_generalized_forces_file().empty()
            && _netGeneralizedForces, Exception,
            "Net generalized forces have been specified via the "
            "net_generalized_forces_file property AND via "
            "setNetGeneralizedForcesData(); only one of these two mechanisms "
            "can be used. Consider setting the net_generalized_forces_file "
            "property to an empty string.");
    if (!get_net_generalized_forces_file().empty()) {
        std::string netGenForcesFilePath =
                Pathname::getAbsolutePathnameUsingSpecifiedWorkingDirectory(
                        setupDir, get_net_generalized_forces_file());
        DataAdapter::OutputTables tables =
                FileAdapter::readFile(netGenForcesFilePath);

        // There should only be one table.
        OPENSIM_THROW_IF_FRMOBJ(tables.size() != 1, Exception,
                "Expected net generalized forces file '" +
                get_net_generalized_forces_file() + "' to contain 1 table, but "
                "it contains " + std::to_string(tables.size()) + " tables.");

        // Get the first table.
        auto* firstTable =
                dynamic_cast<TimeSeriesTable*>(tables.begin()->second.get());
        OPENSIM_THROW_IF_FRMOBJ(!firstTable, Exception,
                "Expected net generalized forces file to contain a (scalar) "
                "TimeSeriesTable, but it contains a different type of table.");

        netGeneralizedForces = *firstTable;
    } else if (_netGeneralizedForces) {
        netGeneralizedForces = TimeSeriesTable(*_netGeneralizedForces.get());
    } else {
        // This data table is optional, so this is not an error.
        // Set to an empty table to communicate that this was not provided.
        netGeneralizedForces = TimeSeriesTable();
    }
}

void InverseMuscleSolver::processActuatorsToInclude(Model& model) const {
    if (!getProperty_actuators_to_include().empty()) {
        // Keep track of which requested actuators we actually find.
        std::set<std::string> actuToInclude;
        auto numActuToInclude = getProperty_actuators_to_include().size();
        for (int iInclude = 0; iInclude < numActuToInclude; ++iInclude) {
            actuToInclude.insert(get_actuators_to_include(iInclude));
        }
        // Check each actuator to see if it should be included.
        const ComponentPath modelPath = model.getAbsolutePathName();
        auto actuList = model.updComponentList<Actuator>();
        for (auto& actu : actuList) {
            const auto actuPath = ComponentPath(actu.getAbsolutePathName())
                    .formRelativePath(modelPath).toString();
            const auto foundActuPath = actuToInclude.find(actuPath);
            if (foundActuPath == actuToInclude.end()) {
                // Could not find in the set; do not include.
                actu.set_appliesForce(false);
            } else {
                // Found this actuator in the set; make sure it is enabled.
                actu.set_appliesForce(true);
                actuToInclude.erase(foundActuPath);
            }
        }
        // Any remaining paths are not in the model.
        if (!actuToInclude.empty()) {
            std::string msg = "Could not find the following actuators "
                    "listed under 'actuators_to_include' (make sure to "
                    "use the *path* to the actuator):\n";
            for (const auto& actuPath : actuToInclude) {
                msg += "  " + actuPath + "\n";
            }
            OPENSIM_THROW_FRMOBJ(Exception, msg);
        }
        std::cout << "The following Actuators will apply force (excluding "
                "those from 'create_reserve_actuators'):"
                << std::endl;
        for (const auto& actu : actuList) {
            if (actu.get_appliesForce()) {
                std::cout << "  " << actu.getAbsolutePathName() << std::endl;
            }
        }
    }
}

void InverseMuscleSolver::determineInitialAndFinalTimes(
        TimeSeriesTable& kinematics, TimeSeriesTable& netGeneralizedForces,
        double& initialTime, double& finalTime) const {

    double initialTimeFromData = kinematics.getIndependentColumn().front();
    double finalTimeFromData = kinematics.getIndependentColumn().back();
    if (netGeneralizedForces.getNumRows()) {
        initialTimeFromData = std::max(initialTimeFromData,
                netGeneralizedForces.getIndependentColumn().front());
        finalTimeFromData = std::min(finalTimeFromData,
                netGeneralizedForces.getIndependentColumn().back());
    }
    if (!getProperty_initial_time().empty()) {
        OPENSIM_THROW_IF_FRMOBJ(get_initial_time() < initialTimeFromData,
                Exception, "Provided initial time of " +
                std::to_string(get_initial_time()) + " is less than what "
                "is available from data, " +
                std::to_string(initialTimeFromData) + ".");
        initialTime = get_initial_time();
    } else {
        initialTime = initialTimeFromData;
    }
    if (!getProperty_final_time().empty()) {
        OPENSIM_THROW_IF_FRMOBJ(get_final_time() > finalTimeFromData,
                Exception, "Provided final time of " +
                std::to_string(get_final_time()) + " is greater than what "
                "is available from data, " +
                std::to_string(finalTimeFromData) + ".");
        finalTime = get_final_time();
    } else {
        finalTime = finalTimeFromData;
    }
    OPENSIM_THROW_IF_FRMOBJ(finalTime < initialTime, Exception,
            "Initial time of " + std::to_string(initialTime) + " is greater "
            "than final time of " + std::to_string(finalTime) + ".");
}
