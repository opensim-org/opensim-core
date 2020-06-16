#include "C3DFileAdapter.h"

#ifdef WITH_EZC3D
#include "ezc3d_all.h"
#else
#include "btkAcquisitionFileReader.h"
#include "btkAcquisition.h"
#include "btkForcePlatformsExtractor.h"
#include "btkGroundReactionWrenchFilter.h"
#endif

namespace {

#ifdef WITH_EZC3D
// Function to convert ezc3d matrix to SimTK matrix. This can become a lambda
// function inside extendRead in future.
SimTK::Matrix_<double>
convertToSimtkMatrix(const ezc3d::Matrix& mat) {
    SimTK::Matrix_<double> simtkMat{static_cast<int>(mat.nbRows()),
                                    static_cast<int>(mat.nbCols())};

    for(int r = 0; r < (int)mat.nbRows(); ++r)
        for(int c = 0; c < (int)mat.nbCols(); ++c)
            simtkMat(r, c) = mat(r, c);

    return simtkMat;
}

// Function to convert a collection of ezc3d Vector3d to SimTK matrix.
// This can become a lambda function inside extendRead in future.
SimTK::Matrix_<double>
convertToSimtkMatrix(const std::vector<ezc3d::Vector3d>& all_vec) {
    SimTK::Matrix_<double> simtkMat{3, static_cast<int>(all_vec.size())};

    for(int r = 0; r < (int)all_vec.size(); ++r){
        const ezc3d::Vector3d& vec(all_vec[r]);
        for(int c = 0; c < 3; ++c){
            simtkMat(c, r) = vec(c);
        }
    }

    return simtkMat;
}

#else // WITH_BTK.
// Function to convert Eigen matrix to SimTK matrix. This can become a lambda
// function inside extendRead in future.
template<typename _Scalar, int _Rows, int _Cols>
SimTK::Matrix_<double>
convertToSimtkMatrix(const Eigen::Matrix<_Scalar, _Rows, _Cols>&
                     eigenMat) {
    SimTK::Matrix_<double> simtkMat{static_cast<int>(eigenMat.rows()),
                                    static_cast<int>(eigenMat.cols())};
            
    for(int r = 0; r < eigenMat.rows(); ++r)
        for(int c = 0; c < eigenMat.cols(); ++c)
            simtkMat(r, c) = eigenMat(r, c);

    return simtkMat;
}
#endif
} // anonymous namespace


namespace OpenSim {

const std::string C3DFileAdapter::_markers{"markers"};
const std::string C3DFileAdapter::_forces{"forces"};

const std::unordered_map<std::string, size_t>
C3DFileAdapter::_unit_index{{"marker", 0},
                            {"angle" , 1},
                            {"force" , 2},
                            {"moment", 3},
                            {"power" , 4},
                            {"scalar", 5}};

C3DFileAdapter*
C3DFileAdapter::clone() const {
    return new C3DFileAdapter{*this};
}

void
C3DFileAdapter::write(const C3DFileAdapter::Tables& tables,
                      const std::string& fileName) {
    OPENSIM_THROW(Exception, "Writing C3D not supported yet.");
}

C3DFileAdapter::OutputTables
C3DFileAdapter::extendRead(const std::string& fileName) const {
#ifdef WITH_EZC3D
    auto c3d = ezc3d::c3d(fileName);

    EventTable event_table{};
    std::vector<std::string> eventDescription;
    if (c3d.parameters().isGroup("EVENT")
        && c3d.parameters().group("EVENT").isParameter("DESCRIPTION")){
        eventDescription = c3d.parameters().group("EVENT")
                .parameter("DESCRIPTION").valuesAsString();
    }

    for (size_t i=0; i<c3d.header().eventsTime().size();++i) {
        std::string eventDescriptionStr("");
        if (eventDescription.size() > i){
            eventDescriptionStr = eventDescription[i];
        }
        event_table.push_back(
                {
                        c3d.header().eventsLabel(i),
                        static_cast<double>(c3d.header().eventsTime(i)),
                        static_cast<int>(
                                c3d.header().eventsTime(i) / c3d.header().frameRate()),
                        eventDescriptionStr
                });
    }
    OutputTables tables{};

    int numFrames(static_cast<int>(c3d.data().nbFrames()));
    int numMarkers(c3d.parameters().group("POINT")
                           .parameter("USED").valuesAsInt()[0]);
    double pointFrequency(
            static_cast<double>(
                    c3d.parameters().group("POINT")
                            .parameter("RATE").valuesAsDouble()[0]));

    if(numMarkers != 0) {

        int marker_nrow = numFrames;
        int marker_ncol = numMarkers;

        std::vector<double> marker_times(marker_nrow);
        SimTK::Matrix_<SimTK::Vec3> marker_matrix(marker_nrow, marker_ncol);

        std::vector<std::string> marker_labels{};
        for (auto label : c3d.parameters().group("POINT")
                .parameter("LABELS").valuesAsString()) {
            marker_labels.push_back(SimTK::Value<std::string>(label));
        }

        double time_step{1.0 / pointFrequency};
        for(int f = 0; f < marker_nrow; ++f) {
            SimTK::RowVector_<SimTK::Vec3> row{ numMarkers,
                                                SimTK::Vec3(SimTK::NaN) };
            int m{0};
            // C3D standard is to read empty values as zero, but sets a
            // "residual" value to -1 and it is how it knows to export these
            // values as blank, instead of 0,  when exporting to .trc
            // See: C3D documention 3D Point Residuals
            // Read in value if it is not zero or residual is not -1
            for(auto pt : c3d.data().frame(f).points().points()) {
                if (!pt.isEmpty() ) {//residual is not -1
                    row[m] = SimTK::Vec3{ static_cast<double>(pt.x()),
                                          static_cast<double>(pt.y()),
                                          static_cast<double>(pt.z()) };
                }
                ++m;
            }

            marker_matrix.updRow(f) = row;
            marker_times[f] = 0 + f * time_step; //TODO: 0 should be start_time
        }

        // Create the data
        auto marker_table =
                std::make_shared<TimeSeriesTableVec3>(marker_times,
                                                      marker_matrix,
                                                      marker_labels);

        marker_table->
                updTableMetaData().
                setValueForKey("DataRate",
                               std::to_string(pointFrequency));

        const auto& units_param = c3d.parameters().group("POINT")
                .parameter("UNITS").valuesAsString();
        std::string units;
        if (units_param.size() > 0){
            units = units_param[0];
        }
        else {
            units = "";
        }
        marker_table->updTableMetaData().setValueForKey("Units", units);

        marker_table->updTableMetaData().setValueForKey("events", event_table);

        tables.emplace(_markers, marker_table);
    }
    else { // insert empty table
        std::vector<double> emptyTimes;
        std::vector<std::string> emptyLabels;
        SimTK::Matrix_<SimTK::Vec3> noData;
        auto emptyMarkersTable =
                std::make_shared<TimeSeriesTableVec3>(
                        emptyTimes, noData, emptyLabels);
        tables.emplace(_markers, emptyMarkersTable);
    }

    std::vector<SimTK::Matrix_<double>> fpCalMatrices{};
    std::vector<SimTK::Matrix_<double>> fpCorners{};
    std::vector<SimTK::Matrix_<double>> fpOrigins{};
    std::vector<unsigned>               fpTypes{};
    const auto& force_platforms_extractor = ezc3d::Modules::ForcePlatforms(c3d);

    ForceLocation forceLocation(getLocationForForceExpression());
    auto numPlatform(static_cast<int>(
                             force_platforms_extractor.forcePlatforms().size()));

    for (const auto& platform : force_platforms_extractor.forcePlatforms()){

        const auto& calMatrix = platform.calMatrix();
        const auto& corners   = platform.corners();
        const auto& origins   = platform.origin();
        auto type = platform.type();

        fpCalMatrices.push_back(convertToSimtkMatrix(calMatrix));
        fpCorners.push_back(convertToSimtkMatrix(corners));
        fpOrigins.push_back(convertToSimtkMatrix(origins));
        fpTypes.push_back(static_cast<unsigned>(type));

    }

    if(numPlatform != 0) {
        std::vector<std::string> labels{};
        ValueArray<std::string> units{};
        for(int fp = 1; fp <= numPlatform; ++fp) {
            auto fp_str = std::to_string(fp);

            auto force_unit =
                    force_platforms_extractor.forcePlatform(fp-1).forceUnit();
            auto position_unit =
                    force_platforms_extractor.forcePlatform(fp-1).positionUnit();
            auto moment_unit =
                    force_platforms_extractor.forcePlatform(fp-1).momentUnit();

            labels.push_back(SimTK::Value<std::string>("f" + fp_str));
            units.upd().push_back(SimTK::Value<std::string>(force_unit));

            labels.push_back(SimTK::Value<std::string>("p" + fp_str));
            units.upd().push_back(SimTK::Value<std::string>(position_unit));

            labels.push_back(SimTK::Value<std::string>("m" + fp_str));
            units.upd().push_back(SimTK::Value<std::string>(moment_unit));
        }

        const int nf = static_cast<const int>(force_platforms_extractor.forcePlatform(0).nbFrames());
        auto analogFrequency = static_cast<double>(c3d.header().frameRate()
                                                   * c3d.header().nbAnalogByFrame());
        const auto& pf_ref(force_platforms_extractor.forcePlatforms());

        std::vector<double> force_times(nf);
        SimTK::Matrix_<SimTK::Vec3> force_matrix(nf, (int)labels.size());

        double time_step{1.0 / analogFrequency};

        for(int f = 0; f < nf;  ++f) {
            SimTK::RowVector_<SimTK::Vec3>
                    row{numPlatform * 3};
            int col{0};
            for (size_t i = 0; i < (int)numPlatform; ++i){
                row[col] = SimTK::Vec3{pf_ref[i].forces()[f](0),
                                       pf_ref[i].forces()[f](1),
                                       pf_ref[i].forces()[f](2)};
                ++col;
                if (forceLocation == ForceLocation::CenterOfPressure){
                    row[col] = SimTK::Vec3{pf_ref[i].CoP()[f](0),
                                           pf_ref[i].CoP()[f](1),
                                           pf_ref[i].CoP()[f](2)};
                    ++col;
                    row[col] = SimTK::Vec3{pf_ref[i].Tz()[f](0),
                                           pf_ref[i].Tz()[f](1),
                                           pf_ref[i].Tz()[f](2)};
                    ++col;
                } else if (forceLocation == ForceLocation::OriginOfForcePlate){
                    row[col] = SimTK::Vec3{pf_ref[i].meanCorners()(0),
                                           pf_ref[i].meanCorners()(1),
                                           pf_ref[i].meanCorners()(2)};
                    ++col;
                    row[col] = SimTK::Vec3{pf_ref[i].moments()[f](0),
                                           pf_ref[i].moments()[f](1),
                                           pf_ref[i].moments()[f](2)};
                    ++col;
                } else {
                    OPENSIM_THROW(Exception,
                                  "The selected force location is not "
                                  "implemented for ezc3d files");
                }
            }
            force_matrix.updRow(f) = row;
            force_times[f] = 0 + f * time_step; //TODO: 0 should be start_time
        }

        auto&  force_table =
                *(new TimeSeriesTableVec3(force_times, force_matrix, labels));

        TimeSeriesTableVec3::DependentsMetaData force_dep_metadata
                = force_table.getDependentsMetaData();

        // add units to the dependent meta data
        force_dep_metadata.setValueArrayForKey("units", units);
        force_table.setDependentsMetaData(force_dep_metadata);

        force_table.
                updTableMetaData().
                setValueForKey("CalibrationMatrices", std::move(fpCalMatrices));

        force_table.
                updTableMetaData().
                setValueForKey("Corners", std::move(fpCorners));

        force_table.
                updTableMetaData().
                setValueForKey("Origins", std::move(fpOrigins));

        force_table.
                updTableMetaData().
                setValueForKey("Types", std::move(fpTypes));

        force_table.
                updTableMetaData().
                setValueForKey("DataRate",
                               std::to_string(analogFrequency));

        tables.emplace(_forces,
                       std::shared_ptr<TimeSeriesTableVec3>(&force_table));

        force_table.updTableMetaData().setValueForKey("events", event_table);
    }
    else { // insert empty table
        std::vector<double> emptyTimes;
        std::vector<std::string> emptyLabels;
        SimTK::Matrix_<SimTK::Vec3> noData;
        auto emptyforcesTable = std::make_shared<TimeSeriesTableVec3>(
                emptyTimes, noData, emptyLabels);
        tables.emplace(_forces, emptyforcesTable);
    }
    return tables;

#else // WITH_BTK.
    auto reader = btk::AcquisitionFileReader::New();
    reader->SetFilename(fileName);
    reader->Update();
    auto acquisition = reader->GetOutput();

    EventTable event_table{};
    auto events = acquisition->GetEvents();
    for (auto it = events->Begin();
        it != events->End();
        ++it) {
        auto et = *it;
        event_table.push_back({ et->GetLabel(),
            et->GetTime(),
            et->GetFrame(),
            et->GetDescription() });
    }
    OutputTables tables{};

    auto marker_pts = btk::PointCollection::New();

    for(auto it = acquisition->BeginPoint();
        it != acquisition->EndPoint();
        ++it) {
        auto pt = *it;
        if(pt->GetType() == btk::Point::Marker)
               marker_pts->InsertItem(pt);
    }
    int numFrames(marker_pts->GetFrontItem()->GetFrameNumber());
    int numMarkers(marker_pts->GetItemNumber());
    double pointFrequency(acquisition->GetPointFrequency());

    if(numMarkers != 0) {

        int marker_nrow = numFrames;
        int marker_ncol = numMarkers;

        std::vector<double> marker_times(marker_nrow);
        SimTK::Matrix_<SimTK::Vec3> marker_matrix(marker_nrow, marker_ncol);

        std::vector<std::string> marker_labels{};
        for (auto it = marker_pts->Begin(); it != marker_pts->End(); ++it) {
            marker_labels.push_back(SimTK::Value<std::string>((*it)->GetLabel()));
        }

        double time_step{1.0 / pointFrequency};
        for(int f = 0; f < marker_nrow; ++f) {
            SimTK::RowVector_<SimTK::Vec3> row{ numMarkers,
                                                SimTK::Vec3(SimTK::NaN) };
            int m{0};
            // C3D standard is to read empty values as zero, but sets a
            // "residual" value to -1 and it is how it knows to export these
            // values as blank, instead of 0,  when exporting to .trc
            // See: C3D documention 3D Point Residuals
            // Read in value if it is not zero or residual is not -1
            for(auto it = marker_pts->Begin();  it != marker_pts->End(); ++it) {
                // See: BTKCore/Code/IO/btkTRCFileIO.cpp#L359-L360
                auto pt = *it;
                if (!pt->GetValues().row(f).isZero() ||    //not precisely zero
                    (pt->GetResiduals().coeff(f) != -1) ) {//residual is not -1
                    row[m] = SimTK::Vec3{ pt->GetValues().coeff(f, 0),
                                          pt->GetValues().coeff(f, 1),
                                          pt->GetValues().coeff(f, 2) };
                }
                ++m;
            }

            marker_matrix.updRow(f) = row;
            marker_times[f] = 0 + f * time_step; //TODO: 0 should be start_time
        }

        // Create the data
        auto marker_table = 
            std::make_shared<TimeSeriesTableVec3>(marker_times, 
                                                  marker_matrix, 
                                                  marker_labels);

        marker_table->
            updTableMetaData().
            setValueForKey("DataRate",
                std::to_string(pointFrequency));

        std::string units = acquisition->GetPointUnit();
        marker_table->updTableMetaData().setValueForKey("Units", units);

        marker_table->updTableMetaData().setValueForKey("events", event_table);

        tables.emplace(_markers, marker_table);
    }
    else { // insert empty table
        std::vector<double> emptyTimes;
        std::vector<std::string> emptyLabels;
        SimTK::Matrix_<SimTK::Vec3> noData;
        auto emptyMarkersTable =
                std::make_shared<TimeSeriesTableVec3>(
                    emptyTimes, noData, emptyLabels);
        tables.emplace(_markers, emptyMarkersTable);
    }

    std::vector<SimTK::Matrix_<double>> fpCalMatrices{};
    std::vector<SimTK::Matrix_<double>> fpCorners{};
    std::vector<SimTK::Matrix_<double>> fpOrigins{};
    std::vector<unsigned>               fpTypes{};
    // This is probably the right way to get the raw forces data from force
    // platforms. Extract the collection of force platforms.
    auto force_platforms_extractor = btk::ForcePlatformsExtractor::New();
    force_platforms_extractor->SetInput(acquisition);
    auto force_platform_collection = force_platforms_extractor->GetOutput();
    force_platforms_extractor->Update();

    auto    fp_force_pts = btk::PointCollection::New();
    auto   fp_moment_pts = btk::PointCollection::New();
    auto fp_position_pts = btk::PointCollection::New();

    for(auto platform = force_platform_collection->Begin();
        platform != force_platform_collection->End();
        ++platform) {
        const auto& calMatrix = (*platform)->GetCalMatrix();
        const auto& corners   = (*platform)->GetCorners();
        const auto& origins   = (*platform)->GetOrigin();
        auto type = (*platform)->GetType();

        fpCalMatrices.push_back(convertToSimtkMatrix(calMatrix));
        fpCorners.push_back(convertToSimtkMatrix(corners));
        fpOrigins.push_back(convertToSimtkMatrix(origins));
        fpTypes.push_back(static_cast<unsigned>(type));

        // Get ground reaction wrenches for the force platform.
        auto ground_reaction_wrench_filter = 
            btk::GroundReactionWrenchFilter::New();
        ground_reaction_wrench_filter->setLocation(
            btk::GroundReactionWrenchFilter::Location(getLocationForForceExpression()));
        ground_reaction_wrench_filter->SetInput(*platform);
        auto wrench_collection = ground_reaction_wrench_filter->GetOutput();
        ground_reaction_wrench_filter->Update();
        
        for(auto wrench = wrench_collection->Begin();
            wrench != wrench_collection->End(); 
            ++wrench) {
            // Forces time series.
            fp_force_pts->InsertItem((*wrench)->GetForce());
            // Moment time series.
            fp_moment_pts->InsertItem((*wrench)->GetMoment());
            // Position time series.
            fp_position_pts->InsertItem((*wrench)->GetPosition());
        }
    }
    auto numPlatform(fp_force_pts->GetItemNumber());

    if(numPlatform != 0) {
        std::vector<std::string> labels{};
        ValueArray<std::string> units{};
        for(int fp = 1; fp <= numPlatform; ++fp) {
            auto fp_str = std::to_string(fp);

            auto force_unit = acquisition->GetPointUnits().
                    at(_unit_index.at("force"));
            auto position_unit = acquisition->GetPointUnits().
                at(_unit_index.at("marker"));
            auto moment_unit = acquisition->GetPointUnits().
                at(_unit_index.at("moment"));

            labels.push_back(SimTK::Value<std::string>("f" + fp_str));
            units.upd().push_back(SimTK::Value<std::string>(force_unit));

            labels.push_back(SimTK::Value<std::string>("p" + fp_str));
            units.upd().push_back(SimTK::Value<std::string>(position_unit));

            labels.push_back(SimTK::Value<std::string>("m" + fp_str));
            units.upd().push_back(SimTK::Value<std::string>(moment_unit));
        }

        const int nf = fp_force_pts->GetFrontItem()->GetFrameNumber();
        auto analogFrequency = acquisition->GetAnalogFrequency();

        std::vector<double> force_times(nf);
        SimTK::Matrix_<SimTK::Vec3> force_matrix(nf, (int)labels.size());

        double time_step{1.0 / analogFrequency};

        for(int f = 0; f < nf;  ++f) {
            SimTK::RowVector_<SimTK::Vec3> 
                row{numPlatform * 3};
            int col{0};
            for(auto fit = fp_force_pts->Begin(),
                mit =     fp_moment_pts->Begin(),
                pit =   fp_position_pts->Begin();
                fit != fp_force_pts->End();
                ++fit, 
                ++mit,
                ++pit) {
                row[col] = SimTK::Vec3{(*fit)->GetValues().coeff(f, 0),
                                       (*fit)->GetValues().coeff(f, 1),
                                       (*fit)->GetValues().coeff(f, 2)};
                ++col;
                row[col] = SimTK::Vec3{(*pit)->GetValues().coeff(f, 0),
                                       (*pit)->GetValues().coeff(f, 1),
                                       (*pit)->GetValues().coeff(f, 2)};
                ++col;
                row[col] = SimTK::Vec3{(*mit)->GetValues().coeff(f, 0),
                                       (*mit)->GetValues().coeff(f, 1),
                                       (*mit)->GetValues().coeff(f, 2)};
                ++col;
            }
            force_matrix.updRow(f) = row;
            force_times[f] = 0 + f * time_step; //TODO: 0 should be start_time
        }

        auto&  force_table = 
            *(new TimeSeriesTableVec3(force_times, force_matrix, labels));

        TimeSeriesTableVec3::DependentsMetaData force_dep_metadata
            = force_table.getDependentsMetaData();

        // add units to the dependent meta data
        force_dep_metadata.setValueArrayForKey("units", units);
        force_table.setDependentsMetaData(force_dep_metadata);

        force_table.
            updTableMetaData().
            setValueForKey("CalibrationMatrices", std::move(fpCalMatrices));

        force_table.
            updTableMetaData().
            setValueForKey("Corners", std::move(fpCorners));

        force_table.
            updTableMetaData().
            setValueForKey("Origins", std::move(fpOrigins));

        force_table.
            updTableMetaData().
            setValueForKey("Types", std::move(fpTypes));

        force_table.
            updTableMetaData().
            setValueForKey("DataRate",
                std::to_string(analogFrequency));

        tables.emplace(_forces,
            std::shared_ptr<TimeSeriesTableVec3>(&force_table));

        force_table.updTableMetaData().setValueForKey("events", event_table);
    }
    else { // insert empty table
        std::vector<double> emptyTimes;
        std::vector<std::string> emptyLabels;
        SimTK::Matrix_<SimTK::Vec3> noData;
        auto emptyforcesTable = std::make_shared<TimeSeriesTableVec3>(
                emptyTimes, noData, emptyLabels);
        tables.emplace(_forces, emptyforcesTable);
    }
    return tables;
#endif
}

void
C3DFileAdapter::extendWrite(const InputTables& absTables,
                            const std::string& fileName) const {
    OPENSIM_THROW(Exception, "Writing to C3D not supported yet.");
}

} // namespace OpenSim
