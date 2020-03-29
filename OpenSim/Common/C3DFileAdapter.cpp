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

#ifdef WITH_BTK
// Function to convert Eigen matrix to SimTK matrix. This can become a lambda
// funciton inside extendRead in future.
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
    throw Exception{"Writing C3D not supported yet."};
}

C3DFileAdapter::OutputTables
C3DFileAdapter::extendRead(const std::string& fileName) const {
#ifdef WITH_EZC3D
    auto c3d = ezc3d::c3d(fileName);
#else
    auto reader = btk::AcquisitionFileReader::New();
    reader->SetFilename(fileName);
    reader->Update();
    auto acquisition = reader->GetOutput();
#endif

    EventTable event_table{};
#ifdef WITH_EZC3D
    std::vector<std::string> eventDescription;
    if (c3d.parameters().isGroup("EVENT")
            && c3d.parameters().group("EVENT").isParameter("DESCRIPTION")){
        eventDescription = c3d.parameters().group("EVENT")
                .parameter("DESCRIPTION").valuesAsString();
    }

    for (size_t i=0; i<c3d.header().eventsTime().size();++i) {
        std::string eventDescriptionStr("");
        if (eventDescription.size() >= i){
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
#else
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
#endif
    OutputTables tables{};

#ifdef WITH_EZC3D
    int numFrames(static_cast<int>(c3d.data().nbFrames()));
    int numMarkers(c3d.parameters().group("POINT")
                  .parameter("USED").valuesAsInt()[0]);
    double pointFrequency(
                static_cast<double>(
                    c3d.parameters().group("POINT")
                    .parameter("RATE").valuesAsDouble()[0]));
#else
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
#endif

    if(numMarkers != 0) {

        int marker_nrow = numFrames;
        int marker_ncol = numMarkers;

        std::vector<double> marker_times(marker_nrow);
        SimTK::Matrix_<SimTK::Vec3> marker_matrix(marker_nrow, marker_ncol);

        std::vector<std::string> marker_labels{};
#ifdef WITH_EZC3D
        for (auto label : c3d.parameters().group("POINT")
                          .parameter("LABELS").valuesAsString()) {
            marker_labels.push_back(SimTK::Value<std::string>(label));
        }
#else
        for (auto it = marker_pts->Begin(); it != marker_pts->End(); ++it) {
            marker_labels.push_back(SimTK::Value<std::string>((*it)->GetLabel()));
        }
#endif

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
#ifdef WITH_EZC3D
            for(auto pt : c3d.data().frame(f).points().points()) {
                if (!pt.isEmpty() ) {//residual is not -1
                    row[m] = SimTK::Vec3{ static_cast<double>(pt.x()),
                                          static_cast<double>(pt.y()),
                                          static_cast<double>(pt.z()) };
                }
                ++m;
            }
#else
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
#endif

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

#ifdef WITH_EZC3D
        const auto& units_param = c3d.parameters().group("POINT")
                .parameter("UNITS").valuesAsString();
        std::string units;
        if (units_param.size() > 0){
            units = units_param[0];
        }
        else {
            units = "";
        }
#else
        std::string units = acquisition->GetPointUnit();
#endif
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
#ifndef WITH_EZC3D
    // This is probably the right way to get the raw forces data from force 
    // platforms. Extract the collection of force platforms.
    auto force_platforms_extractor = btk::ForcePlatformsExtractor::New();
    force_platforms_extractor->SetInput(acquisition);
    auto force_platform_collection = force_platforms_extractor->GetOutput();
    force_platforms_extractor->Update();

    std::vector<SimTK::Matrix_<double>> fpCalMatrices{};
    std::vector<SimTK::Matrix_<double>> fpCorners{};
    std::vector<SimTK::Matrix_<double>> fpOrigins{};
    std::vector<unsigned>               fpTypes{};
    auto    fp_force_pts = btk::PointCollection::New();
    auto   fp_moment_pts = btk::PointCollection::New();
    auto fp_position_pts = btk::PointCollection::New();
    for(auto platform = force_platform_collection->Begin(); 
        platform != force_platform_collection->End(); 
        ++platform) {
        const auto& calMatrix = (*platform)->GetCalMatrix();
        const auto& corners   = (*platform)->GetCorners();
        const auto& origins   = (*platform)->GetOrigin();
        fpCalMatrices.push_back(convertToSimtkMatrix(calMatrix));
        fpCorners.push_back(convertToSimtkMatrix(corners));
        fpOrigins.push_back(convertToSimtkMatrix(origins));
        fpTypes.push_back(static_cast<unsigned>((*platform)->GetType()));

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

    if(fp_force_pts->GetItemNumber() != 0) {

        std::vector<std::string> labels{};
        ValueArray<std::string> units{};
        for(int fp = 1; fp <= fp_force_pts->GetItemNumber(); ++fp) {
            auto fp_str = std::to_string(fp);

            labels.push_back(SimTK::Value<std::string>("f" + fp_str));
            auto force_unit = acquisition->GetPointUnits().
                at(_unit_index.at("force"));
            units.upd().push_back(SimTK::Value<std::string>(force_unit));

            labels.push_back(SimTK::Value<std::string>("p" + fp_str));
            auto position_unit = acquisition->GetPointUnits().
                at(_unit_index.at("marker"));
            units.upd().push_back(SimTK::Value<std::string>(position_unit));

            labels.push_back(SimTK::Value<std::string>("m" + fp_str));
            auto moment_unit = acquisition->GetPointUnits().
                at(_unit_index.at("moment"));
            units.upd().push_back(SimTK::Value<std::string>(moment_unit));
        }

        const int nf = fp_force_pts->GetFrontItem()->GetFrameNumber();
        
        std::vector<double> force_times(nf);
        SimTK::Matrix_<SimTK::Vec3> force_matrix(nf, (int)labels.size());

        double time_step{1.0 / acquisition->GetAnalogFrequency()};

        for(int f = 0; f < nf;  ++f) {
            SimTK::RowVector_<SimTK::Vec3> 
                row{fp_force_pts->GetItemNumber() * 3};
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
                std::to_string(acquisition->GetAnalogFrequency()));

        tables.emplace(_forces,
            std::shared_ptr<TimeSeriesTableVec3>(&force_table));

        force_table.updTableMetaData().setValueForKey("events", event_table);
    }
    else { // insert empty table
        std::vector<double> emptyTimes;
        std::vector<std::string> emptyLabels;
        SimTK::Matrix_<SimTK::Vec3> noData;
        auto emptyforcesTable = std::make_shared<TimeSeriesTableVec3>(emptyTimes, noData, emptyLabels);
        tables.emplace(_forces, emptyforcesTable);
    }
#endif
    return tables;
}

void
C3DFileAdapter::extendWrite(const InputTables& absTables,
                            const std::string& fileName) const {
    throw Exception{"Writing to C3D not supported yet."};
}

} // namespace OpenSim
