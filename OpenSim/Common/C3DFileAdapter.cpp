#include "C3DFileAdapter.h"

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

C3DFileAdapter::Tables
C3DFileAdapter::read(const std::string& fileName) const {
    auto abstables = extendRead(fileName);
    Tables tables{};
    tables.emplace(_markers, 
                   static_cast<TimeSeriesTableVec3&>(*abstables.at(_markers)));
    tables.emplace( _forces, 
                    static_cast<TimeSeriesTableVec3&>(*abstables.at( _forces)));
    return tables;
}

void
C3DFileAdapter::write(const C3DFileAdapter::Tables& tables,
                      const std::string& fileName) const {
    throw Exception{"Writing C3D not supported yet."};
}

C3DFileAdapter::OutputTables
C3DFileAdapter::extendRead(const std::string& fileName) const {
    auto reader = btk::AcquisitionFileReader::New();
    reader->SetFilename(fileName);
    reader->Update();
    auto acquisition = reader->GetOutput();

    OutputTables tables{};
    auto& marker_table = *(new TimeSeriesTableVec3{});
    auto&  force_table = *(new TimeSeriesTableVec3{});
    tables.emplace(_markers, 
                   std::shared_ptr<TimeSeriesTableVec3>(&marker_table));
    tables.emplace(_forces, 
                   std::shared_ptr<TimeSeriesTableVec3>(&force_table));

    auto marker_pts = btk::PointCollection::New();

    for(auto it = acquisition->BeginPoint();
        it != acquisition->EndPoint();
        ++it) {
        auto pt = *it;
        if(pt->GetType() == btk::Point::Marker)
               marker_pts->InsertItem(pt);
    }

    if(marker_pts->GetItemNumber() != 0) {
        marker_table.
            updTableMetaData().
            setValueForKey("DataRate", 
                           std::to_string(acquisition->GetPointFrequency()));

        marker_table.
            updTableMetaData().
            setValueForKey("Units", 
                           acquisition->GetPointUnit());

        ValueArray<std::string> marker_labels{};
        for(auto it = marker_pts->Begin();
            it != marker_pts->End();
            ++it) {
            marker_labels.
            upd().
            push_back(SimTK::Value<std::string>((*it)->GetLabel()));
        }

        TimeSeriesTableVec3::DependentsMetaData marker_dep_metadata{};
        marker_dep_metadata.setValueArrayForKey("labels", marker_labels);
        marker_table.setDependentsMetaData(marker_dep_metadata);

        double time_step{1.0 / acquisition->GetPointFrequency()};
        for(int f = 0; 
            f < marker_pts->GetFrontItem()->GetFrameNumber();
            ++f) {
            SimTK::RowVector_<SimTK::Vec3> row{marker_pts->GetItemNumber()};
            int m{0};
            for(auto it = marker_pts->Begin();
                it != marker_pts->End();
                ++it) {
                auto pt = *it;
                row[m++] = SimTK::Vec3{pt->GetValues().coeff(f, 0),
                                       pt->GetValues().coeff(f, 1),
                                       pt->GetValues().coeff(f, 2)};
            }
            marker_table.appendRow(0 + f * time_step, row);
        }
    }

    // This is probably the right way to get the raw forces data from force 
    // platforms. Extract the collection of force platforms.
    auto force_platforms_extractor = btk::ForcePlatformsExtractor::New();
    force_platforms_extractor->SetInput(acquisition);
    auto force_platform_collection = force_platforms_extractor->GetOutput();
    force_platforms_extractor->Update();

    std::vector<btk::ForcePlatform::CalMatrix> fp_cal_matrices{};
    std::vector<btk::ForcePlatform::Corners>   fp_corners{};
    std::vector<btk::ForcePlatform::Origin>    fp_origins{};
    std::vector<unsigned>                      fp_types{};
    auto    fp_force_pts = btk::PointCollection::New();
    auto   fp_moment_pts = btk::PointCollection::New();
    auto fp_position_pts = btk::PointCollection::New();
    for(auto platform = force_platform_collection->Begin(); 
        platform != force_platform_collection->End(); 
        ++platform) {
        fp_cal_matrices.push_back((*platform)->GetCalMatrix());
        fp_corners.push_back((*platform)->GetCorners());
        fp_origins.push_back((*platform)->GetOrigin());
        fp_types.push_back(static_cast<unsigned>((*platform)->GetType()));

        // Get ground reaction wrenches for the force platform.
        auto ground_reaction_wrench_filter = 
            btk::GroundReactionWrenchFilter::New();
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
        force_table.
            updTableMetaData().
            setValueForKey("CalibrationMatrices", std::move(fp_cal_matrices));

        force_table.
            updTableMetaData().
            setValueForKey("Corners",             std::move(fp_corners));

        force_table.
            updTableMetaData().
            setValueForKey("Origins",             std::move(fp_origins));

        force_table.
            updTableMetaData().
            setValueForKey("Types",               std::move(fp_types));

        force_table.
            updTableMetaData().
            setValueForKey("DataRate", 
                           std::to_string(acquisition->GetAnalogFrequency()));

        ValueArray<std::string> labels{};
        ValueArray<std::string> units{};
        for(int fp = 1; fp <= fp_force_pts->GetItemNumber(); ++fp) {
            auto fp_str = std::to_string(fp);

            labels.upd().push_back(SimTK::Value<std::string>("f" + fp_str));
            auto force_unit = acquisition->GetPointUnits().
                at(_unit_index.at("force"));
            units.upd().push_back(SimTK::Value<std::string>(force_unit));

            labels.upd().push_back(SimTK::Value<std::string>("m" + fp_str));
            auto moment_unit = acquisition->GetPointUnits().
                at(_unit_index.at("moment"));
            units.upd().push_back(SimTK::Value<std::string>(moment_unit));

            labels.upd().push_back(SimTK::Value<std::string>("p" + fp_str));
            auto position_unit = acquisition->GetPointUnits().
                at(_unit_index.at("marker"));
            units.upd().push_back(SimTK::Value<std::string>(position_unit));
        }
        TimeSeriesTableVec3::DependentsMetaData force_dep_metadata{};
        force_dep_metadata.setValueArrayForKey("labels", labels);
        force_dep_metadata.setValueArrayForKey("units", units);
        force_table.setDependentsMetaData(force_dep_metadata);

        double time_step{1.0 / acquisition->GetAnalogFrequency()};
        for(int f = 0;
            f < fp_force_pts->GetFrontItem()->GetFrameNumber();
            ++f) {
            SimTK::RowVector_<SimTK::Vec3>
                row{fp_force_pts->GetItemNumber() * 3};
            size_t col{0};
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
                row[col] = SimTK::Vec3{(*mit)->GetValues().coeff(f, 0),
                                       (*mit)->GetValues().coeff(f, 1),
                                       (*mit)->GetValues().coeff(f, 2)};
                ++col;
                row[col] = SimTK::Vec3{(*pit)->GetValues().coeff(f, 0),
                                       (*pit)->GetValues().coeff(f, 1),
                                       (*pit)->GetValues().coeff(f, 2)};
                ++col;
            }
            force_table.appendRow(0 + f * time_step, row);
        }
    }

    EventTable event_table{};
    auto events = acquisition->GetEvents();
    for(auto it = events->Begin();
        it != events->End();
        ++it) {
        auto et = *it;
        event_table.push_back({et->GetLabel(),
                               et->GetTime(),
                               et->GetFrame(),
                               et->GetDescription()});
    }
       marker_table.updTableMetaData().setValueForKey("events", event_table);
        force_table.updTableMetaData().setValueForKey("events", event_table);

    return tables;
}

void
C3DFileAdapter::extendWrite(const InputTables& absTables,
                            const std::string& fileName) const {
    throw Exception{"Writing to C3D not supported yet."};
}

} // namespace OpenSim
