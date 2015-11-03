#include "C3DFileAdapter.h"

namespace OpenSim {

    template<typename T>
    class shrik;

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
    auto tables = extendRead(fileName);
    auto    abs_marker_table = tables[0].release();
    auto     abs_force_table = tables[1].release();
    auto abs_usr_force_table = tables[2].release();
    auto    marker_table = static_cast<MarkerTable*>(   abs_marker_table);
    auto     force_table = static_cast< ForceTable*>(    abs_force_table);
    auto usr_force_table = static_cast< ForceTable*>(abs_usr_force_table);
    return std::make_tuple(std::unique_ptr<MarkerTable>{   marker_table}, 
                           std::unique_ptr< ForceTable>{    force_table},
                           std::unique_ptr< ForceTable>{usr_force_table});
}

void
C3DFileAdapter::write(const C3DFileAdapter::Tables& tables,
                      const std::string& fileName) const {
}

C3DFileAdapter::OutputTables
C3DFileAdapter::extendRead(const std::string& fileName) const {
    auto reader = btk::AcquisitionFileReader::New();
    reader->SetFilename(fileName);
    reader->Update();
    auto acquisition = reader->GetOutput();

    OutputTables tables{};
    auto&    marker_table = *(new MarkerTable{});
    auto&     force_table = *(new  ForceTable{});
    auto& usr_force_table = *(new  ForceTable{});
    tables.emplace_back(   &marker_table);
    tables.emplace_back(    &force_table);
    tables.emplace_back(&usr_force_table);

    auto    marker_pts = btk::PointCollection::New();
    auto usr_force_pts = btk::PointCollection::New();

    for(auto it = acquisition->BeginPoint();
        it != acquisition->EndPoint();
        ++it) {
        auto pt = *it;
        if(pt->GetType() == btk::Point::Marker)
               marker_pts->InsertItem(pt);
        else if(pt->GetType() == btk::Point::Force)
            usr_force_pts->InsertItem(pt);
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

        MarkerTable::DependentsMetaData marker_dep_metadata{};
        marker_dep_metadata.setValueArrayForKey("labels", marker_labels);
        marker_table.setDependentsMetaData(marker_dep_metadata);

        double time_step{1.0 / acquisition->GetPointFrequency()};
        for(size_t f = 0; 
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

    if(usr_force_pts->GetItemNumber() != 0) {
        usr_force_table.
            updTableMetaData().
            setValueForKey("DataRate", 
                           std::to_string(acquisition->GetAnalogFrequency()));

        usr_force_table.
            updTableMetaData().
            setValueForKey("Units", 
                           acquisition->GetPointUnits().
                           at(_unit_index.at("force")));

        ValueArray<std::string> force_labels{};
        for(auto it = usr_force_pts->Begin();
            it != usr_force_pts->End();
            ++it) {
            auto pt = *it;
            force_labels.
                upd().
                push_back(SimTK::Value<std::string>(pt->GetLabel()));
        }
        ForceTable::DependentsMetaData force_dep_metadata{};
        force_dep_metadata.setValueArrayForKey("labels", force_labels);
        usr_force_table.setDependentsMetaData(force_dep_metadata);

        double time_step{1.0 / acquisition->GetAnalogFrequency()};
        for(size_t f = 0;
            f < usr_force_pts->GetFrontItem()->GetFrameNumber();
            ++f) {
            SimTK::RowVector_<SimTK::Vec3> row{usr_force_pts->GetItemNumber()};
            int m{0};
            for(auto it = usr_force_pts->Begin();
                it != usr_force_pts->End();
                ++it) {
                auto pt = *it;
                row[m++] = SimTK::Vec3{pt->GetValues().coeff(f, 0),
                                       pt->GetValues().coeff(f, 1),
                                       pt->GetValues().coeff(f, 2)};
            }
            usr_force_table.appendRow(0 + f * time_step, row);
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
        for(size_t fp = 1; fp <= fp_force_pts->GetItemNumber(); ++fp) {
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
        ForceTable::DependentsMetaData force_dep_metadata{};
        force_dep_metadata.setValueArrayForKey("labels", labels);
        force_dep_metadata.setValueArrayForKey("units", units);
        force_table.setDependentsMetaData(force_dep_metadata);

        double time_step{1.0 / acquisition->GetAnalogFrequency()};
        for(size_t f = 0;
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
    usr_force_table.updTableMetaData().setValueForKey("events", event_table);

    return tables;
}

void
C3DFileAdapter::extendWrite(const InputTables& absTables,
                            const std::string& fileName) const {
}

} // namespace OpenSim
