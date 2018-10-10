#include "C3DFileAdapter.h"

#include "btkAcquisitionFileReader.h"
#include "btkAcquisition.h"
#include "btkForcePlatformsExtractor.h"
#include "btkGroundReactionWrenchFilter.h"

namespace {

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

C3DFileAdapter::Tables
C3DFileAdapter::read(const std::string& fileName, ForceLocation wrt)
{
    C3DFileAdapter c3dreader{};
    c3dreader.setLocationForForceExpression(wrt);

    auto abstables = c3dreader.extendRead(fileName);
    auto marker_table = 
        std::static_pointer_cast<TimeSeriesTableVec3>(abstables.at(_markers));
    auto force_table = 
        std::static_pointer_cast<TimeSeriesTableVec3>(abstables.at(_forces));
    Tables tables{};
    tables.emplace(_markers, marker_table);
    tables.emplace( _forces,  force_table);
    return tables;
}

void
C3DFileAdapter::write(const C3DFileAdapter::Tables& tables,
                      const std::string& fileName) {
    throw Exception{"Writing C3D not supported yet."};
}

C3DFileAdapter::OutputTables
C3DFileAdapter::extendRead(const std::string& fileName) const {
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

    if(marker_pts->GetItemNumber() != 0) {

        int marker_nrow = marker_pts->GetFrontItem()->GetFrameNumber();
        int marker_ncol = marker_pts->GetItemNumber();

        std::vector<double> marker_times(marker_nrow);
        SimTK::Matrix_<SimTK::Vec3> marker_matrix(marker_nrow, marker_ncol);

        std::vector<std::string> marker_labels{};
        for (auto it = marker_pts->Begin(); it != marker_pts->End(); ++it) {
            marker_labels.push_back(SimTK::Value<std::string>((*it)->GetLabel()));
        }

        double time_step{1.0 / acquisition->GetPointFrequency()};
        for(int f = 0; f < marker_nrow; ++f) {
            SimTK::RowVector_<SimTK::Vec3> row{ marker_pts->GetItemNumber(), 
                                                SimTK::Vec3(SimTK::NaN) };
            int m{0};
            for(auto it = marker_pts->Begin();  it != marker_pts->End(); ++it) {
                auto pt = *it;
                // BTK reads empty values as zero, but sets a "residual" value
                // to -1 and it is how it knows to export these values as 
                // blank, instead of 0,  when exporting to .trc
                // See: BTKCore/Code/IO/btkTRCFileIO.cpp#L359-L360
                // Read in value if it is not zero or residual is not -1
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
        auto& marker_table = *new 
            TimeSeriesTableVec3(marker_times, marker_matrix, marker_labels);

        marker_table.
            updTableMetaData().
            setValueForKey("DataRate",
                std::to_string(acquisition->GetPointFrequency()));

        marker_table.
            updTableMetaData().
            setValueForKey("Units",
                acquisition->GetPointUnit());

        marker_table.updTableMetaData().setValueForKey("events", event_table);

        tables.emplace(_markers,
            std::shared_ptr<TimeSeriesTableVec3>(&marker_table));
    }

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

    return tables;
}

void
C3DFileAdapter::extendWrite(const InputTables& absTables,
                            const std::string& fileName) const {
    throw Exception{"Writing to C3D not supported yet."};
}

} // namespace OpenSim
