#include "TRCFileAdapter.h"
#include <fstream>

namespace OpenSim {

const std::string TRCFileAdapter::_markers{"markers"};
const std::string TRCFileAdapter::_delimiterWrite{"\t"};
const std::string TRCFileAdapter::_delimitersRead{" \t"};
const std::string TRCFileAdapter::_frameNumColumnLabel{"Frame#"};
const std::string TRCFileAdapter::_timeColumnLabel{"Time"};
const std::string TRCFileAdapter::_xLabel{"X"};
const std::string TRCFileAdapter::_yLabel{"Y"};
const std::string TRCFileAdapter::_zLabel{"Z"};
const std::string TRCFileAdapter::_numMarkersLabel{"NumMarkers"};
const std::string TRCFileAdapter::_numFramesLabel{"NumFrames"};
const unsigned    TRCFileAdapter::_dataStartsAtLine{6};
const std::vector<std::string> TRCFileAdapter::_metadataKeys{"DataRate", 
        "CameraRate", "NumFrames", "NumMarkers", "Units", "OrigDataRate", 
        "OrigDataStartFrame", "OrigNumFrames"};

TRCFileAdapter* 
TRCFileAdapter::clone() const {
    return new TRCFileAdapter{*this};
}

TimeSeriesTableVec3
TRCFileAdapter::read(const std::string& fileName) {
    auto abs_table = TRCFileAdapter{}.extendRead(fileName).at(_markers);
    return static_cast<TimeSeriesTableVec3&>(*abs_table);
}

void 
TRCFileAdapter::write(const TimeSeriesTableVec3& table, 
                      const std::string& fileName) {
    InputTables tables{};
    tables.emplace(_markers, &table);
    TRCFileAdapter{}.extendWrite(tables, fileName);
}

TRCFileAdapter::OutputTables
TRCFileAdapter::extendRead(const std::string& fileName) const {
    OPENSIM_THROW_IF(fileName.empty(),
                     EmptyFileName);

    std::ifstream in_stream{fileName};
    OPENSIM_THROW_IF(!in_stream.good(),
                     FileDoesNotExist,
                     fileName);

    auto table = std::make_shared<TimeSeriesTableVec3>();

    // Callable to get the next line in form of vector of tokens.
    auto nextLine = [&] {
        return getNextLine(in_stream, _delimitersRead);
    };

    // First line of the stream is considered the header.
    std::string header{};
    std::getline(in_stream, header);
    auto header_tokens = tokenize(header, _delimitersRead);
    OPENSIM_THROW_IF(header_tokens.empty(),
                     FileIsEmpty,
                     fileName);        
    OPENSIM_THROW_IF(header_tokens.at(0) != "PathFileType",
                     MissingHeader);
    table->updTableMetaData().setValueForKey("header", header);

    // Read the line containing metadata keys.
    auto keys = nextLine();
    OPENSIM_THROW_IF(keys.size() != _metadataKeys.size(),
                     IncorrectNumMetaDataKeys,
                     fileName,
                     _metadataKeys.size(), 
                     keys.size());

    for(size_t i = 0; i < keys.size(); ++i)
        OPENSIM_THROW_IF(keys[i] != _metadataKeys[i],
                         UnexpectedMetaDataKey,
                         fileName,
                         _metadataKeys[i],
                         keys[i]);

    // Read the line containing metadata values.
    auto values = nextLine();
    OPENSIM_THROW_IF(keys.size() != values.size(),
                     MetaDataLengthMismatch,
                     fileName,
                     keys.size(),
                     values.size());

    // Fill up the metadata container.
    for(std::size_t i = 0; i < keys.size(); ++i)
        table->updTableMetaData().setValueForKey(keys[i], values[i]);

    auto num_markers_expected = 
        std::stoul(table->
                   getTableMetaData().
                   getValueForKey(_numMarkersLabel).
                   template getValue<std::string>());

    // Read the line containing column labels and fill up the column labels
    // container.
    auto column_labels = nextLine();
    OPENSIM_THROW_IF(column_labels.size() != num_markers_expected + 2,
                     IncorrectNumColumnLabels,
                     fileName,
                     num_markers_expected + 2,
                     column_labels.size());

    // Column 0 should be the frame number. Check and get rid of it as it is
    // not used. The whole column is discarded as the data is read in.
    OPENSIM_THROW_IF(column_labels[0] != _frameNumColumnLabel,
                     UnexpectedColumnLabel,
                     fileName,
                     _frameNumColumnLabel,
                     column_labels[0]);
    column_labels.erase(column_labels.begin());

    // Column 0 (originally column 1 before removing frame number) should
    // now be the time column. Check and get rid of it. The data in this
    // column is maintained separately from rest of the data.
    OPENSIM_THROW_IF(column_labels[0] != _timeColumnLabel,
                     UnexpectedColumnLabel,
                     fileName,
                     _timeColumnLabel,
                     column_labels[0]);
    column_labels.erase(column_labels.begin());

    // Read in the next line of column labels containing (Xdd, Ydd, Zdd)
    // tuples where dd is a 1 or 2 digit subscript. For example --
    // X1, Y1, Z1, X2, Y2, Z2, ... so on.
    // Check and ignore these labels.
    auto xyz_labels_found = nextLine();
    for(unsigned i = 1; i <= num_markers_expected; ++i) {
        unsigned j = 0;
        for(auto& letter : {_xLabel, _yLabel, _zLabel}) {
            const unsigned ind = ((i - 1) * 3) + j++;
            const std::string expected{letter + std::to_string(i)};
            OPENSIM_THROW_IF(xyz_labels_found.at(ind) != expected,
                             UnexpectedColumnLabel,
                             fileName,
                             expected,
                             xyz_labels_found.at(ind));
        }
    }

    // Read the rows one at a time and fill up the time column container and
    // the data container.
    std::size_t line_num{_dataStartsAtLine - 1};
    auto row = nextLine();
    while(!row.empty()) {
        ++line_num;
        size_t expected{column_labels.size() * 3 + 2};
        OPENSIM_THROW_IF(row.size() != expected,
                         RowLengthMismatch,
                         fileName,
                         line_num,
                         expected,
                         row.size());

        // Columns 2 till the end are data.
        TimeSeriesTableVec3::RowVector 
            row_vector{static_cast<int>(num_markers_expected)};
        int ind{0};
        for(std::size_t c = 2; c < column_labels.size() * 3 + 2; c += 3)
            row_vector[ind++] = SimTK::Vec3{std::stod(row.at(c)),
                                            std::stod(row.at(c+1)),
                                            std::stod(row.at(c+2))};

        // Column 1 is time.
        table->appendRow(std::stod(row.at(1)), std::move(row_vector));

        row = nextLine();
    }

    // Set the column labels of the table.
    ValueArray<std::string> value_array{};
    for(const auto& cl : column_labels)
        value_array.upd().push_back(SimTK::Value<std::string>{cl});
    TimeSeriesTableVec3::DependentsMetaData dep_metadata{};
    dep_metadata.setValueArrayForKey("labels", value_array);
    table->setDependentsMetaData(dep_metadata);

    OutputTables output_tables{};
    output_tables.emplace(_markers, table);

    return output_tables;
}

void
TRCFileAdapter::extendWrite(const InputTables& absTables, 
                            const std::string& fileName) const {
    OPENSIM_THROW_IF(absTables.empty(),
                     NoTableFound);

    const TimeSeriesTableVec3* table{};
    try {
        auto abs_table = absTables.at(_markers);
        table = dynamic_cast<const TimeSeriesTableVec3*>(abs_table);
    } catch(std::out_of_range) {
        OPENSIM_THROW(KeyMissing,
                      _markers);
    } catch(std::bad_cast&) {
        OPENSIM_THROW(IncorrectTableType);
    }

    OPENSIM_THROW_IF(fileName.empty(),
                     EmptyFileName);

    std::ofstream out_stream{fileName};

    // First line of the stream is the header.
    try {
    out_stream << table->
                  getTableMetaData().
                  getValueForKey("header").
                  getValue<std::string>() << "\n";
    } catch(KeyNotFound&) {
        out_stream << "PathFileType\t4\t(X/Y/Z)\t" << fileName << "\n";
    }

    // Line containing metadata keys.
    out_stream << _metadataKeys[0];
    for(unsigned i = 1; i < _metadataKeys.size(); ++i)
        out_stream << _delimiterWrite << _metadataKeys[i];
    out_stream << "\n";

    // Line containing metadata values.
    std::string datarate;
    try {
        datarate = table->
                   getTableMetaData().
                   getValueForKey(_metadataKeys[0]).
                   getValue<std::string>();
    } catch(KeyNotFound&) {
        OPENSIM_THROW(MissingMetaData,
                      "DataRate");
    }
    out_stream << datarate << _delimiterWrite;
    try {
        out_stream << table->
                      getTableMetaData().
                      getValueForKey(_metadataKeys[1]).
                      getValue<std::string>()
                   << _delimiterWrite;
    } catch(KeyNotFound&) {
        out_stream << datarate << _delimiterWrite;
    }
    try {
        out_stream << table->
                      getTableMetaData().
                      getValueForKey(_metadataKeys[2]).
                      getValue<std::string>()
                   << _delimiterWrite;
    } catch(KeyNotFound&) {
        out_stream << table->getNumRows() << _delimiterWrite;
    }
    try {
        out_stream << table->
                      getTableMetaData().
                      getValueForKey(_metadataKeys[3]).
                      getValue<std::string>()
                   << _delimiterWrite;
    } catch(KeyNotFound&) {
        out_stream << table->getNumColumns() << _delimiterWrite;
    }
    try {
        out_stream << table->
                      getTableMetaData().
                      getValueForKey(_metadataKeys[4]).
                      getValue<std::string>()
                   << _delimiterWrite;
    } catch(KeyNotFound&) {
        OPENSIM_THROW(MissingMetaData,
                      "Units");
    }
    try {
        out_stream << table->
                      getTableMetaData().
                      getValueForKey(_metadataKeys[5]).
                      getValue<std::string>()
                   << _delimiterWrite;
    } catch(KeyNotFound&) {
        out_stream << datarate << _delimiterWrite;
    }
    try {
        out_stream << table->
                      getTableMetaData().
                      getValueForKey(_metadataKeys[6]).
                      getValue<std::string>()
                   << _delimiterWrite;
    } catch(KeyNotFound&) {
        out_stream << 0 << _delimiterWrite;
    }
    try {
        out_stream << table->
                      getTableMetaData().
                      getValueForKey(_metadataKeys[7]).
                      getValue<std::string>();
    } catch(KeyNotFound&) {
        out_stream << table->getNumRows();
    }
    out_stream << "\n";

    // Line containing column labels.
    out_stream << _frameNumColumnLabel << _delimiterWrite
               << _timeColumnLabel     << _delimiterWrite;
    for(unsigned col = 0; col < table->getNumColumns(); ++col)
        out_stream << table->
                      getDependentsMetaData().
                      getValueArrayForKey("labels")[col].
                      getValue<std::string>()
                   << _delimiterWrite << _delimiterWrite << _delimiterWrite;
    out_stream << "\n";

    // Line containing xyz component labels for each marker.
    out_stream << _delimiterWrite << _delimiterWrite;
    for(unsigned col = 1; col <= table->getNumColumns(); ++col)
        for(auto& letter : {_xLabel, _yLabel, _zLabel})
            out_stream << (letter + std::to_string(col)) << _delimiterWrite;
    out_stream << "\n";

    // Empty line.
    out_stream << "\n";

    // Data rows.
    for(unsigned row = 0; row < table->getNumRows(); ++row) {
        constexpr auto prec = std::numeric_limits<double>::digits10 + 1;
        out_stream << row + 1                           << _delimiterWrite
                   << std::setprecision(prec) 
                   << table->getIndependentColumn()[row] << _delimiterWrite;
        const auto& row_r = table->getRowAtIndex(row);
        for(unsigned col = 0; col < table->getNumColumns(); ++col) {
            const auto& elt = row_r[col];
            out_stream << std::setprecision(prec) 
                       << elt[0] << _delimiterWrite
                       << elt[1] << _delimiterWrite
                       << elt[2] << _delimiterWrite;
        }
        out_stream << "\n";
    }
}

}
