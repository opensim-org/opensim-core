#include "TRCFileAdapter.h"
#include <fstream>

namespace OpenSim {

const std::string TRCFileAdapter::delimiter_write_{"\t"};
const std::string TRCFileAdapter::delimiters_read_{" \t"};
const std::string TRCFileAdapter::newline_{"\n"};
const std::string TRCFileAdapter::frame_num_column_label_{"Frame#"};
const std::string TRCFileAdapter::time_column_label_{"Time"};
const std::string TRCFileAdapter::x_label_{"X"};
const std::string TRCFileAdapter::y_label_{"Y"};
const std::string TRCFileAdapter::z_label_{"Z"};
const std::string TRCFileAdapter::num_markers_label_{"NumMarkers"};
const std::string TRCFileAdapter::num_frames_label_{"NumFrames"};
const unsigned    TRCFileAdapter::data_starts_at_row_{7};
const std::vector<std::string> TRCFileAdapter::metadata_keys_{"DataRate", 
        "CameraRate", "NumFrames", "NumMarkers", "Units", "OrigDataRate", 
        "OrigDataStartFrame", "OrigNumFrames"};

TRCFileAdapter* 
TRCFileAdapter::clone() const {
    return new TRCFileAdapter{*this};
}

std::unique_ptr<TRCFileAdapter::Table>
TRCFileAdapter::read(const std::string& fileName) const {
    auto abs_table = extendRead(fileName).at(0).release();
    return std::unique_ptr<Table>{static_cast<Table*>(abs_table)};
}

void 
TRCFileAdapter::write(const TRCFileAdapter::Table& table, 
                      const std::string& fileName) const {
    extendWrite({&table}, fileName);
}

TRCFileAdapter::OutputTables
TRCFileAdapter::extendRead(const std::string& fileName) const {
    if(fileName.empty())
        throw Exception{"Input filename is not set."};

    std::ifstream in_stream{fileName};

    std::unique_ptr<Table> table{new Table{}};

    // First line of the stream is considered the header.
    std::string line{};
    std::getline(in_stream, line);
    table->updTableMetaData().setValueForKey("header", line);

    // Read the line containing metadata keys.
    std::getline(in_stream, line);
    auto keys = tokenize(line, delimiters_read_);

    // Read the line containing metadata values.
    std::getline(in_stream, line);
    auto values = tokenize(line, delimiters_read_);

    if(keys.size() != values.size())
        throw Exception{"Number of metadata keys and values do not match"};

    // Fill up the metadata container.
    for(std::size_t i = 0; i < keys.size(); ++i)
        table->updTableMetaData().setValueForKey(keys[i], values[i]);

    // Read the line containing column labels and fill up the column labels
    // container.
    std::getline(in_stream, line);
    auto column_labels = tokenize(line, delimiters_read_);

    // Column 0 should be the frame number. Check and get rid of it as it is
    // not used. The whole column is discarded as the data is read in.
    if(column_labels[0] != frame_num_column_label_)
        throw Exception{"Expected label for column 0 to be '" +
                frame_num_column_label_ + "' but found it to be '" +
                column_labels[0] + "'."};
    column_labels.erase(column_labels.begin());

    // Column 0 (originally column 1 before removing frame number) should
    // now be the time column. Check and get rid of it. The data in this
    // column is maintained separately from rest of the data.
    if(column_labels[0] != time_column_label_)
        throw Exception{"Expected label for column 1 to be '" +
                time_column_label_ + "' but found it to be '" +
                column_labels[0] + "'."};
    column_labels.erase(column_labels.begin());

    // Read in the next line of column labels containing (Xdd, Ydd, Zdd)
    // tuples where dd is a 1 or 2 digit subscript. For example --
    // X1, Y1, Z1, X2, Y2, Z2, ... so on.
    // Check and ignore these labels.
    std::getline(in_stream, line);
    auto xyz_labels_found = tokenize(line, delimiters_read_);
    auto num_markers_expected = 
        std::stoul(table->
                   getTableMetaData().
                   getValueForKey(num_markers_label_).
                   template getValue<std::string>());
    decltype(xyz_labels_found) xyz_labels_expected{};
    xyz_labels_expected.reserve(num_markers_expected * 3);
    for(int i = 1; i <= num_markers_expected; ++i)
        for(auto& letter : {x_label_, y_label_, z_label_})
            xyz_labels_expected.push_back(letter + std::to_string(i));
    if(xyz_labels_found != xyz_labels_expected)
        throw Exception{"Expected secondary column labels to "
                "be of form X1, Y1, Z1, X2, Y2, Z2, ... so on. There must "
                "be one (X,Y,Z) triplet per marker."};

    // Read the rows one at a time and fill up the time column container and
    // the data container.
    auto num_frames_expected = 
        std::stoul(table->
                   getTableMetaData().
                   getValueForKey(num_frames_label_).
                   template getValue<std::string>());
    std::size_t row_num{data_starts_at_row_ - 1};
    while(std::getline(in_stream, line)) {
        auto row = tokenize(line, delimiters_read_);
        ++row_num;

        if(row.size() != column_labels.size() * 3 + 2)
            throw Exception{"There are " + 
                    std::to_string(column_labels.size() * 3 + 2) + 
                    " column labels but row " + std::to_string(row_num) + 
                    " contains " + std::to_string(row.size()) + " columns."};

        // Columns 2 till the end are data.
        auto num_markers_expected = 
            std::stoi(table->
                      getTableMetaData().
                      getValueForKey(num_markers_label_).
                      template getValue<std::string>());
        Table::RowVector row_vector{num_markers_expected};
        for(std::size_t c = 2; c < column_labels.size() * 3 + 2; c += 3)
            row_vector[c] = SimTK::Vec3{std::stod(row[c]),
                                        std::stod(row[c+1]),
                                        std::stod(row[c+2])};

        // Column 1 is time.
        table->appendRow(std::stod(row[1]), std::move(row_vector));
    }

    // Set the column labels of the table.
    ValueArray<std::string> value_array{};
    for(const auto& cl : column_labels)
        value_array.upd().push_back(SimTK::Value<std::string>{cl});
    Table::DependentsMetaData dep_metadata{};
    dep_metadata.setValueForKey("labels", value_array);
    table->setDependentsMetaData(dep_metadata);

    OutputTables output_tables{};
    output_tables.emplace_back(table.release());

    return std::move(output_tables);
}

void
TRCFileAdapter::extendWrite(const InputTables& absTables, 
                            const std::string& fileName) const {
    auto& table = dynamic_cast<const Table&>(*absTables[0]);

    if(fileName.empty())
        throw Exception{"Input filename is not set."};

    std::ofstream out_stream{fileName};

    // First line of the stream is the header.
    try {
    out_stream << table.
                  getTableMetaData().
                  getValueForKey("header").
                  getValue<std::string>() << std::endl;
    } catch(std::out_of_range&) {
        out_stream << "PathFileType\t4\t(X/Y/Z)\t" << fileName << std::endl;
    }

    // Line containing metadata keys.
    out_stream << metadata_keys_[0];
    for(unsigned i = 1; i < metadata_keys_.size(); ++i)
        out_stream << delimiter_write_ << metadata_keys_[i];
    out_stream << std::endl;

    // Line containing metadata values.
    std::string datarate;
    try {
        datarate = table.
                   getTableMetaData().
                   getValueForKey(metadata_keys_[0]).
                   getValue<std::string>();
    } catch(std::out_of_range&) {
        throw Exception{"Metadata 'DataRate' missing."};
    }
    out_stream << datarate << delimiter_write_;
    try {
        out_stream << table.
                      getTableMetaData().
                      getValueForKey(metadata_keys_[1]).
                      getValue<std::string>()
                   << delimiter_write_;
    } catch(std::out_of_range&) {
        out_stream << datarate << delimiter_write_;
    }
    try {
        out_stream << table.
                      getTableMetaData().
                      getValueForKey(metadata_keys_[2]).
                      getValue<std::string>()
                   << delimiter_write_;
    } catch(std::out_of_range&) {
        out_stream << table.getNumRows() << delimiter_write_;
    }
    try {
        out_stream << table.
                      getTableMetaData().
                      getValueForKey(metadata_keys_[3]).
                      getValue<std::string>()
                   << delimiter_write_;
    } catch(std::out_of_range&) {
        out_stream << table.getNumColumns() << delimiter_write_;
    }
    try {
        out_stream << table.
                      getTableMetaData().
                      getValueForKey(metadata_keys_[4]).
                      getValue<std::string>()
                   << delimiter_write_;
    } catch(std::out_of_range&) {
        throw Exception{"Metadata 'Units' missing."};
    }
    try {
        out_stream << table.
                      getTableMetaData().
                      getValueForKey(metadata_keys_[5]).
                      getValue<std::string>()
                   << delimiter_write_;
    } catch(std::out_of_range&) {
        out_stream << datarate << delimiter_write_;
    }
    try {
        out_stream << table.
                      getTableMetaData().
                      getValueForKey(metadata_keys_[6]).
                      getValue<std::string>()
                   << delimiter_write_;
    } catch(std::out_of_range&) {
        out_stream << 0 << delimiter_write_;
    }
    try {
        out_stream << table.
                      getTableMetaData().
                      getValueForKey(metadata_keys_[7]).
                      getValue<std::string>();
    } catch(std::out_of_range&) {
        out_stream << table.getNumRows();
    }
    out_stream << std::endl;

    // Line containing column labels.
    out_stream << frame_num_column_label_ << delimiter_write_
               << time_column_label_      << delimiter_write_;
    for(unsigned col = 0; col < table.getNumColumns(); ++col)
        out_stream << table.
                      getDependentsMetaData().
                      getValueArrayForKey("labels")[col].
                      getValue<std::string>()
                   << delimiter_write_ << delimiter_write_ << delimiter_write_;
    out_stream << std::endl;

    // Line containing xyz component labels for each marker.
    out_stream << delimiter_write_ << delimiter_write_;
    for(unsigned col = 0; col <= table.getNumColumns(); ++col)
        for(auto& letter : {x_label_, y_label_, z_label_})
            out_stream << (letter + std::to_string(col)) << delimiter_write_;
    out_stream << std::endl;

    // Empty line.
    out_stream << std::endl;

    // Data rows.
    for(unsigned row = 0; row < table.getNumRows(); ++row) {
        out_stream << row                  << delimiter_write_ 
                   << table.getIndependentColumn()[row] << delimiter_write_;
        for(unsigned col = 0; col < table.getNumColumns(); ++col) {
            const auto& row_r = table.getRowAtIndex(row);
            const auto& elt = row_r[col];
            out_stream << elt[0] << delimiter_write_
                       << elt[1] << delimiter_write_
                       << elt[2] << delimiter_write_;
        }
        out_stream << std::endl;
    }
}

}
