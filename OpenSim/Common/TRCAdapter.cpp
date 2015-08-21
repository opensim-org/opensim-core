#include "TRCAdapter.h"
#include <fstream>

namespace OpenSim {

const std::string TRCAdapter::delimiter_write_{"\t"};
const std::string TRCAdapter::delimiters_read_{" \t"};
const std::string TRCAdapter::newline_{"\n"};
const std::string TRCAdapter::frame_num_column_label_{"Frame#"};
const std::string TRCAdapter::time_column_label_{"Time"};
const std::string TRCAdapter::x_label_{"X"};
const std::string TRCAdapter::y_label_{"Y"};
const std::string TRCAdapter::z_label_{"Z"};
const std::string TRCAdapter::num_markers_label_{"NumMarkers"};
const std::string TRCAdapter::num_frames_label_{"NumFrames"};
const unsigned    TRCAdapter::data_starts_at_row_{7};
const std::vector<std::string> TRCAdapter::metadata_keys_{"DataRate", 
        "CameraRate", "NumFrames", "NumMarkers", "Units", "OrigDataRate", 
        "OrigDataStartFrame", "OrigNumFrames"};

TRCAdapter* 
TRCAdapter::clone() const {
    return new TRCAdapter{*this};
}

void 
TRCAdapter::prepareForReading(AbstractDataTable& table) {
    table_ = &dynamic_cast<Table&>(table);
}

void
TRCAdapter::prepareForWriting(const AbstractDataTable& table) {
    table_ = &dynamic_cast<Table&>(const_cast<AbstractDataTable&>(table));
}

void 
TRCAdapter::read() {
    if(filename_.empty())
        throw Exception{"Input filename is not set."};

    std::ifstream in_stream{filename_};

    // First line of the stream is considered the header.
    std::string line{};
    std::getline(in_stream, line);
    table_->insertMetaData("header", line);

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
        table_->insertMetaData(keys[i], values[i]);

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
        std::stoul(table_->getMetaData<std::string>(num_markers_label_));
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
        std::stoul(table_->getMetaData<std::string>(num_frames_label_));
    std::size_t row_num{data_starts_at_row_ - 1};
    while(std::getline(in_stream, line)) {
        auto row = tokenize(line, delimiters_read_);
        ++row_num;

        if(row.size() == 0)
            continue;

        if(row.size() != column_labels.size() * 3 + 2)
            throw Exception{"There are " + 
                    std::to_string(column_labels.size() * 3 + 2) + 
                    " column labels but row " + std::to_string(row_num) + 
                    " contains " + std::to_string(row.size()) + 
                    " columns."};

        // Columns 2 till the end are data.
        std::vector<SimTK::Vec3> row_vector{};
        auto num_markers_expected = 
           std::stoul(table_->getMetaData<std::string>(num_markers_label_));
        row_vector.reserve(num_markers_expected);
        for(std::size_t c = 2; c < column_labels.size() * 3 + 2; c += 3)
            row_vector.push_back({std::stod(row[c]),
                                  std::stod(row[c+1]),
                                  std::stod(row[c+2])});

        // Column 1 is time.
        table_->addTimeAndRow(std::stod(row[1]), std::move(row_vector));
    }

    // Set the column labels of the table.
    table_->setColumnLabels(column_labels);

    // if(table_->getNumRows() != num_frames_expected)
    //     throw Exception{"Expected " + std::to_string(num_frames_expected) + 
    //             " frames but found " + std::to_string(table_->getNumRows()) 
    //             + " frames."};
}

void
TRCAdapter::write() {
    if(filename_.empty())
        throw Exception{"Input filename is not set."};

    std::ofstream out_stream{filename_};

    // First line of the stream is the header.
    out_stream << table_->getMetaData<std::string>("header") << std::endl;

    // Line containing metadata keys.
    out_stream << metadata_keys_[0];
    for(unsigned i = 1; i < metadata_keys_.size(); ++i)
        out_stream << delimiter_write_ << metadata_keys_[i];
    out_stream << std::endl;

    // Line containing metadata values.
    out_stream << table_->getMetaData<std::string>(metadata_keys_[0]);
    for(unsigned i = 1; i < metadata_keys_.size(); ++i)
        out_stream << delimiter_write_ 
                   << table_->getMetaData<std::string>(metadata_keys_[i]);
    out_stream << std::endl;

    // Line containing column labels.
    out_stream << frame_num_column_label_ << delimiter_write_
               << time_column_label_      << delimiter_write_;
    for(unsigned col = 0; col < table_->getNumColumns(); ++col)
        out_stream << table_->getColumnLabel(col) 
                   << delimiter_write_ << delimiter_write_ << delimiter_write_;
    out_stream << std::endl;

    // Line containing xyz component labels for each marker.
    out_stream << delimiter_write_ << delimiter_write_;
    for(unsigned col = 0; col <= table_->getNumColumns(); ++col)
        for(auto& letter : {x_label_, y_label_, z_label_})
            out_stream << (letter + std::to_string(col)) << delimiter_write_;
    out_stream << std::endl;

    // Empty line.
    out_stream << std::endl;

    // Data rows.
    for(unsigned row = 0; row < table_->getNumRows(); ++row) {
        out_stream << row                  << delimiter_write_ 
                   << table_->getTime(row) << delimiter_write_;
        for(unsigned col = 0; col < table_->getNumColumns(); ++col)
            out_stream << table_->getElt(row, col) << delimiter_write_;
        out_stream << std::endl;
    }
}

std::string 
TRCAdapter::getIdentifier() {
    return "trc";
}

}
