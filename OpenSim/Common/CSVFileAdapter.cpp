#include "CSVFileAdapter.h"

namespace OpenSim {

const std::string CSVFileAdapter::delimiter_read_{","};
const std::string CSVFileAdapter::delimiter_write_{","};
const std::string CSVFileAdapter::time_column_label_{"time"};

CSVFileAdapter*
CSVFileAdapter::clone() const {
    return new CSVFileAdapter{*this};
}

std::unique_ptr<CSVFileAdapter::Table>
CSVFileAdapter::read(const std::string& fileName) const {
    auto abs_table = extendRead(fileName).at(0).release();
    return std::unique_ptr<Table>{static_cast<Table*>(abs_table)};
}

void 
CSVFileAdapter::write(const CSVFileAdapter::Table& table, 
                      const std::string& fileName) const {
    extendWrite({&table}, fileName);
}

CSVFileAdapter::OutputTables 
CSVFileAdapter::extendRead(const std::string& fileName) const {
    if(fileName.empty())
        throw Exception{"Input filename is not set."};

    std::ifstream in_stream{fileName};

    std::unique_ptr<Table> table{new Table{}};

    // First line has the column labels.
    std::string line{};
    std::getline(in_stream, line);
    auto column_labels = tokenize(line, delimiter_read_);
    column_labels.erase(column_labels.begin());

    // Set the column labels of the table.
    ValueArray<std::string> value_array{};
    for(const auto& cl : column_labels)
        value_array.upd().push_back(SimTK::Value<std::string>{cl});
    Table::DependentsMetaData dep_metadata{};
    dep_metadata.setValueForKey("labels", value_array);
    table->setDependentsMetaData(dep_metadata);

    // Read the rows one at a time.
    std::size_t row_num{0};
    while(std::getline(in_stream, line)) {
        auto row = tokenize(line, delimiter_read_);
        ++row_num;
        
        if(row.size() != column_labels.size() + 1)
            throw Exception{"There are " + std::to_string(column_labels.size())
                    + " column labels but row " + std::to_string(row_num) + 
                    " contains " + std::to_string(row.size()) + " columns."};

        // Columns 2 till the end are data.
        Table::RowVector row_vector{static_cast<int>(row.size() - 1)};
        for(unsigned i = 1; i < row.size(); ++i)
            row_vector[i - 1] = std::stod(row[i]);

        // Column 0 is time.
        table->appendRow(std::stod(row[0]), std::move(row_vector));
    }

    OutputTables output_tables{};
    output_tables.emplace_back(table.release());

    return std::move(output_tables);
}

void
CSVFileAdapter::extendWrite(const InputTables& absTables,
                            const std::string& fileName) const {
    auto& table = dynamic_cast<const Table&>(*absTables[0]);

    if(fileName.empty())
        throw Exception{"Input filename is not set."};

    std::ofstream out_stream{fileName};

    // First line is column labels.
    out_stream << time_column_label_;
    for(unsigned col = 0; col < table.getNumColumns(); ++col)
        out_stream << delimiter_write_ << table.
                                          getDependentsMetaData().
                                          getValueArrayForKey("labels")[col];
    out_stream << std::endl;

    // Data rows.
    for(unsigned row = 0; row < table.getNumRows(); ++row) {
        out_stream << table.getIndependentColumn()[row];
        for(unsigned col = 0; col < table.getNumColumns(); ++col)
            out_stream << delimiter_write_ << table.getRowAtIndex(row)[col];
        out_stream << std::endl;
    }
}

} // namespace OpenSim
