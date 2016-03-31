#include "DelimFileAdapter.h"
#include <fstream>
#include <regex>

namespace OpenSim {

const std::string DelimFileAdapter::_table{"table"};
const std::string DelimFileAdapter::_endHeaderString{"endheader"};
const std::string DelimFileAdapter::_timeColumnLabel{"time"};

DelimFileAdapter::DelimFileAdapter(const std::string& delimitersRead,
                                   const std::string& delimiterWrite) :
    _delimitersRead{delimitersRead}, 
    _delimiterWrite{delimiterWrite}
{}

DelimFileAdapter*
DelimFileAdapter::clone() const {
    return new DelimFileAdapter{*this};
}

TimeSeriesTable
DelimFileAdapter::read(const std::string& fileName) const {
    auto abs_table = extendRead(fileName).at(_table);
    return static_cast<TimeSeriesTable&>(*abs_table);
}

void 
DelimFileAdapter::write(const TimeSeriesTable& table, 
                        const std::string& fileName) const {
    InputTables tables{};
    tables.emplace(_table, &table);
    extendWrite(tables, fileName);
}

DelimFileAdapter::OutputTables
DelimFileAdapter::extendRead(const std::string& fileName) const {
    OPENSIM_THROW_IF(fileName.empty(),
                     EmptyFileName);

    std::ifstream in_stream{fileName};
    OPENSIM_THROW_IF(!in_stream.good(),
                     FileDoesNotExist,
                     fileName);

    auto table = std::make_shared<TimeSeriesTable>();

    size_t line_num{};
    // All the lines until "endheader" is header.
    std::regex endheader{R"([ \t]*)" + _endHeaderString + R"([ \t]*)"};
    std::string header{};
    std::string line{};
    while(std::getline(in_stream, line)) {
        ++line_num;
        if(std::regex_match(line, endheader))
            break;

        if(header.empty())
            header = line;
        else
            header += "\n" + line;
    }
    table->updTableMetaData().setValueForKey("header", header);

    // Callable to get the next line in form of vector of tokens.
    auto nextLine = [&] {
        return getNextLine(in_stream, _delimitersRead);
    };

    // Read the line containing column labels and fill up the column labels
    // container.
    auto column_labels = nextLine();
    ++line_num;
    // Column 0 is the time column. Check and get rid of it. The data in this
    // column is maintained separately from rest of the data.
    OPENSIM_THROW_IF(column_labels[0] != _timeColumnLabel,
                     UnexpectedColumnLabel,
                     fileName,
                     _timeColumnLabel,
                     column_labels[0]);
    column_labels.erase(column_labels.begin());
    // Set the column labels as metadata.
    ValueArray<std::string> value_array{};
    for(const auto& cl : column_labels)
        value_array.upd().push_back(SimTK::Value<std::string>{cl});
    TimeSeriesTable::DependentsMetaData dep_metadata{};
    dep_metadata.setValueArrayForKey("labels", value_array);
    table->setDependentsMetaData(dep_metadata);

    // Read the rows one at a time and fill up the time column container and
    // the data container.
    auto row = nextLine();
    while(!row.empty()) {
        ++line_num;
        size_t expected{column_labels.size() + 1};
        OPENSIM_THROW_IF(row.size() != expected,
                         RowLengthMismatch,
                         fileName,
                         line_num,
                         expected,
                         row.size());

        // Columns 2 till the end are data.
        TimeSeriesTable::RowVector 
            row_vector{static_cast<int>(column_labels.size())};
        for(std::size_t c = 0; c < column_labels.size(); ++c)
            row_vector[static_cast<int>(c)] = std::stod(row.at(c + 1));

        // Column 1 is time.
        table->appendRow(std::stod(row.at(0)), std::move(row_vector));

        row = nextLine();
    }

    OutputTables output_tables{};
    output_tables.emplace(_table, table);

    return output_tables;
}

void
DelimFileAdapter::extendWrite(const InputTables& absTables, 
                              const std::string& fileName) const {
    OPENSIM_THROW_IF(absTables.empty(),
                     NoTableFound);

    const TimeSeriesTable* table{};
    try {
        auto abs_table = absTables.at(_table);
        table = dynamic_cast<const TimeSeriesTable*>(abs_table);
    } catch(std::out_of_range&) {
        OPENSIM_THROW(KeyMissing,
                      _table);
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
        OPENSIM_THROW(TableMissingHeader);
    }
    out_stream << _endHeaderString << "\n";

    // Line containing column labels.
    out_stream << _timeColumnLabel;
    for(unsigned col = 0; col < table->getNumColumns(); ++col)
        out_stream << _delimiterWrite
                   << table->
                      getDependentsMetaData().
                      getValueArrayForKey("labels")[col].
                      getValue<std::string>();
    out_stream << "\n";

    // Data rows.
    for(unsigned row = 0; row < table->getNumRows(); ++row) {
        constexpr auto prec = std::numeric_limits<double>::digits10 + 1;
        out_stream << std::setprecision(prec)
                   << table->getIndependentColumn()[row];
        const auto& row_r = table->getRowAtIndex(row);
        for(unsigned col = 0; col < table->getNumColumns(); ++col) {
            const auto& elt = row_r[col];
            out_stream << _delimiterWrite << std::setprecision(prec) << elt;
        }
        out_stream << "\n";
    }
}

}
