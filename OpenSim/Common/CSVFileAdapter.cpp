#include "CSVFileAdapter.h"

namespace OpenSim {

CSVFileAdapter::CSVFileAdapter() :
    DelimFileAdapter(",", // delimiter for read
                     ","  // delimiter for write
                     ) {}

CSVFileAdapter*
CSVFileAdapter::clone() const {
    return new CSVFileAdapter{*this};
}

TimeSeriesTable
CSVFileAdapter::read(const std::string& fileName) {
    auto abs_table = CSVFileAdapter{}.extendRead(fileName).at(_table);
    return static_cast<TimeSeriesTable&>(*abs_table);
}

void 
CSVFileAdapter::write(const TimeSeriesTable& table, 
                        const std::string& fileName) {
    InputTables tables{};
    tables.emplace(_table, &table);
    CSVFileAdapter{}.extendWrite(tables, fileName);
}

}
