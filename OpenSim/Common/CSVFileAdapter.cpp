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

void 
CSVFileAdapter::write(const TimeSeriesTable& table, 
                        const std::string& fileName) {
    InputTables tables{};
    tables.emplace(tableString(), &table);
    CSVFileAdapter{}.extendWrite(tables, fileName);
}

}
