#include "STOFileAdapter.h"

namespace OpenSim {

STOFileAdapter::STOFileAdapter() :
    DelimFileAdapter(" \t", // delimites for read
                     "\t"   // delimiter for write
                     ) {}

STOFileAdapter*
STOFileAdapter::clone() const {
    return new STOFileAdapter{*this};
}

TimeSeriesTable
STOFileAdapter::read(const std::string& fileName) {
    auto abs_table = STOFileAdapter{}.extendRead(fileName).at(_table);
    return static_cast<TimeSeriesTable&>(*abs_table);
}

void 
STOFileAdapter::write(const TimeSeriesTable& table, 
                        const std::string& fileName) {
    InputTables tables{};
    tables.emplace(_table, &table);
    STOFileAdapter{}.extendWrite(tables, fileName);
}

}
