#include "MOTFileAdapter.h"

namespace OpenSim {

MOTFileAdapter::MOTFileAdapter() :
    DelimFileAdapter(" \t", // delimites for read
                     "\t"   // delimiter for write
                     ) {}

MOTFileAdapter*
MOTFileAdapter::clone() const {
    return new MOTFileAdapter{*this};
}

TimeSeriesTable
MOTFileAdapter::read(const std::string& fileName) {
    auto abs_table = MOTFileAdapter{}.extendRead(fileName).at(_table);
    return static_cast<TimeSeriesTable&>(*abs_table);
}

void 
MOTFileAdapter::write(const TimeSeriesTable& table, 
                        const std::string& fileName) {
    InputTables tables{};
    tables.emplace(_table, &table);
    MOTFileAdapter{}.extendWrite(tables, fileName);
}

}
