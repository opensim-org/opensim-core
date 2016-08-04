#include "STOFileAdapter.h"

namespace OpenSim {

std::shared_ptr<DataAdapter> 
createSTOFileAdapterForReading(const std::string& fileName) {
    std::ifstream file{fileName};

    std::regex keyvalue{R"((.*)=(.*))"};
    std::string line{};
    while(std::getline(file, line)) {
        if(line.find("endheader") != std::string::npos)
            break;

        std::smatch matchRes{};
        if(std::regex_match(line, matchRes, keyvalue)) {
            auto key = matchRes[1].str();
            auto value = matchRes[2].str();
            if(!key.empty() && 
               !value.empty() &&
               key.find("DataType") != std::string::npos) {
                using namespace SimTK;

                if(value == "double")
                    return std::make_shared<STOFileAdapter<double>>();
                else if(value == "Vec3")
                    return std::make_shared<STOFileAdapter<Vec3>>();
                else if(value == "Vec6")
                    return std::make_shared<STOFileAdapter<Vec6>>();
                else if(value == "SpatialVec")
                    return std::make_shared<STOFileAdapter<SpatialVec>>();
                else {
                    OPENSIM_THROW(STODataTypeNotSupported,
                                  value);
                }
            }
        }
    }

    OPENSIM_THROW(STODataTypeNotFound);
}

std::shared_ptr<DataAdapter>
createSTOFileAdapterForWriting(const DataAdapter::InputTables& absTables) {
    using namespace SimTK;

    auto& absTable = *absTables.at("table");
    try {
        using Table = const TimeSeriesTable_<double>;
        auto table = dynamic_cast<Table&>(absTable);
        return std::make_shared<STOFileAdapter<double>>();
    } catch(const std::bad_cast&) {}
    try {
        using Table = const TimeSeriesTable_<Vec3>;
        auto table = dynamic_cast<Table&>(absTable);
        return std::make_shared<STOFileAdapter<Vec3>>();
    } catch(const std::bad_cast&) {}
    try {
        using Table = const TimeSeriesTable_<Vec6>;
        auto table = dynamic_cast<Table&>(absTable);
        return std::make_shared<STOFileAdapter<Vec6>>();
    } catch(const std::bad_cast&) {}
    try {
        using Table = const TimeSeriesTable_<SpatialVec>;
        auto table = dynamic_cast<Table&>(absTable);
        return std::make_shared<STOFileAdapter<SpatialVec>>();
    } catch(const std::bad_cast&) {}

    OPENSIM_THROW(STODataTypeNotSupported,
                  "<unknown>");
}

}
