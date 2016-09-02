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
                    return std::make_shared<STOFileAdapter_<double>>();
                else if(value == "Vec2")
                    return std::make_shared<STOFileAdapter_<Vec2>>();
                else if(value == "Vec3")
                    return std::make_shared<STOFileAdapter_<Vec3>>();
                else if(value == "Vec4")
                    return std::make_shared<STOFileAdapter_<Vec4>>();
                else if(value == "Vec5")
                    return std::make_shared<STOFileAdapter_<Vec5>>();
                else if(value == "Vec6")
                    return std::make_shared<STOFileAdapter_<Vec6>>();
                else if(value == "Vec7")
                    return std::make_shared<STOFileAdapter_<Vec7>>();
                else if(value == "Vec8")
                    return std::make_shared<STOFileAdapter_<Vec8>>();
                else if(value == "Vec9")
                    return std::make_shared<STOFileAdapter_<Vec9>>();
                else if(value == "Vec10")
                    return std::make_shared<STOFileAdapter_<Vec<10>>>();
                else if(value == "Vec11")
                    return std::make_shared<STOFileAdapter_<Vec<11>>>();
                else if(value == "Vec12")
                    return std::make_shared<STOFileAdapter_<Vec<12>>>();
                else if(value == "UnitVec3")
                    return std::make_shared<STOFileAdapter_<UnitVec3>>();
                else if(value == "Quaternion")
                    return std::make_shared<STOFileAdapter_<Quaternion>>();
                else if(value == "SpatialVec")
                    return std::make_shared<STOFileAdapter_<SpatialVec>>();
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

    // Try derived class before base class.
    
    try {
        using Table = const TimeSeriesTable_<UnitVec3>;
        auto table = dynamic_cast<Table&>(absTable);
        return std::make_shared<STOFileAdapter_<UnitVec3>>();
    } catch(const std::bad_cast&) {}
    try {
        using Table = const TimeSeriesTable_<Quaternion>;
        auto table = dynamic_cast<Table&>(absTable);
        return std::make_shared<STOFileAdapter_<Quaternion>>();
    } catch(const std::bad_cast&) {}
    try {
        using Table = const TimeSeriesTable_<SpatialVec>;
        auto table = dynamic_cast<Table&>(absTable);
        return std::make_shared<STOFileAdapter_<SpatialVec>>();
    } catch(const std::bad_cast&) {}
    try {
        using Table = const TimeSeriesTable_<double>;
        auto table = dynamic_cast<Table&>(absTable);
        return std::make_shared<STOFileAdapter_<double>>();
    } catch(const std::bad_cast&) {}
    try {
        using Table = const TimeSeriesTable_<Vec2>;
        auto table = dynamic_cast<Table&>(absTable);
        return std::make_shared<STOFileAdapter_<Vec2>>();
    } catch(const std::bad_cast&) {}
    try {
        using Table = const TimeSeriesTable_<Vec3>;
        auto table = dynamic_cast<Table&>(absTable);
        return std::make_shared<STOFileAdapter_<Vec3>>();
    } catch(const std::bad_cast&) {}
    try {
        using Table = const TimeSeriesTable_<Vec4>;
        auto table = dynamic_cast<Table&>(absTable);
        return std::make_shared<STOFileAdapter_<Vec4>>();
    } catch(const std::bad_cast&) {}
    try {
        using Table = const TimeSeriesTable_<Vec5>;
        auto table = dynamic_cast<Table&>(absTable);
        return std::make_shared<STOFileAdapter_<Vec5>>();
    } catch(const std::bad_cast&) {}
    try {
        using Table = const TimeSeriesTable_<Vec6>;
        auto table = dynamic_cast<Table&>(absTable);
        return std::make_shared<STOFileAdapter_<Vec6>>();
    } catch(const std::bad_cast&) {}
    try {
        using Table = const TimeSeriesTable_<Vec7>;
        auto table = dynamic_cast<Table&>(absTable);
        return std::make_shared<STOFileAdapter_<Vec7>>();
    } catch(const std::bad_cast&) {}
    try {
        using Table = const TimeSeriesTable_<Vec8>;
        auto table = dynamic_cast<Table&>(absTable);
        return std::make_shared<STOFileAdapter_<Vec8>>();
    } catch(const std::bad_cast&) {}
    try {
        using Table = const TimeSeriesTable_<Vec9>;
        auto table = dynamic_cast<Table&>(absTable);
        return std::make_shared<STOFileAdapter_<Vec9>>();
    } catch(const std::bad_cast&) {}
    try {
        using Table = const TimeSeriesTable_<Vec<10>>;
        auto table = dynamic_cast<Table&>(absTable);
        return std::make_shared<STOFileAdapter_<Vec<10>>>();
    } catch(const std::bad_cast&) {}
    try {
        using Table = const TimeSeriesTable_<Vec<11>>;
        auto table = dynamic_cast<Table&>(absTable);
        return std::make_shared<STOFileAdapter_<Vec<11>>>();
    } catch(const std::bad_cast&) {}
    try {
        using Table = const TimeSeriesTable_<Vec<12>>;
        auto table = dynamic_cast<Table&>(absTable);
        return std::make_shared<STOFileAdapter_<Vec<12>>>();
    } catch(const std::bad_cast&) {}

    OPENSIM_THROW(STODataTypeNotSupported,
                  "<unknown>");
}

}
