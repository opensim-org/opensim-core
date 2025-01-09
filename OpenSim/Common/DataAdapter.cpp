#include "Adapters.h"

namespace OpenSim {

DataAdapter::RegisteredDataAdapters 
DataAdapter::_registeredDataAdapters{};

bool 
DataAdapter::registerDataAdapter(const std::string& identifier,
                                 const DataAdapter& adapter) {
    OPENSIM_THROW_IF(_registeredDataAdapters.find(identifier) != 
                     _registeredDataAdapters.end(),
                     DataAdapterAlreadyRegistered,
                     identifier);

    auto result = _registeredDataAdapters.emplace(identifier, 
                                 std::shared_ptr<DataAdapter>{adapter.clone()});

    return result.second;
}

std::shared_ptr<DataAdapter> 
DataAdapter::createAdapter(const std::string& identifier) {
    try {
        DataAdapter* adapter = 
            _registeredDataAdapters.at(identifier)->clone();
        return std::shared_ptr<DataAdapter>{adapter};
    } catch(std::out_of_range&) {
        OPENSIM_THROW(NoRegisteredDataAdapter,
                      identifier);
    }
}

namespace {

// Automatic registration of OpenSim adapters. There will be one call per 
// registration. The return values (of type bool) are AND(ed) together to
// initialize this variable. This variable exists only for this purpose.
// bool 
// registerAdapters{DataAdapter::registerDataAdapter("trc", TRCFileAdapter{}) &&
//                  DataAdapter::registerDataAdapter("csv", CSVFileAdapter{}) &&
//                  DataAdapter::registerDataAdapter("mot", STOFileAdapter{})};
bool 
registerAdapters{DataAdapter::registerDataAdapter("trc", TRCFileAdapter{}) 
        && DataAdapter::registerDataAdapter("mot", STOFileAdapter_<double>{}) 
        && DataAdapter::registerDataAdapter("csv", CSVFileAdapter{})
#if defined (WITH_EZC3D)
              && DataAdapter::registerDataAdapter("c3d", C3DFileAdapter{})
#endif
                };

}

// Ignore this function. This exists to suppress compiler warning
// about variable 'registerAdapters' being 'unused'.
void ignore() {
    registerAdapters = false;
}

}
