#include "DataAdapter.h"


namespace OpenSim {

DataAdapter::RegisteredDataAdapters 
DataAdapter::registered_data_adapters{};

void 
DataAdapter::registerDataAdapter(const std::string& identifier,
                                 const DataAdapter& adapter) {
    if(registered_data_adapters.find(identifier) != 
       registered_data_adapters.end())
        throw Exception{"DataAdapter::registerDataAdapter() adapter for '" +
                identifier + "' already registered."};

    auto kv = std::make_pair(identifier, 
                             std::unique_ptr<DataAdapter>{adapter.clone()});

    registered_data_adapters.insert(std::move(kv));
}

std::unique_ptr<DataAdapter> 
DataAdapter::createAdapter(const std::string& identifier) {
    try {
        DataAdapter* adapter = 
            registered_data_adapters.at(identifier)->clone();
        return std::unique_ptr<DataAdapter>{adapter};
    } catch(std::out_of_range&) {
        throw Exception{"No DataAdapter was found among the "
                "registered DataAdapters for the identifier: " + identifier 
                + ". DataAdapters must be registered before use. If "
                "multiple DataAdapters registered for same identifier, the "
                "latest registration is kept."};
    }
}

}
