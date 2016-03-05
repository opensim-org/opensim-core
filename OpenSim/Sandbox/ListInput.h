class Component : public Object {
public:
    AbstractInput& updInput(std::string name);
    const AbstractInput& getInput(std::string name) const;
    template <typename T>
    getInputValue(const SimTK::State, std::string name) const;
protected:
    template <typename T>
    void constructInput(std::string name, SimTK::Stage);
};

class AbstractConnector : public Object {
public:
    OpenSim_DECLARE_LIST_PROPERTY(connectee_name, std::string);
    AbstractConnector();
    AbstractConnector(std::string name, SimTK::Stage);
    SimTK::Stage getConnectAtStage() const;
    virtual bool isConnected() const = 0;
    virtual std::string getConnecteeTypeName() const = 0;
    virtual void connect(const Object& connectee) = 0;
    virtual void findAndConnect(const Component& root) = 0;
    virtual void disconnect() = 0;
};

template <class T>
class Connector : public AbstractConnector {
public:
    Connector();
    Connector(std::string name, SimTK::Stage);
    bool isConnected() const override;
    const T& getConnectee() const;
    void connect(const Object&) override;
    void findAndConnect(const Component&) override;
    void disconnect() override;
    std::string getConnecteeTypeName() const override;
    
};

class AbstractInput : public AbstractConnector {
    AbstractInput();
    AbstractInput(std::string name, SimTK::Stage);
    void connect(const Object&) override;
    virtual void connect(const AbstractOutput&) const = 0;
};

template <class T>
class Input : public AbstractInput {
    Input();
    Input(std::string name, SimTK::Stage);
    void connect(const AbstractOutput&) const override;
    void disconnect() override;
    bool isConnected() const override;
    std::string getConnecteeTypeName() const override;
    void findAndConnect(const Component&) override;
    const T& getValue(const SImTK::State&) const;
};
