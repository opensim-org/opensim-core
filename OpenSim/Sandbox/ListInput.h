
#define OpenSim_DECLARE_INPUT(name, T, stage) \
int _input_##name { constructInput<T>(name, stage) }; \
const Input<T>& getInput_##name() const; \
Input<T> updInput_##name(); \
const T& getInputValue_##name() const; \

#define OpenSim_DECLARE_LIST_INPUT(name, T, stage) \
int _input_##name { constructListInput<T>(name, stage) }; \
const Input<T>& getInput_##name() const; \
Input<T> updInput_##name(); \
OutputValuesIterator getInputValues_##name() const; \


#define OpenSim_DECLARE_SOCKET(name, T) \
int _socket_##name { constructSocket<T>(name) }; \
const T& getConnectee_##name getConnectee_##name() const;

#define OpenSim_DECLARE_LIST_SOCKET(name, T) \
int _socket_##name { constructListSocket<T>(name) }; \
Socket<T>::iterator getConnectees_##name() const;

class Component : public Object {
public:
x   AbstractInput& updInput(std::string name);
x   const AbstractInput& getInput(std::string name) const;
    
    template <typename T>
x   const Input<T>& getInput(std::string name) const;
    
    template <typename T>
x   const T& getInputValue(const SimTK::State&, std::string name); // exception if listinput.
    template <typename T>
x   Input<T>::OutputValuesIterator& getInputValues(const SimTK::State&, std::string name) const;
    
    template <typename T>
    const T& getConnectee(std::string name) const; // exception if listsocket
    
    template <typename T>
    Socket<T>::iterator getConnectees(std::string) const;
    
protected:
    template <typename T>
x   void constructInput(std::string name, SimTK::Stage);
    template <typename T>
x   void constructListInput(std::string name, SimTK::Stage
    /*, int minSize, int maxSize*/);
    
    template <typename T>
x   void constructSocket(std::string name);
    template <typename T>
x   void constructListSocket(std::string name);
};

class AbstractSocket : public Object {
public:
    OpenSim_DECLARE_LIST_PROPERTY(connectee_name, std::string);
    AbstractSocket();
    AbstractSocket(std::string name, SimTK::Stage, bool isListSocket);
    SimTK::Stage getConnectAtStage() const;
    virtual bool isConnected() const = 0;
    virtual std::string getConnecteeTypeName() const = 0;
    virtual void connect(const Object& connectee) = 0;
    virtual void findAndConnect(const Component& root) = 0;
    virtual void disconnect() = 0;
    bool isListSocket() const;
    virtual size_t getNumConnectees() const = 0;
};

template <class T>
class Socket : public AbstractSocket {
public:
    Socket();
    Socket(std::string name, SimTK::Stage, bool isListSocket);
    bool isConnected() const override;
    size_t getNumConnectees() const override;
    const T& getConnectee(int index=-1) const;
    typedef std::vector<SimTK::ReferencePtr<const T>>::const_iterator const_iterator;
    SimTK::IteratorRange<iterator> getConnectees() const;
    void connect(const Object&) override;  // appends if can.
    void findAndConnect(const Component&) override;
    void disconnect() override;
    std::string getConnecteeTypeName() const override;
};

class AbstractInput : public AbstractSocket {
x   AbstractInput();
x   AbstractInput(std::string name, SimTK::Stage, bool isListSocket);
x   void connect(const Object&) override;
x   virtual void connect(const AbstractOutput&) const = 0; // appends if can.
    // TODO iterator through connectees.
};

template <class T>
class Input : public AbstractInput {
x   Input();
x   Input(std::string name, SimTK::Stage, bool isListSocket);
x   void connect(const AbstractOutput&) const override;
x   void disconnect() override;
x   bool isConnected() const override;
x   size_t getNumConnectees() const override;
x   std::string getConnecteeTypeName() const override;
x   void findAndConnect(const Component&) override;
x   const T& getValue(const SimTK::State&, size_t index=-1) const;
x   class OutputValueIterator;
x   SimTK::IteratorRange<OutputValueIterator> getValues() const;
};






