#ifndef OPENSIM_COMPONENT_SOCKET_H_
#define OPENSIM_COMPONENT_SOCKET_H_
/* -------------------------------------------------------------------------- *
 *                       OpenSim:  ComponentSocket.h                       *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2016 Stanford University and the Authors                *
 * Author(s): Ajay Seth                                                       *
 *                                                                            *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may    *
 * not use this file except in compliance with the License. You may obtain a  *
 * copy of the License at http://www.apache.org/licenses/LICENSE-2.0.         *
 *                                                                            *
 * Unless required by applicable law or agreed to in writing, software        *
 * distributed under the License is distributed on an "AS IS" BASIS,          *
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.   *
 * See the License for the specific language governing permissions and        *
 * limitations under the License.                                             *
 * -------------------------------------------------------------------------- */

/** @file
 * This file defines the Socket class, which formalizes the dependency of 
 * of a Component on another Object/Component in order to operate, BUT it does 
 * not own it. While Components can be composites (of multiple components) 
 * they often depend on unrelated objects/components that are defined and
 * owned elsewhere.
 *
 * For example a Joint connects two bodies together, but the Joint does 
 * not own either body. Instead, the Joint has Sockets to a parent and 
 * a child body that already exists. The maintenance of the dependency and 
 * the run-time verification of the existence of the bodies is the duty
 * of the Socket.
 */

// INCLUDES
#include "OpenSim/Common/ComponentOutput.h"
#include "OpenSim/Common/ComponentList.h"
//#include <Simbody.h>

namespace OpenSim {

//==============================================================================
/// ComponentSocket Exceptions
//==============================================================================
class InputNotConnected : public Exception {
public:
    InputNotConnected(const std::string& file,
                      size_t line,
                      const std::string& func,
                      const Object& obj,
                      const std::string& inputName) :
        Exception(file, line, func, obj) {
        std::string msg = "Input '" + inputName;
        msg += "' has not been connected.";
        addMessage(msg);
    }
};

//=============================================================================
//                        OPENSIM COMPONENT SOCKET
//=============================================================================
/**
 * A Socket formalizes the dependency between a Component and another object
 * (typically another Component) without owning that object. The object that
 * satisfies the requirements of the Socket will be the "connectee". When a 
 * Socket is satisfied by a connectee we have a successful "connection" or
 * is said to be connected.
 *
 * The purpose of a Socket is to specify: 1) the connectee type that the
 * Component is dependent on, 2) by when (what stage) the socket must be
 * connected in order for the component to function, 3) the name of a connectee
 * that can be found at run-time to satisfy the socket, and 4) whether or
 * not it is connected. A Socket maintains a reference to the instance
 * (connectee) until it is disconnected.
 *
 * For example, a Joint has two Sockets for the parent and child Bodies that
 * it joins. The type for the socket is a PhysicalFrame and any attempt to 
 * connect to a non-Body (or frame rigidly attached to a Body) will throw an
 * exception. The connectAt Stage is Topology. That is, the Joint's connection 
 * to a Body must be performed at the Topology system stage, and any attempt to
 * change the connection status will invalidate that Stage and above.
 *
 * Other Components like a Marker or a Probe that do not change the system
 * topology or add new states could potentially be connected at later stages
 * like Model or Instance.
 *
 * @author  Ajay Seth
 */
class OSIMCOMMON_API AbstractSocket : public Object {
    OpenSim_DECLARE_ABSTRACT_OBJECT(AbstractSocket, Object);
public:
    
//==============================================================================
// PROPERTIES
//==============================================================================
    OpenSim_DECLARE_LIST_PROPERTY(connectee_name, std::string,
        "Name of the component this Socket should be connected to.");
public:
    //--------------------------------------------------------------------------
    // CONSTRUCTION
    //--------------------------------------------------------------------------
    /** Default constructor */
    AbstractSocket() : Object(), connectAtStage(SimTK::Stage::Topology) {
        constructProperties();
    }

    // default destructor, copy constructor

    /** Convenience constructor 
        Create a Socket with specified name and stage at which it
        should be connected.
    @param name             name of the socket, usually describes its 
                            dependency. 
    @param connectAtStage   Stage at which Socket should be connected.
    @param isList           Whether this Socket can have multiple connectees.
    @param owner            Component to which this Socket belongs.*/
    AbstractSocket(const std::string& name,
                   const SimTK::Stage& connectAtStage,
                   bool isList,
                   const Component& owner) :
        connectAtStage(connectAtStage), _isList(isList), _owner(&owner) {
        constructProperties();
        setName(name);
    }

    // default copy assignment

    virtual ~AbstractSocket() {};
    
    /** get the system Stage when the connection should be made */
    SimTK::Stage getConnectAtStage() const {
        return connectAtStage;
    }
    
    bool isListSocket() const { return _isList; }

    //--------------------------------------------------------------------------
    /** Derived classes must satisfy this Interface */
    //--------------------------------------------------------------------------
    /** Is the Socket connected to anything? */
    virtual bool isConnected() const = 0;
    
    /** The number of connectees connected to this socket. This is either
        0 or 1 for a non-list socket. */
    unsigned getNumConnectees() const {
        auto num = getProperty_connectee_name().size();
        return static_cast<unsigned>(num);
    }

    /** Get the type of object this socket connects to*/
    virtual std::string getConnecteeTypeName() const = 0;

    /** Generic access to the connectee. Not all sockets support this method
     * (e.g., the connectee for an Input is not an Object). */
    virtual const Object& getConnecteeAsObject() const {
        OPENSIM_THROW_FRMOBJ(Exception,
                "Not supported for this type of socket.");
    }

    /** Connect this Socket to the provided connectee object. If this is a
        list socket, the connectee is appended to the list of connectees;
        otherwise, the provided connectee replaces the single connectee. */
    virtual void connect(const Object& connectee) = 0;

    /** Connect this Socket according to its connectee_name property
        given a root Component to search its subcomponents for the connect_to
        Component. */
    virtual void findAndConnect(const Component& root) {
        throw Exception("findAndConnect() not implemented; not supported "
                        "for this type of socket", __FILE__, __LINE__);
    }

    /** Set connectee name. This function can only be used if this socket is
    not a list socket.                                                     */
    void setConnecteeName(const std::string& name) {
        OPENSIM_THROW_IF(_isList,
                         Exception,
                         "An index must be provided for a list Socket.");

        setConnecteeName(name, 0);
            
    }

    /** Set connectee name of a connectee among a list of connectees. This
    function is used if this socket is a list socket.                   */
    void setConnecteeName(const std::string& name, unsigned ix) {
        OPENSIM_THROW_IF(ix >= getNumConnectees(),
                         IndexOutOfRange,
                         ix, 0, static_cast<unsigned>(getNumConnectees() - 1));
        upd_connectee_name(ix) = name;
    }

    /** Get connectee name. This function can only be used if this socket is
    not a list socket.                                                     */
    const std::string& getConnecteeName() const {
        OPENSIM_THROW_IF(_isList,
                         Exception,
                         "An index must be provided for a list Socket.");

        return getConnecteeName(0);
    }

    /** Get connectee name of a connectee among a list of connectees.         */
    const std::string& getConnecteeName(unsigned ix) const {
        OPENSIM_THROW_IF(ix >= getNumConnectees(),
                         IndexOutOfRange,
                         ix, 0, static_cast<unsigned>(getNumConnectees() - 1));
        return get_connectee_name(ix);
    }

    void appendConnecteeName(const std::string& name) {
        OPENSIM_THROW_IF((getNumConnectees() > 0 && !_isList), Exception,
            "Multiple connectee names can only be appended to a list Socket.");
        updProperty_connectee_name().appendValue(name);
    }


    /** Disconnect this Socket from its connectee. */
    virtual void disconnect() = 0;

protected:
    const Component& getOwner() const { return _owner.getRef(); }
    void setOwner(const Component& o) { _owner.reset(&o); }

private:
    void constructProperties() {
        constructProperty_connectee_name();
        if (!_isList) {
            updProperty_connectee_name().appendValue("");
            updProperty_connectee_name().setAllowableListSize(1);
        }
    }
    SimTK::Stage connectAtStage;
    bool _isList = false;

    SimTK::ReferencePtr<const Component> _owner;

    friend Component;

//=============================================================================
};  // END class AbstractSocket


template<class T>
class Socket : public AbstractSocket {
    OpenSim_DECLARE_CONCRETE_OBJECT_T(Socket, T, AbstractSocket);
public:
    
    /** Default constructor */
    Socket() {}

    // default destructor, copy constructor

    /** Convenience constructor
    Create a Socket that can only connect to Object of type T with specified 
    name and stage at which it should be connected.
    @param name             name of the socket used to describe its dependency.
    @param connectAtStage   Stage at which Socket should be connected.
    @param owner The component that contains this input. */
    Socket(const std::string& name,
           const SimTK::Stage& connectAtStage,
           const Component& owner) :
        AbstractSocket(name, connectAtStage, false, owner),
        connectee(nullptr) {}

    virtual ~Socket() {}

    /** Is the Socket connected to object of type T? */
    bool isConnected() const override {
        return !connectee.empty();
    }

    const T& getConnecteeAsObject() const override {
        return connectee.getRef();
    }

    /** Temporary access to the connectee for testing purposes. Real usage
        will be through the Socket (and Input) interfaces. 
        For example, Input should short circuit to its Output's getValue()
        once it is connected.
    Return a const reference to the object connected to this Socket */
    const T& getConnectee() const {
        if (!isConnected()) {
            std::string msg = getOwner().getConcreteClassName() + "::Socket '"
                + getName() + "' is not connected to '" + getConnecteeName()
                + "' of type " + T::getClassName();
            OPENSIM_THROW(Exception, msg);
        }
        return connectee.getRef();
    }

    /** Connect this Socket to the provided connectee object */
    void connect(const Object& object) override {
        const T* objT = dynamic_cast<const T*>(&object);
        if (objT) {
            connectee = *objT;

            std::string objPathName = objT->getAbsolutePathName();
            std::string ownerPathName = getOwner().getAbsolutePathName();

            // check if the absolute pathname is just /name
            if (objPathName.compare("/" + objT->getName()) == 0) { //exact match
                // in which case we likely are connecting to an orphan
                // (yet to adopted component) which the API permits when passing
                // in the dependency directly.
                // better off stripping off the / to identify it as a "floating"
                // Component and we will need to find its absolute path next 
                // time we try to connect
                setConnecteeName(objT->getName());
            }
            // This can happen when top level components like a Joint and Body
            // have the same name like a pelvis Body and pelvis Joint that
            // connects that connects to a Body of the same name.
            else if(objPathName == ownerPathName)
                setConnecteeName(objPathName);
            else { // otherwise store the relative path name to the object
                std::string relPathName = objT->getRelativePathName(getOwner());
                setConnecteeName(relPathName);
            }
        }
        else {
            std::stringstream msg;
            msg << "Socket::connect(): ERR- Cannot connect '"
                << object.getName()
                << "' of type " << object.getConcreteClassName()
                << ". Socket requires "
                << getConnecteeTypeName() << ".";
            throw Exception(msg.str(), __FILE__, __LINE__);
        }
    }

    /** Connect this Socket given its connectee_name property  */
    void findAndConnect(const Component& root) override;

    void disconnect() override {
        connectee.reset(nullptr);
    }
    
    /** Derived classes must satisfy this Interface */
    /** get the type of object this socket connects to*/
    std::string getConnecteeTypeName() const override {
        return T::getClassName();
    }

    SimTK_DOWNCAST(Socket, AbstractSocket);

private:
    mutable SimTK::ReferencePtr<const T> connectee;
}; // END class Socket<T>
            

/** A specialized Socket that connects to an Output signal is an Input.
    An AbstractInput enables maintenance of a list of unconnected Inputs. 
    An Input can either be a single-value Input or a list Input. A list Input
    can connect to multiple (Output) Channels.
*/
class OSIMCOMMON_API AbstractInput : public AbstractSocket {
    OpenSim_DECLARE_ABSTRACT_OBJECT(AbstractInput, AbstractSocket);
public:
    /** Default constructor */
    AbstractInput() : AbstractSocket() {}
    /** Convenience constructor
    Create an AbstractInput (Socket) that connects only to an AbstractOutput
    specified by name and stage at which it should be connected.
    @param name             name of the dependent (Abstract)Output.
    @param connectAtStage   Stage at which Input should be connected.
    @param isList           Whether this Input can have multiple connectees.
    @param owner The component that contains this input. */
    AbstractInput(const std::string& name,
                  const SimTK::Stage& connectAtStage,
                  bool isList, const Component& owner) :
        AbstractSocket(name, connectAtStage, isList, owner) {}

    virtual ~AbstractInput() {}

    // Socket interface
    void connect(const Object& object) override {
        std::stringstream msg;
        msg << "Input::connect(): ERR- Cannot connect '" << object.getName()
            << "' of type " << object.getConcreteClassName() <<
            ". Input can only connect to an Output.";
        throw Exception(msg.str(), __FILE__, __LINE__);
    }

    /** Connect this Input to a single-valued (single-channel) Output or, if
    this is a list %Input and the %Output is a list %Output, connect to all the
    channels of the %Output. You can optionally provide an alias that will be
    used by the Component owning this %Input to refer to the %Output. If this
    method connects to multiple channels, the alias will be used for all
    channels. */
    virtual void connect(const AbstractOutput& output,
                         const std::string& alias = "") = 0;

    /** Connect this Input to a single-valued (single-channel) Output. This
    method can be used with both single-valued and list %Inputs. You can
    optionally provide an alias that will be used by the Component owning this
    %Input to refer to the %Output. */
    virtual void connect(const AbstractChannel& channel,
                         const std::string& alias = "") = 0;

    /** Get the alias for a Channel. An alias is a description for a %Channel
    that is specific to how the Input will use the %Channel. For example, the
    Component that owns this %Input might expect the aliases to be the names of
    markers in the model. This method can be used only for non-list %Inputs; for
    list %Inputs, use the single-argument overload. */
    virtual const std::string& getAlias() const = 0;

    /** Get the alias for the Channel indicated by the provided index. An alias
    is a description for a %Channel that is specific to how the Input will use
    the %Channel. For example, the Component that owns this %Input might expect
    the aliases to be the names of markers in the model. */
    virtual const std::string& getAlias(unsigned index) const = 0;
    // TODO what's the best way to serialize aliases?

    /** Set the alias for a Channel. If this is a list Input, the aliases of all
    %Channels will be set to the provided string. If you wish to set the alias
    of only one %Channel, use the two-argument overload. */
    virtual void setAlias(const std::string& alias) = 0;

    /** Set the alias for the Channel indicated by the provided index. */
    virtual void setAlias(unsigned index, const std::string& alias) = 0;

    /** Get the label for this Channel. If an alias has been set, the label is
    the alias; otherwise, the label is the full path of the Output that has been
    connected to this Input. This method can be used only for non-list %Inputs;
    for list %Inputs, use the single-argument overload. */
    virtual std::string getLabel() const = 0;

    /** Get the label for the Channel indicated by the provided index. If an
    alias has been set, the label is the alias; otherwise, the label is the full
    path of the %Channel that has been connected to this Input. */
    virtual std::string getLabel(unsigned index) const = 0;

    /** Break up a connectee name into its output path, channel name
     (empty for single-value outputs), and alias. This function writes
     to the passed-in outputPath, channelName, and alias.

     Examples:
     @verbatim
     /foo/bar/output
     outputPath is "/foo/bar/output"
     channelName is ""
     alias is ""

     /foo/bar/output:channel
     outputPath is "/foo/bar/output"
     channelName is "channel"
     alias is ""

     /foo/bar/output(baz)
     outputPath is "/foo/bar/output"
     channelName is ""
     alias is "baz"

     /foo/bar/output:channel(baz)
     outputPath is "/foo/bar/output"
     channelName is "channel"
     alias is "baz"
     @endverbatim
     */
    static bool parseConnecteeName(const std::string& connecteeName,
                                   std::string& outputPath,
                                   std::string& channelName,
                                   std::string& alias) {
        auto lastSlash = connecteeName.rfind("/");
        auto colon = connecteeName.rfind(":");
        auto leftParen = connecteeName.rfind("(");
        auto rightParen = connecteeName.rfind(")");
        
        std::string outputName = connecteeName.substr(lastSlash + 1,
                                    std::min(colon, leftParen) - lastSlash);
        outputPath = connecteeName.substr(0, std::min(colon, leftParen));
        
        // Channel name.
        if (colon != std::string::npos) {
            channelName = connecteeName.substr(colon + 1,
                                               leftParen - (colon + 1));
        } else {
            channelName = "";
        }
        
        // Alias.
        if (leftParen != std::string::npos && rightParen != std::string::npos) {
            alias = connecteeName.substr(leftParen + 1,
                                         rightParen - (leftParen + 1));
        } else {
            alias = "";
        }

        return true;
    }
    
//=============================================================================
};  // END class AbstractInput


/** An Input<Y> must be connected by an Output<Y> */
template<class T>
class  Input : public AbstractInput {
    OpenSim_DECLARE_CONCRETE_OBJECT(Input, AbstractInput);
public:

    typedef typename Output<T>::Channel Channel;

    typedef std::vector<SimTK::ReferencePtr<const Channel>> ChannelList;
    typedef std::vector<std::string> AliasList;
    
    /** Default constructor */
    Input() : AbstractInput() {}
    /** Convenience constructor
    Create an Input<T> (Socket) that can only connect to an Output<T>
    name and stage at which it should be connected.
    @param name             name of the Output dependency.
    @param connectAtStage   Stage at which Input should be connected.
    @param isList           Whether this Input can have multiple connectees.
    @param owner The component that contains this input. */
    Input(const std::string& name, const SimTK::Stage& connectAtStage,
          bool isList, const Component& owner) :
        AbstractInput(name, connectAtStage, isList, owner) {}

    /** Connect this Input to the provided (Abstract)Output.
     */
    // Definition is in Component.h
    void connect(const AbstractOutput& output,
                 const std::string& alias = "") override;
    
    void connect(const AbstractChannel& channel,
                 const std::string& alias = "") override;

    /** Connect this Input given a root Component to search for
    the Output according to the connectee_name of this Input  */
    void findAndConnect(const Component& root) override;
    
    void disconnect() override {
        _connectees.clear();
        _aliases.clear();
    }
    
    bool isConnected() const override {
        return !_connectees.empty();
    }
    
    /** Get the value of this Input when it is connected. Redirects to connected
    Output<T>'s getValue() with minimal overhead. This method can be used only
    for non-list Input(s). For list Input(s), use the other overload.         */
    const T& getValue(const SimTK::State &state) const {
        OPENSIM_THROW_IF(isListSocket(),
                         Exception,
                         "Input<T>::getValue(): an index must be "
                         "provided for a list input.");

        return getValue(state, 0);
    }

    /**Get the value of this Input when it is connected. Redirects to connected
    Output<T>'s getValue() with minimal overhead. Specify the index of the 
    Channel whose value is desired.                                           */
    const T& getValue(const SimTK::State &state, unsigned index) const {
        // TODO remove this check in order to improve speed?
        OPENSIM_THROW_IF(index >= getNumConnectees(),
                         IndexOutOfRange,
                         index, 0, 
                         static_cast<unsigned>(getNumConnectees() - 1));

        return _connectees[index].getRef().getValue(state);
    }

    /** Get the Channel associated with this Input. This method can only be
    used for non-list Input(s). For list Input(s), use the other overload.    */
    const Channel& getChannel() const {
        OPENSIM_THROW_IF(isListSocket(),
                         Exception,
                         "Input<T>::getChannel(): an index must be "
                         "provided for a list input.");

        return getChannel(0);
    }

    /** Get the Channel associated with this Input. Specify the index of the
    channel desired.                                                          */
    const Channel& getChannel(unsigned index) const {
        OPENSIM_THROW_IF(index >= getNumConnectees(),
                         IndexOutOfRange,
                         index, 0, getNumConnectees() - 1u);

        return _connectees[index].getRef();
    }
    
    const std::string& getAlias() const override {
        OPENSIM_THROW_IF_FRMOBJ(!isConnected(),
                                InputNotConnected, getName());
        OPENSIM_THROW_IF(isListSocket(),
                         Exception,
                         "Input<T>::getAlias(): this is a list Input; an index "
                         "must be provided.");

        return getAlias(0);
    }
    
    const std::string& getAlias(unsigned index) const override {
        OPENSIM_THROW_IF_FRMOBJ(!isConnected(),
                                InputNotConnected, getName());
        OPENSIM_THROW_IF(index >= getNumConnectees(),
                         IndexOutOfRange,
                         index, 0, getNumConnectees() - 1u);

        return _aliases[index];
    }

    void setAlias(const std::string& alias) override {
        OPENSIM_THROW_IF_FRMOBJ(!isConnected(),
                                InputNotConnected, getName());

        for (unsigned i=0; i<getNumConnectees(); ++i)
            setAlias(i, alias);
    }

    void setAlias(unsigned index, const std::string& alias) override {
        OPENSIM_THROW_IF_FRMOBJ(!isConnected(),
                                InputNotConnected, getName());
        OPENSIM_THROW_IF(index >= getNumConnectees(),
                         IndexOutOfRange,
                         index, 0, getNumConnectees() - 1u);

        _aliases[index] = alias;
    }

    std::string getLabel() const override {
        OPENSIM_THROW_IF_FRMOBJ(!isConnected(),
                                InputNotConnected, getName());
        OPENSIM_THROW_IF(isListSocket(),
                         Exception,
                         "Input<T>::getLabel(): this is a list Input; an index "
                         "must be provided.");

        return getLabel(0);
    }

    std::string getLabel(unsigned index) const override {
        OPENSIM_THROW_IF_FRMOBJ(!isConnected(),
                                InputNotConnected, getName());
        OPENSIM_THROW_IF(index >= getNumConnectees(),
                         IndexOutOfRange,
                         index, 0, getNumConnectees() - 1u);

        const std::string alias = getAlias(index);
        if (!alias.empty())
            return alias;

        return getChannel(index).getPathName();
    }

    /** Access the values of all the channels connected to this Input as a 
    SimTK::Vector_<T>. The elements are in the same order as the channels.
    */
    SimTK::Vector_<T> getVector(const SimTK::State& state) const {
        SimTK::Vector_<T> v(_connectees.size());
        for (int ichan = 0; ichan < _connectees.size(); ++ichan) {
            v[ichan] = _connectees[ichan]->getValue(state);
        }
        return v;
    }
    
    /** Get const access to the channels connected to this input.
        You can use this to iterate through the channels.
        @code{.cpp}
        for (const auto& chan : getChannels()) {
            std::cout << chan.getValue(state) << std::endl;
        }
        @endcode
    */
    const ChannelList& getChannels() const {
        return _connectees;
    }
    
    /** Return the typename of the Output value, T, that satisfies
        this Input<T>. No reason to return Output<T> since it is a
        given that only an Output can satisfy an Input. */
    std::string getConnecteeTypeName() const override {
        return SimTK::NiceTypeName<T>::namestr();
    }

    SimTK_DOWNCAST(Input, AbstractInput);

private:
    SimTK::ResetOnCopy<ChannelList> _connectees;
    // TODO I think the aliases need to be serialized, since
    // tools may depend on them for interpreting the connected channels.
    SimTK::ResetOnCopy<AliasList> _aliases;
}; // END class Input<Y>
        
/// @name Creating Sockets to other objects for your Component
/// Use these macros at the top of your component class declaration,
/// near where you declare @ref Property properties.
/// @{
/** Create a socket for this component's dependence on another component.
 * You must specify the type of the component that can be connected to this
 * socket. The comment should describe how the connected component
 * (connectee) is used by this component.
 *
 * Here's an example for using this macro:
 * @code{.cpp}
 * #include <OpenSim/Simulation/Model/PhysicalOffsetFrame.h>
 * class MyComponent : public Component {
 * public:
 *     OpenSim_DECLARE_SOCKET(parent, PhysicalOffsetFrame,
 *             "To locate this component.");
 *     ...
 * };
 * @endcode
 *
 * @note This macro requires that you have included the header that defines
 * type `T`, as shown in the example above. We currently do not support
 * declaring sockets if `T` is only forward-declared.
 *
 * @note If you use this macro in your class, then you should *NOT* implement
 * a custom copy constructor---try to use the default one. The Socket will
 * not get copied properly if you create a custom copy constructor.
 *
 * @see Component::constructSocket()
 * @relates OpenSim::Socket
 */
#define OpenSim_DECLARE_SOCKET(cname, T, comment)                           \
    /** @name Sockets                                                    */ \
    /** @{                                                               */ \
    /** comment                                                          */ \
    /** This socket was generated with the                               */ \
    /** #OpenSim_DECLARE_SOCKET macro.                                   */ \
    OpenSim_DOXYGEN_Q_PROPERTY(T, cname)                                    \
    /** @}                                                               */ \
    /** @cond                                                            */ \
    int _socket_##cname {                                                   \
        this->template constructSocket<T>(#cname)                           \
    };                                                                      \
    /** @endcond                                                         */

// The following doxygen-like description does NOT actually appear in doxygen.
/* Preferably, use the #OpenSim_DECLARE_SOCKET macro. Only use this macro
 * when are you unable to include the header that defines type `T`. This might
 * be the case if you have a circular dependency between your class and `T`.
 * In such cases, you must:
 *
 *  -# forward-declare type `T`
 *  -# call this macro inside the definition of your class, and
 *  -# call #OpenSim_DEFINE_SOCKET_FD in your class's .cpp file (notice the
 *      difference: DEFINE vs DECLARE).
 *
 * MyComponent.h:
 * @code{.cpp}
 * namespace OpenSim {
 * class PhysicalOffsetFrame;
 * class MyComponent : public Component {
 * OpenSim_DECLARE_CONCRETE_OBJECT(MyComponent, Component);
 * public:
 *     OpenSim_DECLARE_SOCKET_FD(parent, PhysicalOffsetFrame,
 *             "To locate this component.");
 *     ...
 * };
 * }
 * @endcode
 *
 * MyComponent.cpp:
 * @code{.cpp}
 * #include "MyComponent.h"
 * OpenSim_DEFINE_SOCKET_FD(parent, OpenSim::MyComponent);
 * ...
 * @endcode
 *
 * You can also look at the OpenSim::Geometry source code for an example.
 *
 * @note Do NOT forget to call OpenSim_DEFINE_SOCKET_FD in your .cpp file!
 *
 * The "FD" in the name of this macro stands for "forward-declared."
 *
 * @warning This macro is experimental and may be removed in future versions.
 *
 * @see Component::constructSocket()
 * @relates OpenSim::Socket
 */
#define OpenSim_DECLARE_SOCKET_FD(cname, T, comment)                        \
    /** @name Sockets                                                    */ \
    /** @{                                                               */ \
    /** comment                                                          */ \
    /** This is an %OpenSim Socket.                                      */ \
    OpenSim_DOXYGEN_Q_PROPERTY(T, cname)                                    \
    /** @}                                                               */ \
    /** @cond                                                            */ \
    int _socket_##cname {                                                   \
        constructSocket_##cname()                                           \
    };                                                                      \
    /* Declare the method used in the in-class member initializer.       */ \
    /* This method will be defined by OpenSim_DEFINE_SOCKET_FD.          */ \
    int constructSocket_##cname();                                          \
    /* Remember the provided type so we can use it in the DEFINE macro.  */ \
    typedef T _socket_##cname##_type;                                       \
    /** @endcond                                                         */

// The following doxygen-like description does NOT actually appear in doxygen.
/* When specifying a Socket to a forward-declared type (using
 * OpenSim_DECLARE_SOCKET_FD in the class definition), you must call this
 * macro in your .cpp file.  The arguments are the name of the socket (the
 * same one provided to OpenSim_DECLARE_SOCKET_FD) and the class in which
 * the socket exists. See #OpenSim_DECLARE_SOCKET_FD for an example.
 *
 * @warning This macro is experimental and may be removed in future versions.
 *
 * @see #OpenSim_DECLARE_SOCKET_FD
 * @relates OpenSim::Socket
 */
// This macro defines the method that the in-class member initializer calls
// to construct the Socket. The reason why this must be in the .cpp file is
// that putting the template member function `template <typename T>
// Component::constructSocket` in the header requires that `T` is not an
// incomplete type (specifically, when compiling cpp files for classes OTHER
// than `MyComponent` but that include MyComponent.h). OpenSim::Geometry is an
// example of this scenario.
#define OpenSim_DEFINE_SOCKET_FD(cname, Class)                           \
int Class::constructSocket_##cname() {                                   \
    using T = _socket_##cname##_type;                                    \
    return this->template constructSocket<T>(#cname);                    \
}
/// @}

/// @name Creating Inputs for your Component
/// Use these macros at the top of your component class declaration,
/// near where you declare @ref Property properties.
/// @{
/** Create a socket for this component's dependence on an output signal from
 *  another component. It is a placeholder for an Output that can be connected
 *  to it. An output must have the same type T as an input to be connected
 *  to it. You must also specify the stage at which you require this input
 *  quantity. The comment should describe how the input quantity is used.
 *
 *  An Input declared with this macro can connect to only one Output.
 *
 *  Here's an example for using this macro:
 *  @code{.cpp}
 *  class MyComponent : public Component {
 *  public:
 *      OpenSim_DECLARE_INPUT(emg, double, SimTK::Stage::Velocity, "For 
 *  validation.");
 *      ...
 *  };
 *  @endcode
 * @see Component::constructInput()
 * @relates OpenSim::Input
 */
#define OpenSim_DECLARE_INPUT(iname, T, istage, comment)                    \
    /** @name Inputs                                                     */ \
    /** @{                                                               */ \
    /** comment                                                          */ \
    /** This input is needed at stage istage.                            */ \
    /** This input was generated with the                                */ \
    /** #OpenSim_DECLARE_INPUT macro.                                    */ \
    OpenSim_DOXYGEN_Q_PROPERTY(T, iname)                                    \
    /** @}                                                               */ \
    /** @cond                                                            */ \
    bool _has_input_##iname { constructInput<T>(#iname, istage) };          \
    /** @endcond                                                         */
    
/** Create a list input, which can connect to more than one Channel. This
 * makes sense for components like reporters that can handle a flexible
 * number of input values. 
 *
 * @see Component::constructInput()
 * @relates OpenSim::Input
 */
#define OpenSim_DECLARE_LIST_INPUT(iname, T, istage, comment)               \
    /** @name Inputs (list)                                              */ \
    /** @{                                                               */ \
    /** comment                                                          */ \
    /** This input can connect to multiple outputs, all of which are     */ \
    /** needed at stage istage.                                          */ \
    /** This input was generated with the                                */ \
    /** #OpenSim_DECLARE_LIST_INPUT macro.                               */ \
    OpenSim_DOXYGEN_Q_PROPERTY(T, iname)                                    \
    /** @}                                                               */ \
    /** @cond                                                            */ \
    bool _has_input_##iname { constructInput<T>(#iname, istage, true) };    \
    /** @endcond                                                         */
/// @}

} // end of namespace OpenSim

#endif  // OPENSIM_COMPONENT_SOCKET_H_
