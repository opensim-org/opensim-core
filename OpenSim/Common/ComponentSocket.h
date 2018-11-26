#ifndef OPENSIM_COMPONENT_SOCKET_H_
#define OPENSIM_COMPONENT_SOCKET_H_
/* -------------------------------------------------------------------------- *
 *                       OpenSim:  ComponentSocket.h                          *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2017 Stanford University and the Authors                *
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

// INCLUDES
#include "osimCommonDLL.h"

#include "ComponentPath.h"
#include "ComponentOutput.h"
#include "ComponentList.h"
#include "Object.h"
#include "Exception.h"
#include "Property.h"

namespace OpenSim {

//==============================================================================
/// ComponentSocket Exceptions
//==============================================================================
class InputNotConnected : public Exception {
public:
    InputNotConnected(const std::string& file,
                      size_t line,
                      const std::string& func,
                      const std::string& inputName) :
        Exception(file, line, func) {
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
 * (typically another Component) without owning that object. While Components
 * can be composites (of multiple components) they often depend on unrelated
 * objects/components that are defined and owned elsewhere. The object that
 * satisfies the requirements of the Socket we term the "connectee". When a
 * Socket is satisfied by a connectee we have a successful "connection" or is
 * said to be connected.
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
 * Programmatically, the connectee can be specified as an object reference
 * or via a connectee path:
 *
 * @code{.cpp}
 * // Specifying the connectee using an object reference.
 * socket.connect(myConnectee);
 * // Specifying the connectee via a path.
 * socket.setConnecteePath("/path/to/connectee");
 * @endcode
 *
 * Use finalizeConnection() to synchronize the object reference and connectee
 * name. It is preferable to use connect() instead of setConnecteePath().
 * If *both* are set, then the object reference overrides the connectee path.
 *
 * The connectee path appears in XML files and is how a connection is maintained
 * across serialization (writing to an XML file) and deserialization (reading
 * from an XML file).
 *
 * @author  Ajay Seth
 */
class OSIMCOMMON_API AbstractSocket {
public:

    // default copy constructor, copy assignment

    virtual ~AbstractSocket() {};
    
    /// Create a dynamically-allocated copy. You must manage the memory
    /// for the returned pointer.
    /// This function exists to facilitate the use of
    /// SimTK::ClonePtr<AbstractSocket>.
    virtual AbstractSocket* clone() const = 0;
    
    const std::string& getName() const { return _name; }
    /** Get the system Stage when the connection should be made. */
    SimTK::Stage getConnectAtStage() const { return _connectAtStage; }
    /** Can this Socket have more than one connectee? */
    bool isListSocket() const { return _isList; }
    /** The number of slots to fill in order to satisfy this socket.
     * This is 1 for a non-list socket. This is the number of elements in the
     * connectee path property; to sync this with the number of connectee
     * objects, call finalizeConnection(). */
    unsigned getNumConnectees() const {
        return static_cast<unsigned>(getConnecteePathProp().size());
    }

    //--------------------------------------------------------------------------
    /** Derived classes must satisfy this Interface */
    //--------------------------------------------------------------------------
    /** Is the Socket connected to its connectee(s)? For a list socket,
    this is only true if this socket is connected to all its connectees.
     */
    virtual bool isConnected() const = 0;
    
    /** Get the type of object this socket connects to. */
    virtual std::string getConnecteeTypeName() const = 0;

    /** Generic access to the connectee. Not all sockets support this method
     * (e.g., the connectee for an Input is not an Object). */
    virtual const Object& getConnecteeAsObject() const {
        OPENSIM_THROW(Exception, "Not supported for this type of socket.");
    }

    /** Connect this %Socket to the provided connectee object. If this is a
        list socket, the connectee is appended to the list of connectees;
        otherwise, the provided connectee replaces the single connectee. */
    virtual void connect(const Object& connectee) = 0;

    /** Find the connectee using a search with a partial path. Use this if you
     * do not want to specify an exact path (maybe you don't quite know where
     * the connectee is located).
     * */
    virtual void findAndConnect(const ComponentPath& connectee) {
        OPENSIM_THROW(Exception, "Not implemented.");
    }
    /** Same as findAndConnect(const ComponentPath&), but using a string
     * argument. */
    void findAndConnect(const std::string& connectee) {
        findAndConnect(ComponentPath(connectee));
    }

    /** Connect this %Socket according to its connectee path property
        given a root %Component to search its subcomponents for the connect_to
        Component. */
    virtual void finalizeConnection(const Component& root) {
        throw Exception("finalizeConnection() not implemented; not supported "
                        "for this type of socket", __FILE__, __LINE__);
    }

    /** Clear references to connectees. The connectee path property is not
     * affected. Calling finalizeConnection() will use the connectee path
     * property to satisfy the socket. */
    virtual void disconnect() = 0;

    /** %Set connectee path. This function can only be used if this socket is
     * not a list socket. If a connectee reference is set (with connect()) the
     * connectee path is ignored; call disconnect() if you want the socket to be
     * connected using the connectee path.
     *
     * It is preferable to use connect() instead of this function. */
    void setConnecteePath(const std::string& name) {
        OPENSIM_THROW_IF(_isList,
                         Exception,
                         "An index must be provided for a list Socket.");
        setConnecteePath(name, 0);
    }

    /** %Set connectee path of a connectee among a list of connectees. This
     * function is used if this socket is a list socket. If a connectee
     * reference is set (with connect()) the connectee path is ignored; call
     * disconnect() if you want the socket to be connected using the connectee
     * name.
     *
     * It is preferable to use connect() instead of this function. */
    void setConnecteePath(const std::string& name, unsigned ix) {
        using SimTK::isIndexInRange;
        SimTK_INDEXCHECK_ALWAYS(ix, getNumConnectees(),
                                "AbstractSocket::setConnecteePath()");
        updConnecteePathProp().setValue(ix, name);
    }

    /** Get connectee path. This function can only be used if this socket is
    not a list socket.                                                     */
    const std::string& getConnecteePath() const {
        OPENSIM_THROW_IF(_isList,
                         Exception,
                         "An index must be provided for a list Socket.");
        return getConnecteePath(0);
    }

    /** Get connectee path of a connectee among a list of connectees.         */
    const std::string& getConnecteePath(unsigned ix) const {
        using SimTK::isIndexInRange;
        SimTK_INDEXCHECK_ALWAYS(ix, getNumConnectees(),
                                "AbstractSocket::getConnecteePath()");
        return getConnecteePathProp().getValue(ix);
    }

    void appendConnecteePath(const std::string& name) {
        OPENSIM_THROW_IF((getNumConnectees() > 0 && !_isList), Exception,
            "Multiple connectee paths can only be appended to a list Socket.");
        updConnecteePathProp().appendValue(name);
    }

    /** Clear all connectee paths in the connectee path property. */
    void clearConnecteePath() {
        if (isListSocket())
            updConnecteePathProp().clear();
        else
            updConnecteePathProp().setValue(0, "");
    }

    /** Get owner component of this socket */
    const Component& getOwner() const { return _owner.getRef(); }

protected:
    //--------------------------------------------------------------------------
    // CONSTRUCTION
    //--------------------------------------------------------------------------
    /** Create a Socket with specified name and stage at which it should be
    connected.
    @param name               name of the socket, usually describes its 
                              dependency.
    @param connecteePathIndex Index of the property in the containing Component
                              that holds this Socket's connectee path(s).
    @param connectAtStage     Stage at which Socket should be connected.
    @param owner              Component to which this Socket belongs. */
    AbstractSocket(const std::string& name,
                   const PropertyIndex& connecteePathIndex,
                   const SimTK::Stage& connectAtStage,
                   Component& owner) :
            _name(name),
            _connectAtStage(connectAtStage),
            _connecteePathIndex(connecteePathIndex),
            _owner(&owner),
            _isList(getConnecteePathProp().isListProperty()) {}

    /** %Set an internal pointer to the Component that contains this Socket.
    This should only be called by Component.
    This exists so that after the containing Component is copied, the 'owner'
    is the new Component. This Socket needs to be able to modify
    the associated connectee path property in the Component. Thus, we require
    a writable reference. */
    // We could avoid the need for this function by writing a custom copy
    // constructor for Component.
    void setOwner(Component& o) { _owner.reset(&o); }
    /** This will be false immediately after copy construction or assignment.*/
    bool hasOwner() const { return !_owner.empty(); }
    
    /** Check if entries of the connectee path property's value is valid (if
    it contains spaces, etc.); if so, print out a warning. */
    void checkConnecteePathProperty() {
        // TODO Move this check elsewhere once the connectee path
        // property is a ComponentPath (or a ChannelPath?).
        for (unsigned iname = 0u; iname < getNumConnectees(); ++iname) {
            const auto& connecteePath = getConnecteePath(iname);
            ComponentPath cp(connecteePath);
            if (!cp.isLegalPathElement(cp.getComponentName()) ) {
                std::string msg = "In Socket '" + getName() +
                        "', connectee path '" + connecteePath +
                        "' contains illegal characters such as spaces.";
                if (!_isList) {
                    msg += " Did you try to specify multiple connectee "
                           "names for a single-value Socket?";
                }
                OPENSIM_THROW(Exception, msg);
            }
            // TODO update the above for Inputs when ChannelPath exists.
            
            // TODO There might be a bug with empty connectee path being
            // interpreted as "this component."
        }
    }

protected:
    
    /// Const access to the connectee path property from the Component in which
    /// this Socket resides. The name of that property is something like
    /// 'socket_<name>'. This is a special type of property
    /// that users cannot easily access (e.g., there is no macro-generated
    /// `get_socket_<name>()` function).
    const Property<std::string>& getConnecteePathProp() const;
    /// Writable access to the connectee path property from the Component in
    /// which this Socket resides. Calling this will mark the Component as
    /// not "up to date with properties"
    /// (Object::isObjectUpToDateWithProperties()).
    Property<std::string>& updConnecteePathProp();


private:

    /// Prepend the provided name to all (if any) absolute connectee paths
    /// stored in this socket. This is to be used when adding one component
    /// to another.
    virtual void prependComponentPathToConnecteePath(
            const std::string& pathToPrepend);

    std::string _name;
    SimTK::Stage _connectAtStage = SimTK::Stage::Empty;
    PropertyIndex _connecteePathIndex;
    // Even though updConnecteePathProp() requires non-const access to this
    // pointer, we make this a const pointer to reduce the chance of mis-use.
    // If this were a non-const pointer, then const functions in this class
    // would be able to edit _owner (see declaration of ReferencePtr).
    SimTK::ReferencePtr<const Component> _owner;
    // _isList must be after _owner, as _owner is used to set its value.
    bool _isList;

    /* So that Component can invoke setOwner(), etc. */
    friend Component;

//=============================================================================
};  // END class AbstractSocket


template<class T>
class Socket : public AbstractSocket {
public:

    // default copy constructor
    
    virtual ~Socket() {}
    
    Socket<T>* clone() const override { return new Socket<T>(*this); }

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
            std::string msg = "Socket " + getName() + " of type " +
                    T::getClassName() + " in " +
                    getOwner().getAbsolutePathString() + " of type " +
                    getOwner().getConcreteClassName() + " is not connected.";
            OPENSIM_THROW(Exception, msg);
        }
        return connectee.getRef();
    }

    /** Connect this Socket to the provided connectee object */
    void connect(const Object& object) override {
        const T* objT = dynamic_cast<const T*>(&object);
        if (!objT) {
            std::stringstream msg;
            msg << "Type mismatch: Socket '" << getName() << "' of type "
                << getConnecteeTypeName() << " cannot connect to '"
                << object.getName() << "' of type "
                << object.getConcreteClassName() << ".";
            OPENSIM_THROW(Exception, msg.str());
        }

        connectInternal(*objT);
    }

    /** Find a component using a partial path or component name of the correct
     * type anywhere in the model (using Component::findComponent()) and
     * connect to that component.
     *
     * Throws an exception If you provide only a component name and the
     * model has multiple components with that nume.
     * */
    void findAndConnect(const ComponentPath& connectee) override {
        const auto* comp =
            getOwner().getRoot().template findComponent<T>(connectee);
        OPENSIM_THROW_IF(!comp, ComponentNotFound, connectee.toString(),
                getConnecteeTypeName(), getOwner().getAbsolutePathString());
        connect(*comp);
    }

    /** Connect this Socket given its connectee path property  */
    void finalizeConnection(const Component& root) override;

    void disconnect() override {
        connectee.reset(nullptr);
    }
    
    /** Derived classes must satisfy this Interface */
    /** get the type of object this socket connects to*/
    std::string getConnecteeTypeName() const override {
        return T::getClassName();
    }

    SimTK_DOWNCAST(Socket, AbstractSocket);
    
    /** For use in python/java/MATLAB bindings. */
    // This method exists for consistency with Object's safeDownCast.
    static Socket<T>* safeDownCast(AbstractSocket* base) {
        return dynamic_cast<Socket<T>*>(base);
    }
    
protected:
    /** Create a Socket that can only connect to Object of type T with 
    specified name and stage at which it should be connected. Only Component
    can ever construct this class.
    @param name               name of the socket used to describe its dependency.
    @param connecteePathIndex Index of the property in the containing Component
                              that holds this Socket's connectee path(s).
    @param connectAtStage     Stage at which Socket should be connected.
    @param owner              The component that contains this input. */
    Socket(const std::string& name,
           const PropertyIndex& connecteePathIndex,
           const SimTK::Stage& connectAtStage,
           Component& owner) :
        AbstractSocket(name, connecteePathIndex, connectAtStage, owner),
        connectee(nullptr) {}
        
    /** So that Component can construct a Socket. */
    friend Component;

private:

    void connectInternal(const T& objT) {
        connectee = &objT;
    }

    mutable SimTK::ReferencePtr<const T> connectee;
}; // END class Socket<T>
            

/** A specialized Socket that connects to an Output signal is an Input.
An AbstractInput enables maintenance of a list of unconnected Inputs. 
An Input can either be a single-value Input or a list Input. A list Input
can connect to multiple (Output) Channels.

#### XML Syntax of a connectee path

For every %Input that a component has, the XML representation of the component
contains an element named `input_<input_name>`. For example, a component
that has an Input named `desired_angle` might look like the following in XML:
@code
    <MyComponent name="my_comp">
        <input_desired_angle>
            ../foo/angle
        </input_desired_angle>
        ...
    </MyComponent>
@endcode
You use this field to specify the outputs/channels that should be connected to
this input (that is, the connectees). The syntax for the connectee path
property is as follows:
@code
<path/to/component>|<output_name>[:<channel_name>][(<alias>)]
@endcode
Angle brackets indicate fields that one would fill in, and square brackets
indicate optional fields. The `<path/to/component>` can be relative or
absolute, and describes the location of the Component that contains the 
desired Output relative to the location of the Component that contains this
Input. The `<path/to/component>` and `<output_name>` must always be specified.
The `<channel_name>` should only be specified if the %Output is a list output
(i.e., it has multiple channels). The `<alias>` is a name for the
output/channel that is specific to this input, and it is optional.
All fields should contain only letters, numbers, and underscores (the path
to the component can contain slashes and periods); fields must *not* contain
spaces.
Here are some examples:
 - `../marker_data|column:left_ankle`: The TableSourceVec3 component
   `../marker_data` has a list output `column`, and we want to connect to its
   `left_ankle` channel.
 - `../averager|output(knee_joint_center)`: The component `../averager`
   (presumably a component that averages its inputs) has an output named
   `output`, and we are aliasing this output as `knee_joint_center`.
 - `/leg_model/soleus|activation`: This connectee path uses the absolute path
   to component `soleus`, which has an output named `activation`.

List inputs can contain multiple entries in its connectee path, with the
entries separated by a space. For example:
@code
<input_experimental_markers>
    ../marker_data|column:left_ankle ../marker_data|column:right_ankle ../averager|output(knee_joint_center)
</input_experimental_markers>
@endcode
*/
class OSIMCOMMON_API AbstractInput : public AbstractSocket {
public:

    virtual ~AbstractInput() {}
    
    // Change the return type of clone(). This is similar to what the Object
    // macros do (see OpenSim_OBJECT_ABSTRACT_DEFS).
    AbstractInput* clone() const override = 0;

    // Socket interface
    void connect(const Object& object) override {
        std::stringstream msg;
        msg << "Input::connect(): ERR- Cannot connect '" << object.getName()
            << "' of type " << object.getConcreteClassName() <<
            ". Input can only connect to an Output.";
        throw Exception(msg.str(), __FILE__, __LINE__);
    }

    /** TODO */
//    virtual bool isConnecteeSpecified() const = 0;

    /** Connect this Input to a single-valued (single-channel) Output or, if
    this is a list %Input and the %Output is a list %Output, connect to all the
    channels of the %Output. You can optionally provide an alias that will be
    used by the Component owning this %Input to refer to the %Output. If this
    method connects to multiple channels, the alias will be used for all
    channels. */
    virtual void connect(const AbstractOutput& output,
                         const std::string& alias = "") = 0;
    /** Connect this Input to a single output channel. This
    method can be used with both single-valued and list %Inputs. You can
    optionally provide an alias that will be used by the Component owning this
    %Input to refer to the %Channel. */
    virtual void connect(const AbstractChannel& channel,
                         const std::string& alias = "") = 0;

    /** Get the alias for a Channel. An alias is a description for a %Channel
    that is specific to how the Input will use the %Channel. For example, the
    Component that owns this %Input might expect the aliases to be the names of
    markers in the model. This method can be used only for non-list %Inputs; for
    list %Inputs, use the overload that takes an index.
    You must finalize connections (Component::finalizeConnections()) first. */
    virtual const std::string& getAlias() const = 0;

    /** Get the alias for the Channel indicated by the provided index. An alias
    is a description for a %Channel that is specific to how the Input will use
    the %Channel. For example, the Component that owns this %Input might expect
    the aliases to be the names of markers in the model.
    You must finalize connections (Component::finalizeConnections()) first. */
    virtual const std::string& getAlias(unsigned index) const = 0;

    /** %Set the alias for a Channel. If this is a list Input, the aliases of all
    %Channels will be set to the provided string. If you wish to set the alias
    of only one %Channel, use the two-argument overload.
    You must finalize connections (Component::finalizeConnections()) first. */
    virtual void setAlias(const std::string& alias) = 0;

    /** %Set the alias for the Channel indicated by the provided index.
    You must finalize connections (Component::finalizeConnections()) first. */
    virtual void setAlias(unsigned index, const std::string& alias) = 0;

    /** Get the label for this Channel. If an alias has been set, the label is
    the alias; otherwise, the label is the full path of the Output that has been
    connected to this Input. This method can be used only for non-list %Inputs;
    for list %Inputs, use the single-argument overload.
    You must finalize connections (Component::finalizeConnections()) first. */
    virtual std::string getLabel() const = 0;

    /** Get the label for the Channel indicated by the provided index. If an
    alias has been set, the label is the alias; otherwise, the label is the full
    path of the %Channel that has been connected to this Input.
    You must finalize connections (Component::finalizeConnections()) first. */
    virtual std::string getLabel(unsigned index) const = 0;

    /** Break up a connectee path into its output path, channel name
     (empty for single-value outputs), and alias. This function writes
     to the passed-in outputPath, channelName, and alias.

     Examples:
     @verbatim
     /foo/bar|output
     outputPath is "/foo/bar/output"
     channelName is ""
     alias is ""

     /foo/bar|output:channel
     outputPath is "/foo/bar/output"
     channelName is "channel"
     alias is ""

     /foo/bar|output(baz)
     outputPath is "/foo/bar/output"
     channelName is ""
     alias is "baz"

     /foo/bar|output:channel(baz)
     outputPath is "/foo/bar|output"
     channelName is "channel"
     alias is "baz"
     @endverbatim
     */
    static bool parseConnecteePath(const std::string& connecteePath,
                                   std::string& componentPath,
                                   std::string& outputName,
                                   std::string& channelName,
                                   std::string& alias) {
        auto bar = connecteePath.rfind("|");
        auto colon = connecteePath.rfind(":");
        auto leftParen = connecteePath.rfind("(");
        auto rightParen = connecteePath.rfind(")");
        
        componentPath = connecteePath.substr(0, bar);
        outputName = connecteePath.substr(bar + 1,
                                          std::min(colon, leftParen) - (bar + 1));
        
        // Channel name.
        if (colon != std::string::npos) {
            channelName = connecteePath.substr(colon + 1, leftParen - (colon + 1));
        } else {
            channelName = "";
        }
        
        // Alias.
        if (leftParen != std::string::npos && rightParen != std::string::npos) {
            alias = connecteePath.substr(leftParen + 1,
                                         rightParen - (leftParen + 1));
        } else {
            alias = "";
        }

        return true;
    }

    /** Compose the connectee path from its constituents. This is the opposite
    operation of parseConnecteePath().
    Example:
    @verbatim
     if inputs are
       componentPath --> "/foo/bar"
       outputName    --> "output"
       channelName   --> "channel"
       alias         --> "baz"
     then result --> /foo/bar|output:channel(baz)
    @endverbatim
    */
    static std::string composeConnecteePath(const std::string& componentPath,
                                            const std::string& outputName,
                                            const std::string& channelName,
                                            const std::string& alias) {
        auto path = componentPath;
        path += "|";
        path += outputName;
        if(!channelName.empty())
            path += ":" + channelName;
        if(!alias.empty())
            path += "(" + alias + ")";

        return path;
    }
    
protected:
    /** Create an AbstractInput (Socket) that connects only to an 
    AbstractOutput specified by name and stage at which it should be connected.
    Only Component should ever construct this class.
    @param name              name of the dependent (Abstract)Output.
    @param connecteePathIndex Index of the property in the containing Component
                              that holds this Input's connectee path(s).
    @param connectAtStage     Stage at which Input should be connected.
    @param owner              The component that contains this input. */
    AbstractInput(const std::string& name,
                  const PropertyIndex& connecteePathIndex,
                  const SimTK::Stage& connectAtStage,
                  Component& owner) :
        AbstractSocket(name, connecteePathIndex, connectAtStage, owner) {}

private:
    void prependComponentPathToConnecteePath(
            const std::string& pathToPrepend) override;

    /* So that Component can invoke prependComponentPathToConnecteePath(). */
    friend Component;

//=============================================================================
};  // END class AbstractInput


/** An Input<Y> must be connected by an Output<Y> */
template<class T>
class Input : public AbstractInput {
public:

    typedef typename Output<T>::Channel Channel;

    typedef std::vector<SimTK::ReferencePtr<const Channel>> ChannelList;
    typedef std::vector<std::string> AliasList;
    
    Input<T>* clone() const override { return new Input<T>(*this); }

    /** Connect this Input to the provided (Abstract)Output. */
    // Definition is in Component.h
    void connect(const AbstractOutput& output,
                 const std::string& alias = "") override;
    
    void connect(const AbstractChannel& channel,
                 const std::string& alias = "") override;

    /** Connect this Input given a root Component to search for
    the Output according to the connectee path of this Input  */
    void finalizeConnection(const Component& root) override;
    
    void disconnect() override {
        _registeredChannels.clear();
        _connectees.clear();
        _aliases.clear();
    }
    
    bool isConnected() const override {
        return _connectees.size() == getNumConnectees();
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
        OPENSIM_THROW_IF(!isConnected(), InputNotConnected, getName());
        using SimTK::isIndexInRange;
        SimTK_INDEXCHECK(index, getNumConnectees(),
                         "Input<T>::getValue()");

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
        OPENSIM_THROW_IF(!isConnected(), InputNotConnected, getName());
        using SimTK::isIndexInRange;
        SimTK_INDEXCHECK_ALWAYS(index, getNumConnectees(),
                                "Input<T>::getChannel()");
        SimTK_ASSERT(index < _connectees.size(), "Internal error: "
            "getNumConnectees() and _connectees.size() are not consistent.");
        return _connectees[index].getRef();
    }
    
    const std::string& getAlias() const override {
        OPENSIM_THROW_IF(isListSocket(),
                         Exception,
                         "Input<T>::getAlias(): this is a list Input; an index "
                         "must be provided.");

        return getAlias(0);
    }
    
    const std::string& getAlias(unsigned index) const override {
        OPENSIM_THROW_IF(!isConnected(), InputNotConnected, getName());
        using SimTK::isIndexInRange;
        SimTK_INDEXCHECK_ALWAYS(index, getNumConnectees(),
                                "Input<T>::getAlias()");
        SimTK_ASSERT(index < _aliases.size(), "Internal error: "
            "getNumConnectees() and _aliases.size() are not consistent.");
        return _aliases[index];
    }

    void setAlias(const std::string& alias) override {
        for (unsigned i=0; i<getNumConnectees(); ++i)
            setAlias(i, alias);
    }

    void setAlias(unsigned index, const std::string& alias) override {
        OPENSIM_THROW_IF(!isConnected(), InputNotConnected, getName());
        using SimTK::isIndexInRange;
        SimTK_INDEXCHECK_ALWAYS(index, getNumConnectees(),
                                "Input<T>::setAlias()");

        const auto& connecteePath = getConnecteePath(index);
        std::string componentPath{};
        std::string outputName{};
        std::string channelName{};
        std::string currAlias{};
        parseConnecteePath(connecteePath,
                           componentPath,
                           outputName,
                           channelName,
                           currAlias);
        updConnecteePathProp().setValue(index,
                                        composeConnecteePath(componentPath,
                                                             outputName,
                                                             channelName,
                                                             alias));
        
        _aliases[index] = alias;
    }

    std::string getLabel() const override {
        OPENSIM_THROW_IF(!isConnected(),
                         InputNotConnected, getName());
        OPENSIM_THROW_IF(isListSocket(),
                         Exception,
                         "Input<T>::getLabel(): this is a list Input; an index "
                         "must be provided.");

        return getLabel(0);
    }

    std::string getLabel(unsigned index) const override {
        OPENSIM_THROW_IF(!isConnected(),
                         InputNotConnected, getName());
        using SimTK::isIndexInRange;
        SimTK_INDEXCHECK_ALWAYS(index, getNumConnectees(),
                                "Input<T>::getLabel()");

        const std::string alias = getAlias(index);
        if (!alias.empty())
            return alias;

        return getChannel(index).getPathName();
    }

    /** Access the values of all the channels connected to this Input as a 
    SimTK::Vector_<T>. The elements are in the same order as the channels.
    */
    SimTK::Vector_<T> getVector(const SimTK::State& state) const {
        SimTK::Vector_<T> v(static_cast<int>(_connectees.size()));
        for (unsigned ichan = 0u; ichan < _connectees.size(); ++ichan) {
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
    
    /** For use in python/java/MATLAB bindings. */
    // This method exists for consistency with Object's safeDownCast.
    static Input<T>* safeDownCast(AbstractInput* base) {
        return dynamic_cast<Input<T>*>(base);
    }

protected:
    /** Create an Input<T> (Socket) that can only connect to an Output<T>
    name and stage at which it should be connected. Only Component should ever
    construct an Input.
    @param name               name of the Output dependency.
    @param connecteePathIndex Index of the property in the containing Component
                              that holds this Input's connectee path(s).
    @param connectAtStage     Stage at which Input should be connected.
    @param owner              The component that contains this input. */
    Input(const std::string& name, const PropertyIndex& connecteePathIndex,
          const SimTK::Stage& connectAtStage, Component& owner) :
        AbstractInput(name, connecteePathIndex, connectAtStage, owner) {}
    
    /** So that Component can construct an Input. */
    friend Component;
    
private:
    /** Register a channel to connect to. Since channels are created on the fly,
     * we do not want to hold onto references to channels. Instead, we hold
     * onto references of outputs. */
    void registerChannel(const AbstractChannel& channel,
            const std::string& alias, bool validate = true)
    {
        const Channel* chanT = nullptr;
        if (validate) {
            chanT = dynamic_cast<const Channel*>(&channel);
            if (!chanT) {
                std::stringstream msg;
                msg << "Type mismatch between Input and Output: Input '"
                    << getName() << "' of type " << getConnecteeTypeName()
                    << " cannot connect to Output (channel) '"
                    << channel.getPathName()
                    << "' of type " << channel.getTypeName() << ".";
                OPENSIM_THROW(Exception, msg.str());
            }
        } else {
            chanT = static_cast<const Channel*>(&channel);
        }
        _registeredChannels.emplace_back(
                SimTK::ReferencePtr<const Output<T>>(&chanT->getOutput()),
                chanT->getChannelName(), alias);
    }
    void connectInternal(const AbstractChannel& channel,
            const std::string& alias) {
        const auto* chanT = dynamic_cast<const Channel*>(&channel);
        if (!chanT) {
            std::stringstream msg;
            msg << "Type mismatch between Input and Output: Input '"
                << getName() << "' of type " << getConnecteeTypeName()
                << " cannot connect to Output (channel) '"
                << channel.getPathName()
                << "' of type " << channel.getTypeName() << ".";
            OPENSIM_THROW(Exception, msg.str());
        }

        if (!isListSocket()) {
            _connectees.clear();
            _aliases.clear();
        }
        _connectees.emplace_back(chanT);
        _aliases.push_back(alias);
    }
    // These are channels the user has requested that we connect to.
    using RegisteredChannel =
            std::tuple<SimTK::ReferencePtr<const Output<T>>,
                    std::string, std::string>;
    SimTK::ResetOnCopy<std::vector<RegisteredChannel>> _registeredChannels;
    SimTK::ResetOnCopy<ChannelList> _connectees;
    // Aliases are serialized, since tools may depend on them for
    // interpreting the connected channels.
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
 * a custom copy constructor---try to use the default one. The socket will
 * not get copied properly if you create a custom copy constructor.
 * We may add support for custom copy constructors with Sockets in the
 * future.
 *
 * @see Component::constructSocket()
 * @relates OpenSim::Socket */
#define OpenSim_DECLARE_SOCKET(cname, T, comment)                           \
    /** @name Sockets                                                    */ \
    /** @{                                                               */ \
    /** comment                                                          */ \
    /** In an XML file, you can set this Socket's connectee path         */ \
    /** via the <b>\<socket_##cname\></b> element.                       */ \
    /** This socket was generated with the                               */ \
    /** #OpenSim_DECLARE_SOCKET macro;                                   */ \
    /** see AbstractSocket for more information.                         */ \
    /** @see connectSocket_##cname##()                                   */ \
    OpenSim_DOXYGEN_Q_PROPERTY(T, cname)                                    \
    /** @}                                                               */ \
    /** @cond                                                            */ \
    PropertyIndex PropertyIndex_socket_##cname {                            \
        this->template constructSocket<T>(#cname,                           \
                "Path to a Component that satisfies the Socket '"           \
                #cname "' of type " #T " (description: " comment ").")      \
    };                                                                      \
    /** @endcond                                                         */ \
    /** @name Socket-related functions                                   */ \
    /** @{                                                               */ \
    /** Connect the '##cname##' Socket to an object of type T##.         */ \
    /** Call finalizeConnections() afterwards to update the socket's     */ \
    /** connectee path property. The reference to the connectee set here */ \
    /** takes precedence over the connectee path property.               */ \
    void connectSocket_##cname(const Object& object) {                      \
        this->updSocket(#cname).connect(object);                            \
    }                                                                       \
    /** @}                                                               */

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
 *
 * @note If you use this macro in your class, then you should *NOT* implement
 * a custom copy constructor---try to use the default one. The Socket will
 * not get copied properly if you create a custom copy constructor.
 * We may add support for custom copy constructors with Sockets in the
 * future.
 *
 * @see Component::constructSocket()
 * @relates OpenSim::Socket */
#define OpenSim_DECLARE_SOCKET_FD(cname, T, comment)                        \
    /** @name Sockets                                                    */ \
    /** @{                                                               */ \
    /** comment                                                          */ \
    /** In an XML file, you can set this socket's connectee path         */ \
    /** via the <b>\<socket_##cname\></b> element.                       */ \
    /** See AbstractSocket for more information.                         */ \
    /** @see connectsocket_##cname##()                                   */ \
    OpenSim_DOXYGEN_Q_PROPERTY(T, cname)                                    \
    /** @}                                                               */ \
    /** @cond                                                            */ \
    PropertyIndex PropertyIndex_socket_##cname {           \
        constructSocket_##cname()                                           \
    };                                                                      \
    /* Declare the method used in the in-class member initializer.       */ \
    /* This method will be defined by OpenSim_DEFINE_SOCKET_FD.          */ \
    PropertyIndex constructSocket_##cname();                                \
    /* Remember the provided type so we can use it in the DEFINE macro.  */ \
    typedef T _socket_##cname##_type;                                       \
    /** @endcond                                                         */ \
    /** @name Socket-related functions                                   */ \
    /** @{                                                               */ \
    /** Connect the '##cname##' Socket to an object of type T##.         */ \
    /** Call finalizeConnections() afterwards to update the socket's     */ \
    /** connectee path property. The reference to the connectee set here */ \
    /** takes precedence over the connectee path property.               */ \
    void connectSocket_##cname(const Object& object) {                      \
        this->updSocket(#cname).connect(object);                            \
    }                                                                       \
    /** @}                                                               */

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
 * @relates OpenSim::Socket */
// This macro defines the method that the in-class member initializer calls
// to construct the Socket. The reason why this must be in the .cpp file is
// that putting the template member function `template <typename T>
// Component::constructSocket` in the header requires that `T` is not an
// incomplete type (specifically, when compiling cpp files for classes OTHER
// than `MyComponent` but that include MyComponent.h). OpenSim::Geometry is an
// example of this scenario.
#define OpenSim_DEFINE_SOCKET_FD(cname, Class)                              \
PropertyIndex Class::constructSocket_##cname() {                            \
    using T = _socket_##cname##_type;                                       \
    std::string typeStr = T::getClassName();                                \
    return this->template constructSocket<T>(#cname,                        \
        "Path to a Component that satisfies the Socket '"                   \
        #cname "' of type " + typeStr + ".");                               \
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
 *      OpenSim_DECLARE_INPUT(emg, double, SimTK::Stage::Velocity, "For validation.");
 *      ...
 *  };
 *  @endcode
 *
 * @note If you use this macro in your class, then you should *NOT* implement
 * a custom copy constructor---try to use the default one. The Input will
 * not get copied properly if you create a custom copy constructor.
 * We may add support for custom copy constructors with Inputs in the future.
 *
 * @see Component::constructInput()
 * @relates OpenSim::Input */
#define OpenSim_DECLARE_INPUT(iname, T, istage, comment)                    \
    /** @name Inputs                                                     */ \
    /** @{                                                               */ \
    /** comment                                                          */ \
    /** This input is needed at stage istage##.                          */ \
    /** In an XML file, you can set this Input's connectee path          */ \
    /** via the <b>\<input_##iname\></b> element.                        */ \
    /** The syntax for a connectee path is                               */ \
    /** `<path/to/component>|<output_name>[:<channel_name>][(<alias>)]`. */ \
    /** This input was generated with the                                */ \
    /** #OpenSim_DECLARE_INPUT macro;                                    */ \
    /** see AbstractInput for more information.                          */ \
    /** @see connectInput_##iname##()                                    */ \
    OpenSim_DOXYGEN_Q_PROPERTY(T, iname)                                    \
    /** @}                                                               */ \
    /** @cond                                                            */ \
    PropertyIndex PropertyIndex_input_##iname {                             \
        this->template constructInput<T>(#iname, false,                     \
            "Path to an output (channel) to satisfy the one-value Input '"  \
            #iname "' of type " #T " (description: " comment ").",  istage) \
    };                                                                      \
    /** @endcond                                                         */ \
    /** @name Input-related functions                                    */ \
    /** @{                                                               */ \
    /** Connect this Input to a single-valued (single-channel) Output.   */ \
    /** The output must be of type T##.                                  */ \
    /** This input is single-valued and thus cannot connect to a         */ \
    /** list output.                                                     */ \
    /** You can optionally provide an alias that will be used by this    */ \
    /** component to refer to the output.                                */ \
    /** Call finalizeConnections() afterwards to update the input's      */ \
    /** connectee path property. The reference to the output set here    */ \
    /** takes precedence over the connectee path property.               */ \
    void connectInput_##iname(const AbstractOutput& output,                 \
                              const std::string& alias = "") {              \
        updInput(#iname).connect(output, alias);                            \
    }                                                                       \
    /** Connect this Input to an output channel of type T##.             */ \
    /** You can optionally provide an alias that will be used by this    */ \
    /** component to refer to the channel.                               */ \
    /** Call finalizeConnections() afterwards to update the input's      */ \
    /** connectee path property. The reference to the channel set here   */ \
    /** takes precedence over the connectee path property.               */ \
    void connectInput_##iname(const AbstractChannel& channel,               \
                              const std::string& alias = "") {              \
        updInput(#iname).connect(channel, alias);                           \
    }                                                                       \
    /** @}                                                               */

// TODO create new macros to handle custom copy constructors: with
// constructInput_() methods, etc. NOTE: constructProperty_() must be called
// first within these macros, b/c the connectee path property must exist before
// the Input etc is constructed.


/** Create a list input, which can connect to more than one Channel. This
 * makes sense for components like reporters that can handle a flexible
 * number of input values. 
 *
 * @note If you use this macro in your class, then you should *NOT* implement
 * a custom copy constructor---try to use the default one. The Input will
 * not get copied properly if you create a custom copy constructor.
 * We may add support for custom copy constructors with Inputs in the future.
 *
 * @see Component::constructInput()
 * @relates OpenSim::Input */
#define OpenSim_DECLARE_LIST_INPUT(iname, T, istage, comment)               \
    /** @name Inputs (list)                                              */ \
    /** @{                                                               */ \
    /** comment                                                          */ \
    /** This input can connect to multiple outputs, all of which are     */ \
    /** needed at stage istage##.                                        */ \
    /** In an XML file, you can set this Input's connectee path          */ \
    /** via the <b>\<input_##iname\></b> element.                        */ \
    /** The syntax for a connectee path is                               */ \
    /** `<path/to/component>|<output_name>[:<channel_name>][(<alias>)]`. */ \
    /** This input was generated with the                                */ \
    /** #OpenSim_DECLARE_LIST_INPUT macro;                               */ \
    /** see AbstractInput for more information.                          */ \
    /** @see connectInput_##iname##()                                    */ \
    OpenSim_DOXYGEN_Q_PROPERTY(T, iname)                                    \
    /** @}                                                               */ \
    /** @cond                                                            */ \
    PropertyIndex PropertyIndex_input_##iname {                             \
        this->template constructInput<T>(#iname, true,                      \
            "Paths to outputs (channels) to satisfy the list Input '"       \
            #iname "' of type " #T " (description: " comment "). "          \
            "To specify multiple paths, put spaces between them.", istage)  \
    };                                                                      \
    /** @endcond                                                         */ \
    /** @name Input-related functions                                    */ \
    /** @{                                                               */ \
    /** Connect this Input to a single-valued or list Output.            */ \
    /** The output must be of type T##.                                  */ \
    /** If the output is a list output, this connects to all of the      */ \
    /** channels of the output.                                          */ \
    /** You can optionally provide an alias that will be used by this    */ \
    /** component to refer to the output; the alias will be used for all */ \
    /** channels of the output.                                          */ \
    /** Call finalizeConnections() afterwards to update the input's      */ \
    /** connectee path property. The reference to the output set here    */ \
    /** takes precedence over the connectee path property.               */ \
    void connectInput_##iname(const AbstractOutput& output,                 \
                              const std::string& alias = "") {              \
        updInput(#iname).connect(output, alias);                            \
    }                                                                       \
    /** Connect this Input to an output channel of type T##.             */ \
    /** You can optionally provide an alias that will be used by this    */ \
    /** component to refer to the channel.                               */ \
    /** Call finalizeConnections() afterwards to update the input's      */ \
    /** connectee path property. The reference to the channel set here   */ \
    /** takes precedence over the connectee path property.               */ \
    void connectInput_##iname(const AbstractChannel& channel,               \
                              const std::string& alias = "") {              \
        updInput(#iname).connect(channel, alias);                           \
    }                                                                       \
    /** @}                                                               */
/// @}

} // end of namespace OpenSim

#endif  // OPENSIM_COMPONENT_SOCKET_H_
