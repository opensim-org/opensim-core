// Important note: Have to include OpenSim before
// xsens because otherwise there will be compiler
// errors in Simbody headers. This happens because
// xsens sysmbols are in global namespace.
#include <OpenSim/OpenSim.h>

#include <xsensdeviceapi.h>
#include <xsens/xsstring.h>
#include <xsens/xscontrol.h>
#include <xsens/xsmutex.h>

#include <iostream>
#include <iomanip>
#include <mutex>

template<typename Port>
Port findWirelessMaster(Port portsBegin, Port portsEnd) {
    while(portsBegin != portsEnd &&
          !portsBegin->deviceId().isWirelessMaster())
        ++portsBegin;
    
    if(portsBegin == portsEnd)
        throw std::runtime_error{"No wireless master found."};

    return portsBegin;
}

class WirelessMasterCallback : public XsCallback {
protected:
    virtual void onConnectivityChanged(XsDevice* device, 
                                       XsConnectivityState newState) override {
        if(newState == XCS_Disconnected) {
            std::cout << "Master event: disconnected " << device->deviceId()
                      << std::endl;
        } else if (newState == XCS_Rejected) {
            std::cout << "Master event: rejected "     << device->deviceId()
                      << std::endl;
        } else if (newState == XCS_PluggedIn) {
            std::cout << "Master event: plugged-in "   << device->deviceId()
                      << std::endl;
        } else if (newState == XCS_Wireless) {
            std::cout << "Master event: connected "    << device->deviceId()
                      << std::endl;
        } else if (newState == XCS_File) {
            std::cout << "Master event: file "         << device->deviceId()
                      << std::endl;
        } else if (newState == XCS_Unknown) {
            std::cout << "Master event: unknown "      << device->deviceId()
                      << std::endl;
        } else {
            std::cout << "Master event: error "        << device->deviceId() 
                      << std::endl; 
        }
    }
};

class MtwCallback : public XsCallback {
public:
    MtwCallback() = default;
    
    MtwCallback(OpenSim::Model& model) :
        _numPackets{},
        _model{&model}
    {}
    
protected:
    double degToRad(double deg) {
        constexpr double DEGTORAD{0.01745329};
        return deg * DEGTORAD;
    }

    void onDeviceStateChanged(XsDevice *dev, 
                              XsDeviceState newState, 
                              XsDeviceState oldState) override {
        std::cout << "MtwCallback::onDeviceStateChanged()" << std::endl;
    }

    void onLiveDataAvailable(XsDevice* device, 
                             const XsDataPacket* packet) override {
        const auto euler = packet->orientationEuler();
        ++_numPackets;
        if(_numPackets % 10 == 0) {
            auto& state = _model->updWorkingState();
            // Switch yaw and pitch to make the visualizer aligned with IMU.
            _model->getCoordinateSet()[0].setValue(state, 
                                                   degToRad(euler.roll()));
            _model->getCoordinateSet()[1].setValue(state, 
                                                   degToRad(euler.yaw()));
            _model->getCoordinateSet()[2].setValue(state, 
                                                   degToRad(euler.pitch()));
            _model->updVisualizer().updSimbodyVisualizer().drawFrameNow(state);
            
            std::cout << "Packet: " << _numPackets << " " 
                      << "Device: " << device->deviceId() << " "
                      << " Roll: "
                      << std::setw(8) << std::fixed << std::setprecision(2)
                      << euler.x()
                      << " Pitch: "
                      << std::setw(8) << std::fixed << std::setprecision(2)
                      << euler.y()
                      << " Yaw: "
                      << std::setw(8) << std::fixed << std::setprecision(2)
                      << euler.z()
                      << std::endl;
        }
    }

    void onMissedPackets(XsDevice *dev, 
                         int count, 
                         int first, 
                         int last) override {
        std::cout << "MtwCallback::onMissedPackets()" << std::endl;
    }

    void onWakeupReceived(XsDevice *dev) override {
        std::cout << "MtwCallback::onWakeupReceived()" << std::endl;
    }

    void onProgressUpdated(XsDevice *dev, 
                           int current, 
                           int total, 
                           const XsString *identifier) override {
        std::cout << "MtwCallback::onProgressUpdated()" << std::endl;
    }

    int onWriteMessageToLogFile(XsDevice *dev, 
                                const XsMessage *message) override {
        std::cout << "MtwCallback::onWriteMessageToLogFile()" << std::endl;

        return 0;
    }

    void onRecordingDataAvailable(XsDevice *dev, 
                                  const XsDataPacket *data) override {
        std::cout << "MtwCallback::onRecordingDataAvailable()" << std::endl;
    }

    void onConnectivityChanged(XsDevice *dev, 
                               XsConnectivityState newState) override {
        std::cout << "MtwCallback::onConnectivityChanged()" << std::endl;
    }

    void onInfoResponse(XsDevice *dev, 
                        XsInfoRequest request) override {
        std::cout << "MtwCallback::onInfoResponse()" << std::endl;
    }

    void onError(XsDevice *dev, 
                 XsResultValue error) override {
        std::cout << "MtwCallback::onError()" << std::endl;
    }

    void onNonDataMessage(XsDevice *dev, 
                          XsMessage const *message) override {
        std::cout << "MtwCallback::onNonDataMessage()" << std::endl;
    }

    void onMessageReceivedFromDevice(XsDevice *dev, 
                                     XsMessage const *message) override {
        std::cout << "MtwCallback::onMessageReceivedFromDevice()" << std::endl;
    }

    void onMessageSentToDevice(XsDevice *dev, 
                               XsMessage const *message) override {
        std::cout << "MtwCallback::onMessageSentToDevice()" << std::endl;
    }

    void onAllLiveDataAvailable(XsDevicePtrArray *devs, 
                                const XsDataPacket **packets) override {
        std::cout << "MtwCallback::onAllLiveDataAvailable()" << std::endl;
    }

    void onAllRecordingDataAvailable(XsDevicePtrArray *devs, 
                                     const XsDataPacket **packets) override {
        std::cout << "MtwCallback::onAllLiveDataAvailable()" << std::endl;
    }

private:
    unsigned _numPackets;
    OpenSim::Model* _model;
};

int main() {
    constexpr unsigned UPDATE_RATE{60};
    constexpr unsigned RADIO_CHANNEL{19};

    std::cout << "Contruct XsControl ... ";
    auto xscontrol = XsControl::construct();
    if(!xscontrol)
        throw std::runtime_error{"Failed to construct XsControl."};
    std::cout << "done" << std::endl;

    std::cout << "Scan ports ... begin" << std::endl;
    auto detectedPorts = XsScanner::scanPorts();
    for(auto port = detectedPorts.begin();
        port != detectedPorts.end();
        ++port)
        std::cout << *port << std::endl;
    std::cout << "Scan ports ... done" << std::endl;

    std::cout << "Find wireless master ... ";
    auto wirelessMasterPort = findWirelessMaster(detectedPorts.begin(),
                                                 detectedPorts.end());
    std::cout << "done" << std::endl;

    std::cout << "Open wireless master port ... ";
    if(!xscontrol->openPort(*wirelessMasterPort))
        throw std::runtime_error{"Cannot open port to wireless master."};
    std::cout << "done" << std::endl;

    std::cout << "Retrieve wireless master device ... ";
    auto wirelessMaster = xscontrol->device(wirelessMasterPort->deviceId());
    if(wirelessMaster == nullptr)
        throw std::runtime_error{"Unable to retrieve wireless master device."};
    std::cout << "done" << std::endl;

    std::cout << "Go to config mode ... ";
    if(!wirelessMaster->gotoConfig())
        throw std::runtime_error{"Unable to configure wireless master."};
    std::cout << "done" << std::endl;

    std::cout << "Add callback handler for wireless master ... ";
    WirelessMasterCallback wirelessMasterCallback{};
    wirelessMaster->addCallbackHandler(&wirelessMasterCallback);
    std::cout << "done" << std::endl;

    std::cout << "Set update rate for wireless master ... ";
    if(!wirelessMaster->setUpdateRate(UPDATE_RATE))
        throw std::runtime_error{"Cannot set update rate for wireless master."};
    std::cout << "done" << std::endl;

    std::cout << "Set radio channel for wireless master ... ";
    if(!wirelessMaster->enableRadio(RADIO_CHANNEL))
        throw std::runtime_error{"Cannot set the radio channel for wireless"
                                 " master."};
    std::cout << "done" << std::endl;

    std::cout << "Press any key to start measurement when ready. " << std::endl;
    std::getchar();

    std::cout << "Start measurement for wireless master ... ";
    if(!wirelessMaster->gotoMeasurement())
        throw std::runtime_error{"Cannot put master into measurement mode."};
    std::cout << "done" << std::endl;

    std::cout << "Retrieve devices connected to wireless master ... begin"
              << std::endl;
    auto deviceIds = xscontrol->deviceIds();
    std::vector<XsDevice*> connectedDevices{};
    for(auto dev = deviceIds.begin(); dev != deviceIds.end(); ++dev)
        if(dev->isMtw()) {
            connectedDevices.push_back(xscontrol->device(*dev));
            std::cout << *dev << std::endl;
        }
    std::cout << "Retrieve devices connected to wireless master ... end"
              << std::endl;

    OpenSim::Model model{};
    model.setUseVisualizer(true);
    
    auto slab = new OpenSim::Body{"slab", 
                                  1, 
                                  SimTK::Vec3{0}, 
                                  SimTK::Inertia{0}};

    auto balljoint = new OpenSim::BallJoint{"balljoint",
                                            model.getGround(),
                                            SimTK::Vec3{0, 1, 0},
                                            SimTK::Vec3{0},
                                            *slab,
                                            SimTK::Vec3{0},
                                            SimTK::Vec3{0}};

    OpenSim::Brick brick{};
    brick.setFrame(*slab);
    brick.set_half_lengths(SimTK::Vec3{0.5, 0.05, 0.25});
    slab->addComponent(brick.clone());

    model.addBody(slab);
    model.addJoint(balljoint);

    auto& state = model.initSystem();

    model.updMatterSubsystem().setShowDefaultGeometry(false);
    SimTK::Visualizer& viz = model.updVisualizer().updSimbodyVisualizer();
    viz.setBackgroundType(viz.GroundAndSky);
    viz.setShowSimTime(true);
    viz.drawFrameNow(state);
    
    std::cout << "Add callback handler for first device ... begin" << std::endl;
    MtwCallback mtwCallback{model};
    connectedDevices.at(0)->addCallbackHandler(&mtwCallback);
    std::cout << "Add callback handler for first device ... end" << std::endl;

    // std::cout << "Add callback handler for devices ... begin" << std::endl;
    // std::vector<MtwCallback> deviceCallbacks{connectedDevices.size()};
    // for(auto i = 0u; i < connectedDevices.size(); ++i)
        // connectedDevices[i]->addCallbackHandler(&deviceCallbacks.at(i));
    // std::cout << "Add callback handler for devices ... end" << std::endl;

    std::cout << "Measuring ..." << std::endl;
    std::cout << "(Press any key to stop)" << std::endl;
    std::getchar();

    std::cout << "Go to config mode ... " << std::endl;
    if(!wirelessMaster->gotoConfig())
        throw std::runtime_error{"Unable to configure wireless master."};
    std::cout << "done" << std::endl;

    std::cout << "Disable radio for wireless master ... begin";
    if(!wirelessMaster->disableRadio())
        throw std::runtime_error{"Cannot disable radio for wireless master."};
    std::cout << "Disable radio for wireless master ... end";

    std::cout << "Close control ... ";
    xscontrol->close();
    std::cout << "done" << std::endl;

    return 0;
}
