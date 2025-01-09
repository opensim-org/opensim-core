// Make all of OpenSim and Simbody's classes available.
#include <OpenSim/OpenSim.h>
// "Use" the OpenSim namespace and certain SimTK symbols to shorten
// code ("using namespace SimTK" would cause conflicts for Body).
using namespace OpenSim;
using SimTK::Vec3;
using SimTK::Inertia;
using SimTK::Pi;
using SimTK::convertDegreesToRadians;

// Build an OpenSim dynamic walker model and save it to an OSIM file.
int main() {
    try {
        // Code to the construct the model will go here
        // Section: Setup
        // Define key model variables
        double pelvisWidth = 0.20;
        double thighLength = 0.40;
        double shankLength = 0.435;

        // Create an OpenSim Model
        Model osimModel;
        osimModel.setName("DynamicWalkerModel");

        // Get a reference to the ground object
        const Ground& ground = osimModel.getGround();

        // Define the acceleration due to gravity
        osimModel.setGravity(Vec3(0, -9.80665, 0));

        // Section: Create the Platform Body
        double mass = 1;

        // Location of the center of mass from the body origin expressed in the
        // body frame.
        Vec3 comLocInBody(0.0, 0.0, 0.0);

        // Inertia of the body expressed in the body frame
        Inertia bodyInertia(1.0, 1.0, 1.0, 0.0, 0.0, 0.0);

        // Create the body
        Body* platform = new Body("Platform", mass, comLocInBody, bodyInertia);

        // Add geometry to display in the GUI
        platform->attachGeometry(new Brick(Vec3(1, 0.05, 1)));

        // Add the Platform Body to the Model
        osimModel.addBody(platform);

        // Section: Create the Platform Joint
        // Create the joint connection the platform to the ground
        Vec3 locationInParent(0.0, 0.0, 0.0);
        Vec3 orientationInParent(0.0, 0.0, 0.0);
        Vec3 locationInChild(0.0, 0.0, 0.0);
        Vec3 orientationInChild(0.0, 0.0, 0.0);
        PinJoint *platformToGround = new PinJoint("PlatformToGround",
            ground, locationInParent, orientationInParent,
            *platform, locationInChild, orientationInChild);

        // Section: Set the properties of the coordinates that define the joint
        // A pin joint consists of a single coordinate describing a change in
        // orientation about the Z axis
        Coordinate& platform_rz =
            platformToGround->updCoordinate(PinJoint::Coord::RotationZ);
        platform_rz.setName("platform_rz");
        double platform_rz_range[2] = {-Pi / 2.0, 0};
        platform_rz.setRange(platform_rz_range);
        platform_rz.setDefaultValue(convertDegreesToRadians(-10.0));
        platform_rz.setDefaultLocked(true);

        // Add the PlatformToGround Joint to the Model
        osimModel.addJoint(platformToGround);

        // Section: Create the Pelvis
        mass = 1;

        // Location of the Center of Mass from the Body Origin expressed in
        // Body Frame
        comLocInBody = Vec3(0.0, 0.0, 0.0);

        // Inertia of the Body expressed in the Body Frame
        bodyInertia = Inertia(1.0, 1.0, 1.0, 0.0, 0.0, 0.0);

        // Create the body
        Body* pelvis = new Body("Pelvis", mass, comLocInBody, bodyInertia);

        // Add geometry to display in the GUI
        pelvis->attachGeometry(new Ellipsoid(
                    pelvisWidth / 4.0, pelvisWidth / 4.0, pelvisWidth / 2.0));

        // Add the Pelvis Body to the Model
        osimModel.addBody(pelvis);

        // Create the joint which connects the Pelvis to the Platform
        locationInParent = Vec3(0.0, 0.0, 0.0);
        orientationInParent = Vec3(0.0, 0.0, 0.0);
        locationInChild = Vec3(0.0, 0.0, 0.0);
        orientationInChild = Vec3(0.0, 0.0, 0.0);
        PlanarJoint *pelvisToPlatform = new PlanarJoint("PelvisToPlatform",
            *platform, *pelvis);

        // A planar joint has three coordinates:
        //     RotationZ, TranslationX, TranslationY
        // Set the properties of the coordinates that define the joint
        Coordinate& pelvis_rz =
            pelvisToPlatform->updCoordinate(PlanarJoint::Coord::RotationZ);
        pelvis_rz.setName("pelvis_rz");
        double pelvis_rz_range[2] = { -Pi, Pi };
        pelvis_rz.setRange(pelvis_rz_range);
        pelvis_rz.setDefaultValue(convertDegreesToRadians(0));
        pelvis_rz.setDefaultLocked(true);

        Coordinate& pelvis_tx =
            pelvisToPlatform->updCoordinate(PlanarJoint::Coord::TranslationX);
        pelvis_tx.setName("pelvis_tx");
        double pelvis_tx_range[2] = { -10, 10 };
        pelvis_tx.setRange(pelvis_tx_range);
        pelvis_tx.setDefaultValue(0);

        Coordinate& pelvis_ty =
            pelvisToPlatform->updCoordinate(PlanarJoint::Coord::TranslationY);
        pelvis_ty.setName("pelvis_ty");
        double pelvis_ty_range[2] = { -1, 2 };
        pelvis_ty.setRange(pelvis_ty_range);
        pelvis_ty.setDefaultValue(1);

        // Add the PelvisToPlatform Joint to the Model
        osimModel.addJoint(pelvisToPlatform);

        Body* leftThigh = new Body("LeftThigh", 1, Vec3(0),
            Inertia(1, 1, 1, 0, 0, 0));
        Body* rightThigh = new Body("RightThigh", 1, Vec3(0),
            Inertia(1, 1, 1, 0, 0, 0));
        Body* leftShank = new Body("LeftShank", 1, Vec3(0),
            Inertia(1, 1, 1, 0, 0, 0));
        Body* rightShank = new Body("RightShank", 1, Vec3(0),
            Inertia(1, 1, 1, 0, 0, 0));

        leftThigh->attachGeometry(new Ellipsoid(
            0.05 * thighLength, 0.5 * thighLength, 0.05 * thighLength));
        rightThigh->attachGeometry(new Ellipsoid(
            0.05 * thighLength, 0.5 * thighLength, 0.05 * thighLength));
        leftShank->attachGeometry(new Ellipsoid(
            0.05 * shankLength, 0.5 * shankLength, 0.05 * shankLength));
        rightShank->attachGeometry(new Ellipsoid(
            0.05 * shankLength, 0.5 * shankLength, 0.05 * shankLength));

        osimModel.addBody(leftThigh);
        osimModel.addBody(rightThigh);
        osimModel.addBody(leftShank);
        osimModel.addBody(rightShank);

        PinJoint *leftThighToPelvis = new PinJoint("LeftThighToPelvis",
            *pelvis, Vec3(0, 0, -0.5 * pelvisWidth), Vec3(0),
            *leftThigh, Vec3(0, 0.5 * thighLength, 0), Vec3(0));

        Coordinate& LHip_rz = leftThighToPelvis->updCoordinate();
        LHip_rz.setName("LHip_rz");
        double LHip_rz_range[2] =
            { convertDegreesToRadians(-100), convertDegreesToRadians(100) };
        LHip_rz.setRange(LHip_rz_range);
        LHip_rz.setDefaultValue(convertDegreesToRadians(-10));

        PinJoint *rightThighToPelvis = new PinJoint("RightThighToPelvis",
            *pelvis, Vec3(0, 0, 0.5 * pelvisWidth), Vec3(0),
            *rightThigh, Vec3(0, 0.5 * thighLength, 0), Vec3(0));

        Coordinate& RHip_rz = rightThighToPelvis->updCoordinate();
        RHip_rz.setName("RHip_rz");
        double RHip_rz_range[2] =
            { convertDegreesToRadians(-100), convertDegreesToRadians(100) };
        RHip_rz.setRange(RHip_rz_range);
        RHip_rz.setDefaultValue(convertDegreesToRadians(30));


        PinJoint *leftShankToThigh = new PinJoint("LeftShankToThigh",
            *leftThigh, Vec3(0, -0.5 * thighLength, 0), Vec3(0),
            *leftShank, Vec3(0, 0.5 * shankLength, 0), Vec3(0));

        Coordinate& LKnee_rz = leftShankToThigh->updCoordinate();
        LKnee_rz.setName("LKnee_rz");
        double LKnee_rz_range[2] = { convertDegreesToRadians(-100), 0.0 };
        LKnee_rz.setRange(LKnee_rz_range);
        LKnee_rz.setDefaultValue(convertDegreesToRadians(-30));

        PinJoint *rightShankToThigh = new PinJoint("RightShankToThigh",
            *rightThigh, Vec3(0, -0.5 * thighLength, 0), Vec3(0),
            *rightShank, Vec3(0, 0.5 * shankLength, 0), Vec3(0));

        Coordinate& RKnee_rz = rightShankToThigh->updCoordinate();
        RKnee_rz.setName("RKnee_rz");
        double RKnee_rz_range[2] = { convertDegreesToRadians(-100), 0.0 };
        RKnee_rz.setRange(RKnee_rz_range);
        RKnee_rz.setDefaultValue(convertDegreesToRadians(-30));

        osimModel.addJoint(leftThighToPelvis);
        osimModel.addJoint(rightThighToPelvis);
        osimModel.addJoint(leftShankToThigh);
        osimModel.addJoint(rightShankToThigh);
        
        // Section: Add Contact Geometry
        // Add Contact Mesh for Platform
        // The initial orientation is defined by a normal pointing in the
        // positive x direction
        ContactHalfSpace* platformContact = new ContactHalfSpace(
            Vec3(0.0, 0.0, 0.0), Vec3(0.0, 0.0, -Pi / 2.0),
            *platform, "PlatformContact");
        osimModel.addContactGeometry(platformContact);

        // Contact Sphere Properties
        double contactSphereRadius = 0.05;

        // Add Contact Sphere for Right Hip
        Vec3 rightHipLocationInPelvis(0.0, 0.0, pelvisWidth / 2.0);
        ContactSphere* rightHipContact = new ContactSphere(contactSphereRadius,
            rightHipLocationInPelvis,
            *pelvis, "RHipContact");
        osimModel.addContactGeometry(rightHipContact);

        Vec3 leftHipLocationInPelvis(0.0, 0.0, -pelvisWidth / 2.0);
        ContactSphere* leftHipContact = new ContactSphere(contactSphereRadius,
            leftHipLocationInPelvis,
            *pelvis, "LHipContact");
        osimModel.addContactGeometry(leftHipContact);

        ContactSphere* rightKneeContact = new ContactSphere(contactSphereRadius,
            Vec3(0, -0.5 * thighLength, 0),
            *rightThigh, "RKneeContact");
        osimModel.addContactGeometry(rightKneeContact);

        ContactSphere* leftKneeContact = new ContactSphere(contactSphereRadius,
            Vec3(0, -0.5 * thighLength, 0),
            *leftThigh, "LKneeContact");
        osimModel.addContactGeometry(leftKneeContact);

        ContactSphere* leftFootContact = new ContactSphere(contactSphereRadius,
            Vec3(0, -0.5 * shankLength, 0),
            *leftShank, "LFootContact");
        osimModel.addContactGeometry(leftFootContact);

        ContactSphere* rightFootContact = new ContactSphere(contactSphereRadius,
            Vec3(0, -0.5 * shankLength, 0),
            *rightShank, "RFootContact");
        osimModel.addContactGeometry(rightFootContact);

        // Section: Add HuntCrossleyForces
        // Define contact parameters for all the spheres
        double stiffness = 1e7;
        double dissipation = 0.1;
        double staticFriction = 0.6;
        double dynamicFriction = 0.4;
        double viscosity = 0.01;

        // Right Hip Contact Parameters
        HuntCrossleyForce::ContactParameters *rightHipContactParams =
            new HuntCrossleyForce::ContactParameters(stiffness,
                dissipation, staticFriction, dynamicFriction, viscosity);
        rightHipContactParams->addGeometry("RHipContact");
        rightHipContactParams->addGeometry("PlatformContact");

        // Right Hip Force
        HuntCrossleyForce* rightHipForce = new HuntCrossleyForce(
            rightHipContactParams);
        rightHipForce->setName("RightHipForce");

        osimModel.addForce(rightHipForce);

        HuntCrossleyForce::ContactParameters *leftHipContactParams =
            new HuntCrossleyForce::ContactParameters(stiffness,
                dissipation, staticFriction, dynamicFriction, viscosity);
        leftHipContactParams->addGeometry("LHipContact");
        leftHipContactParams->addGeometry("PlatformContact");

        HuntCrossleyForce* leftHipForce = new HuntCrossleyForce(
            leftHipContactParams);
        leftHipForce->setName("LeftHipForce");
        //Add Force
        osimModel.addForce(leftHipForce);

        HuntCrossleyForce::ContactParameters *rightKneeContactParams =
            new HuntCrossleyForce::ContactParameters(stiffness,
                dissipation, staticFriction, dynamicFriction, viscosity);
        rightKneeContactParams->addGeometry("RKneeContact");
        rightKneeContactParams->addGeometry("PlatformContact");

        HuntCrossleyForce* rightKneeForce = new HuntCrossleyForce(
            rightKneeContactParams);
        rightKneeForce->setName("RightKneeForce");
        osimModel.addForce(rightKneeForce);

        HuntCrossleyForce::ContactParameters *leftKneeContactParams =
            new HuntCrossleyForce::ContactParameters(stiffness,
                dissipation, staticFriction, dynamicFriction, viscosity);
        leftKneeContactParams->addGeometry("LKneeContact");
        leftKneeContactParams->addGeometry("PlatformContact");

        HuntCrossleyForce* leftKneeForce = new HuntCrossleyForce(
            leftKneeContactParams);
        leftKneeForce->setName("LeftKneeForce");
        osimModel.addForce(leftKneeForce);
        
        HuntCrossleyForce::ContactParameters *leftFootContactParams =
            new HuntCrossleyForce::ContactParameters(stiffness,
                dissipation, staticFriction, dynamicFriction, viscosity);
        leftFootContactParams->addGeometry("LFootContact");
        leftFootContactParams->addGeometry("PlatformContact");

        HuntCrossleyForce* leftFootForce = new HuntCrossleyForce(
            leftFootContactParams);
        leftFootForce->setName("LeftFootForce");
        osimModel.addForce(leftFootForce);

        HuntCrossleyForce::ContactParameters *rightFootContactParams =
            new HuntCrossleyForce::ContactParameters(stiffness,
                dissipation, staticFriction, dynamicFriction, viscosity);
        rightFootContactParams->addGeometry("RFootContact");
        rightFootContactParams->addGeometry("PlatformContact");

        HuntCrossleyForce* rightFootForce = new HuntCrossleyForce(
            rightFootContactParams);
        rightFootForce->setName("RightFootForce");
        osimModel.addForce(rightFootForce);

        CoordinateLimitForce* clfLHip = new CoordinateLimitForce("LHip_rz",
            100, 1e6, -100, 1e6, 1e5, 5);
        osimModel.addForce(clfLHip);
        CoordinateLimitForce* clfRHip = new CoordinateLimitForce("RHip_rz",
            100, 1e6, -100, 1e6, 1e5, 5);
        osimModel.addForce(clfRHip);
        CoordinateLimitForce* clfLKnee = new CoordinateLimitForce("LKnee_rz",
            0, 1e6, -100, 1e6, 1e5, 5);
        osimModel.addForce(clfLKnee);
        CoordinateLimitForce* clfRKnee = new CoordinateLimitForce("RKnee_rz",
            0, 1e6, -100, 1e6, 1e5, 5);
        osimModel.addForce(clfRKnee);
        osimModel.finalizeConnections(); // Needed so sockets have correct absolute path on print

        // Save the model to a file
        osimModel.print("DynamicWalkerModel.osim");
    } catch (const std::exception& ex) {
        std::cout << ex.what() << std::endl;
        return 1;
    } catch (...) {
        std::cout << "UNRECOGNIZED EXCEPTION" << std::endl;
        return 1;
    }
    std::cout << "OpenSim example completed successfully" << std::endl;
    // std::cout << "Press return to continue" << std::endl;
    // std::cin.get();
    return 0;
}
