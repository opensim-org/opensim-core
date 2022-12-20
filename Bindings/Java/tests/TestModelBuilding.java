import org.opensim.modeling.*;
import java.io.IOException;

class TestModelBuilding {
  public static void main(String[] args) {

  try {
        Vec3 zv = ArrayDouble.createVec3(0, 0, 0);
        Vec3 zoz = ArrayDouble.createVec3(0, 1, 0);

        Model arm = new Model();

        Body hum = new Body();
        hum.setName("humerus");
        hum.setMass(1.0);

        Body rad = new Body();
        rad.setName("radius");
        rad.setMass(1.0);

        PinJoint shoulder = new PinJoint("shoulder", arm.getGround(), zv, zv, hum, zv, zoz);
        PinJoint elbow = new PinJoint("elbow", hum, zv, zv, rad, zv, zoz);
        
        Millard2012AccelerationMuscle biceps = new Millard2012AccelerationMuscle("biceps",
            200.0,      // Max isometric force
            0.6,        // Optimal fibre length
            0.55,       // Tendon slack length
            0.0);       // Pennation angle
        biceps.addNewPathPoint("origin", hum, ArrayDouble.createVec3(new double[]{0, 0.8, 0}));
        biceps.addNewPathPoint("insert", rad, ArrayDouble.createVec3(new double[]{0, 0.7, 0}));

        PrescribedController cns = new PrescribedController();
        cns.addActuator(biceps);
        StepFunction testFunction = new StepFunction(0.5, 3.0, 0.3, 1.0);
        cns.prescribeControlForActuator(0,      //Index in controller set
            testFunction); 
        System.gc(); // Request gc could free testFunction and crash
        arm.addBody(hum);
        arm.addBody(rad);
        arm.addJoint(shoulder);
        arm.addJoint(elbow);
        
        arm.addForce(biceps);
        arm.addController(cns);
        /*
        reporter = TableReporter();
        reporter.set_report_time_interval(1.0); % Why is this not camelCase?
        reporter.updInput('inputs').connect(biceps.getOutput('fiber_force'));
        elbow_coord = elbow.getCoordinateSet().get(0).getOutput('value');
        reporter.updInput('inputs').connect(elbow_coord, 'elbow_angle');
        arm.addComponent(reporter);
        */
        State state = arm.initSystem();
        arm.updCoordinateSet().get(0).setLocked(state, true);
        arm.updCoordinateSet().get(1).setValue(state, 0.5*3.14);
        arm.equilibrateMuscles(state);

        // WrapObjects.
        WrapSphere sphere = new WrapSphere();
        arm.getGround().addWrapObject(sphere);

        WrapCylinder cylinder = new WrapCylinder();
        cylinder.set_radius(0.5);
        arm.getGround().addWrapObject(cylinder);

        WrapTorus torus = new WrapTorus();
        arm.getGround().addWrapObject(torus);

        WrapEllipsoid ellipsoid = new WrapEllipsoid();
        arm.getGround().addWrapObject(ellipsoid);
        {
            // test TrasnformAxis.setFunction issue #3210
            StepFunction stepFunction = new StepFunction(0.5, 3.0, 0.3, 1.0);
            TransformAxis transformAxis = new TransformAxis();
            transformAxis.setFunction(stepFunction);
            System.gc(); // Request gc could free stepFunction and crash
            Function f = transformAxis.getFunction();
            System.out.println(f.dump());
        }
        System.out.println("Test finished!");
    }
    catch (IOException ex){
        System.exit(-1);
    }
    System.gc();
    // TODO to cause test to fail: System.exit(-1);
  }
}
