import org.opensim.modeling.*;

class TestBasics {
  public static void main(String[] args) {
    Model m = new Model();
    m.print("empty_model.osim");
    Frame gnd = m.getGround();
    FrameList frames = m.getFrameList();
    FrameIterator fi = frames.begin();
    while (!fi.equals(frames.end())) {
         System.out.println("Frame name: "+fi.getName());
         System.out.println("number of outputs: "+fi.getNumOutputs());
         StdVectorString outNames = fi.getOutputNames();
         int sz = (int) outNames.size();
         for (int j =0; j <sz; j++) {
            System.out.println("output name:"+outNames.get(j));
         }
         fi.next();
    }

    // Ensure Joint has a default constructor.
    PinJoint pj = new PinJoint();

    m.addComponent(pj);

    // Change from ArrayPtrs<Object*> being non-const const.
    BodySet set = m.getBodySet();
    ObjectGroup group = set.getGroup(0);
    ArrayConstObjPtr members = group.getMembers();

    System.out.println("Test finished!");
    // TODO to cause test to fail: System.exit(-1);
  }
}
