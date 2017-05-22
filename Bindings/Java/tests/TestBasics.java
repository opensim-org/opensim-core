import org.opensim.modeling.*;

class TestBasics {
    public static void testBasics() {
        Model m = new Model();
        m.finalizeFromProperties();
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
        set.addGroup("upper_extremity");
        ObjectGroup group = set.getGroup(0);
        ArrayConstObjPtr members = group.getMembers();
    }

    public static void testMuscleList() {
        Model model = new Model();

        Thelen2003Muscle thelenMuscle =
            new Thelen2003Muscle("Darryl", 1, 0.5, 0.5, 0);
        Millard2012EquilibriumMuscle millardMuscle =
            new Millard2012EquilibriumMuscle("Matt", 1, 0.5, 0.5, 0);

        model.addComponent(thelenMuscle);
        model.addComponent(millardMuscle);

        // Total number of muscles must be 2 during iteration.
        {
            MuscleList muscleList = model.getMuscleList();
            int count = 0;
            MuscleIterator iter = muscleList.begin();
            while(!iter.equals(muscleList.end())) {
                assert iter.__ref__() instanceof Muscle;
                
                iter.next();
                ++count;
            }
            assert count == 2;
        }

        // There is exactly 1 Thelen2003Muscle in the list during iteration.
        {
            Thelen2003MuscleList thelenMuscleList =
                model.getThelen2003MuscleList();
            int count = 0;
            Thelen2003MuscleIterator iter = thelenMuscleList.begin();
            while(!iter.equals(thelenMuscleList.end())) {
                assert iter.__ref__() instanceof Thelen2003Muscle;
                
                iter.next();
                ++count;
            }
            assert count == 1;
        }

        // There is exactly 1 Millard2012EquilibriumMuscle in the list
        // during iteration.
        {
            Millard2012EquilibriumMuscleList millardMuscleList =
                model.getMillard2012EquilibriumMuscleList();
            int count = 0;
            Millard2012EquilibriumMuscleIterator iter =
                millardMuscleList.begin();
            while(!iter.equals(millardMuscleList.end())) {
                assert iter.__ref__() instanceof Millard2012EquilibriumMuscle;
                
                iter.next();
                ++count;
            }
            assert count == 1;
        }
    }

    public static void testToyReflexController() {
        ToyReflexController controller = new ToyReflexController();
    }
        
  public static void main(String[] args) {
      testBasics();
      testMuscleList();
      testToyReflexController();
      
      System.out.println("Test finished!");
      // TODO to cause test to fail: System.exit(-1);
  }
}
