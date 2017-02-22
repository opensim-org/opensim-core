import org.opensim.modeling.*;

class TestIterators {
  public static void main(String[] args) {
    Model model = new Model();

    // Iterate through 
    model.printSubcomponentInfo();
    ComponentsList componentsList = model.getComponentsList();
    ComponentIterator compIter = componentsList.begin();
    int countComponents = 0;
    while(!compIter.equals(componentsList.end())){
        countComponents++;
        System.out.println("Next component:"+compIter.getName()+" type:"+compIter.getConcreteClassName());
        compIter.next();
    }
    // Although the following Lists and Sets are mostly empty
    // this tests that the classes were wrapped properly otherwise
    // this code will not compile.
    BodyList bodyList = model.getBodyList();
    MuscleList muscleList = model.getMuscleList();
    MuscleIterator muscleIter = muscleList.begin();
    while(!muscleIter.equals(muscleList.end())){
        Muscle nextMuscle = muscleIter.__deref__();
        System.out.println("Next muscle:"+nextMuscle.getName());
        muscleIter.next();
    }
    // Access lists used in GUI
    ForceSet fset = model.getForceSet();        int nforces = fset.getSize();
    SetMuscles musset = model.getMuscles();     int nmuscles = musset.getSize();
    SetActuators actset = model.getActuators(); int nacts = actset.getSize();
    ProbeSet pset = model.getProbeSet();        int nprobes = pset.getSize();
    ContactGeometrySet cgset = model.getContactGeometrySet();
    AnalysisSet aset = model.getAnalysisSet();  int nanalysis = aset.getSize();
    
    System.out.println("Test finished!");
    // TODO to cause test to fail: System.exit(-1);
  }
}
