import org.opensim.modeling.*;

class TestBasics {
  public static void main(String[] args) {
    Model m = new Model();
    m.print("empty_model.osim");
    System.out.println("Test finished!");
    // TODO to cause test to fail: System.exit(-1);
  }
}
