import org.opensim.modeling.*;

class TestVectors {
    public static void TestVectors() {
        Vec3 m = new Vec3(1, 2, 3);
        System.out.println(m.toString());
        m.scalarPlusEq(3.);
        assert(m.get(0)==4);
        m.scalarEq(5.);
        assert(m.get(1)==5);
        m.scalarPlusEq(4.);
        System.out.println(m.toString());
        m.scalarMinusEq(1.);
        Vec3 n = new Vec3(8);
        System.out.println(n.toString());
        n.scalarTimesEq(1000.0);
        assert(n.get(1)==8000);
        assert(n.get(2)==8000);
        n.scalarDivideEq(1000.0);
        // Check m & n are same = {8, 8, 8}
        for (int i=0; i<3; i++)
           assert(m.get(i) == n.get(i));
    }
  public static void main(String[] args) {
      TestVectors();
      
      System.out.println("Test finished!");
      // TODO to cause test to fail: System.exit(-1);
  }
}
