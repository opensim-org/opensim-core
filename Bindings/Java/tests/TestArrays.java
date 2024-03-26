import org.opensim.modeling.*;

class TestArrays {
    public static void TestArrays() {
        ArrayDouble arrayDouble = new ArrayDouble();

        // add, remove, getSize, index, make a copy
        double doubleVal = 123.4;
        arrayDouble.append(doubleVal);
        assert arrayDouble.get(0) == doubleVal;
        assert arrayDouble.size() == 1;
        // make a copy
        ArrayDouble arrayCopy = new ArrayDouble(arrayDouble);
        assert arrayCopy.get(0) == doubleVal;
        arrayDouble.remove(0);
        assert arrayCopy.getSize() == 1;
        assert arrayDouble.size() == 0;

        ArrayStr arrayStrings = new ArrayStr();
        String strVal = "AString";
        arrayStrings.append(strVal);
        assert arrayStrings.get(0).equals(strVal);
        assert arrayStrings.size() == 1;
        // make a copy
        ArrayStr arrayStrCopy = new ArrayStr(arrayStrings);
        assert arrayStrCopy.get(0).equals(strVal);
        arrayStrings.remove(0);
        assert arrayStrCopy.getSize() == 1;
        assert arrayStrings.size() == 0;
        // test set 0th element of an array
        ArrayStr mtArray =  new ArrayStr();
        assert mtArray.getSize() == 0;
        mtArray.append("1");
        assert mtArray.getSize() == 1;
        mtArray.set(0, "2");
        System.out.println("Size = " +mtArray.getSize());
        System.out.println("Zeroth entry " +mtArray.get(0));
        assert mtArray.getSize() == 1;

    }
  public static void main(String[] args) {
      TestArrays();
      
      System.out.println("Test finished!");
      // TODO to cause test to fail: System.exit(-1);
  }
}
