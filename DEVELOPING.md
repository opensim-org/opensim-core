Guidelines for Developers of OpenSim-Core
===========================================
OpenSim is a community resource that is housed in the OpenSim-Core repository.

This specific document addresses issues that come up during development and maintenance of the OpenSim code base, that may not be relevant to end users because it's not in release code yet or of historical relevance only but would be 
valuable to new developers or advanced users. For general contribution guidelines please consult [CONTRIBUTING.md](https://github.com/opensim-org/opensim-core/blob/master/CONTRIBUTING.md)

Contents:

- [Backward Compatibility](#backward-compatibility-of-file-formats)


Backward Compatibility of File Formats
--------------------------------------
OpenSim stores models and analysis-tools/objects in xml files (with extension .osim or .xml). Since the first release of OpenSim in 2007, these files have mostly survived many major format changes which is quite important considering that thousands of these files are around both in earlier OpenSim distributions and also in models created by users and/or collaborators. This section describes how backward compatibility is maintained so that if a format change is introduced, users do not lose valuable work and/or models in the process. In what follows we use the term serialization to mean writing and deserialization to mean reading. 

One key piece of information that is kept with each file is a version number (usually in the header section e.g. `<OpenSimDocument Version="NNNNN">`. The same is true for .sto files that contain a header as well, though we will not discuss .sto files here. The version number encodes what the code was expecting the format to be when the file was produced/written. Keeping this number consistent with the content of the file is the fundamental invariant that needs to be maintained. In code, the version number is maintained in XMLDocument.cpp and is updated as version numbers are incremented. The actual numbers do not matter much, as long as they are monotonically increasing. OpenSim 3.2 shipped with OpenSimDocument Version number 30000. OpenSim 4.0 development started at version 30500 and will ship with 40000).

Deserialization of Objects and their Properties (primitives that are read-from/written-to files) is mostly transparent to users/developers. Once you use the property macros (e.g. ` OpenSim_DECLARE_UNNAMED_PROPERTY`) in the header and perform the necessary wiring/calls, these properties know how to serialize themselves to XML elements and back. This serialization/deserialization, however, assumes a fixed XML layout/schema. If this assumption fails (because of a property name change, layout change or addition of new properties) then deserialization will not work out of the box and the developer making the change is responsible for changing the version number and the deserialization code to be in sync with the latest code. This is very important since if the developer does not update the version number then the code and the XML files will not be in sync and it will be very hard to go back to find out what changes should have been made. 

Keep in mind that deserialization methods are practically called during the construction of the top level object/Model so typically you cannot rely on having any member variables or pointers populated unless done by default constructors, instead deserialization methods operate on the XML structure only. This helps isolate the backward compatibility code from the rest of the modeling/simulation or initialization functions.

Deserialization works by making the following calls:
 1. A user constructs an object that is a subclass of `OpenSim::Object` by using a constructor that takes the file name of a .osim or .xml file (only some classes have such a constructor). The construction call sequence ends with calling `OpenSim::Object`'s constructor with the file name as an argument. This constructor invokes code that parses the XML file.
 2. The parsing code looks for XML tags that correspond to names of subclasses of `OpenSim::Object`; when it finds one of the them it performs the next 2 steps.
 3. An Object of the appropriate type is instantiated from the registry of available types using a lookup by the XML tag.
 4. The method `updateFromXMLNode(SimTK::Xml::Element& aNode, int versionNumber) ` is invoked on the object and is passed the XML element corresponding to the object along with the version number from the XML document/file. That’s exactly the hook to deserialization that developers can use to correct for deserialization changes by manipulating the passed in `aNode` to match what the latest code expects. 


If an object has never undergone format changes then it should not implement the method `updateFromXMLNode()` altogether.
If the function needs to be implemented, it would have the following form:
```cpp

void XXX::updateFromXMLNode(SimTK::Xml::Element& aNode, int versionNumber)
{
       // XMLDocument::getLatestVersion() is a number that changes with upgrades
       // Guarding with this condition makes sure that files already converted 
       // do not get penalized or converted again.
       if ( versionNumber < XMLDocument::getLatestVersion()) {
              if (versionNumber <= 20301) {
		         // convert aNode from version 20301 or prior to the next version
		          ……
              }
              if (versionNumber < 30500) {
	             // Convert versions before 30500 
              }
        }
        // At this point of the code, aNode is on the latest XML format
        // Call base class, now assuming aNode has been corrected for current version
        // This call will end up being made on Object, which will do the actual population of Property values
        Super::updateFromXMLNode(aNode, versionNumber);
}
```
