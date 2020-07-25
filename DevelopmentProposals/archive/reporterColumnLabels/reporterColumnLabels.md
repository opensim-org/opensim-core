## Project: Improve column labels used by TableReporter
***OpenSim Development Proposal***

### Overview
This proposal concerns the column labels used by the ConsoleReporter and TableReporter classes.
For convenience, these two classes are collectively referred to as "the Reporters" in this proposal.
(There is also a StatesTrajectoryReporter, but it is unaffected by this proposal.)
For background:
- See [issue #1034](https://github.com/opensim-org/opensim-core/issues/1034)
  (was fixed by [PR #1071](https://github.com/opensim-org/opensim-core/pull/1071)).
- See [issue #1221](https://github.com/opensim-org/opensim-core/issues/1221).

**Current behavior**

`Input::connect()` has the following two signatures:
```cpp
void connect(const AbstractOutput& output,   const std::string& annotation = "");
void connect(const AbstractChannel& channel, const std::string& annotation = "");
```
For simplicity and without loss of generality, we shall assume the user is connecting an Output to the Reporter's Input.
(There may be implementation differences for Outputs and Channels, depending on the design, but these differences are likely to be small.)
The `annotation` argument is optional; if provided, the ConsoleReporter will use the specified string as the column label and the Output's name otherwise;
the TableReporter always uses the full path name as the column label.

The annotation is currently stored when `Input::connect()` is called.
If the user provided the optional second argument when calling `connect()`, then this string is stored as the annotation;
otherwise, a default annotation is stored.
The default annotation is currently the name of the Output (e.g., "value", in the case of a Coordinate).
With the current design, it is impossible to determine whether the stored annotation was specified by the user or set by default.
The user could even have specified a string that happens to be identical to the default.

Currently, the ConsoleReporter always uses the annotation as the column label.
This behavior appears to be appropriate for the ConsoleReporter because Output names tend to be short.
Thus, if no annotation is specified, a line like
```cpp
reporter->updInput("inputs").connect(elbow->getCoordinate(PinJoint::Coord::RotationZ).getOutput("value"));
```
uses a default column label of "value"; if desired, the user can simply specify an annotation and re-run his/her code.
There will likely be only a few Outputs connected to (the Input of) a ConsoleReporter because of the limited width of the typical console.
(If too many Outputs are connected, the data corresponding to each time point will wrap over multiple lines and will be difficult to read.)
Thus, providing an annotation if the default column label is unsatisfactory (e.g., "value" for all Coordinates) is not likely to be burdensome for users.

The TableReporter always uses the (current) full path name as the column label.
TableReporters are likely to report more Outputs than ConsoleReporters because, unlike the console,
there is no detriment to writing a table with many columns (aside, of course, from memory issues for *very* large tables).
As such, it is more important for the TableReporter to use meaningful default column labels.
If reporting the value of all Coordinates in a Model, for example, it would be unhelpful for all column labels to be "value",
and forcing the user to specify an annotation for each Output may not be practical.

### Requirements
- The ConsoleReporter and TableReporter should both use the annotation for the column label if an annotation has been provided by the user.
- The ConsoleReporter should use a default column label that is likely to be as meaningful as possible while still being short.
  Although not strictly enforced, the Output names tend to be short (and are automatically truncated to 12 characters if they are too long).
- The TableReporter should use a default column name that is very likely to be meaningful.

### Architecture
The selected design involves four main changes:

1. Rename "annotation" to "alias" (e.g., `AbstractInput::getAnnotation()` becomes `AbstractInput::getAlias()`).
2. If the user provides the optional second argument when connecting an Output to an Input, store the string as the Input's alias; otherwise, the alias remains null.
3. Create `getShortLabel()` and `getLongLabel()` methods on Input:
  - `getShortLabel()` will be called by ConsoleReporter and will return `isNull(alias) ? output_name : alias`.
  - `getLongLabel()` will be called by TableReporter and will return `isNull(alias) ? full_path_name : alias`.
4. Create `setAlias()` methods analogous to the current `getAnnotation()` methods so the user can change the alias after (but not before) connecting.

Several other designs would satisfy the requirements; four rejected proposals are listed below.
All these designs could benefit from a `setAnnotation()` method (e.g., if a fully connected model was loaded from file).

**Rejected Design 1**

Store an empty string in `_annotations` if no annotation was provided when `Input::connect()` was called.
When writing the column label, use the `_annotations` value if it is not empty; otherwise,
the ConsoleReporter will use the Output's name and the TableReporter will use the Output's full path name.
This design will change the current behavior.
Currently, the name of the Output can differ from the column label if the name is changed after `Input::connect()` has been called.
It seems natural to generate a default column label when reporting rather than when connecting,
so there is presumably a good reason for wanting to generate a default when `Input::connect()` is called
(though always using the Output's full path name in the TableReporter is inconsistent).

**Rejected Design 2**

To retain the current behavior of storing the Output's name at connect time,
the `_annotations` member variable can be changed from a `std::vector<std::string>` to a `std::vector< std::pair<bool, std::string> >`.
The boolean flag would indicate whether the string was the annotation provided by the user or the name of the Output when `Input::connect()` was called.
The column label would then be determined at report time as follows:
```
if (ConsoleReporter)
    column_label = annotation
else //TableReporter
    column_label = annotation_was_provided_by_user ? annotation : Output.getFullPathName()
end
```

**Rejected Design 3**

This design also retains the current behavior of storing the Output's name at connect time.
The `_annotation` would be stored in `Input::connect()` as follows:
```
if (user provided an annotation)
    _annotation = string provided by user
else
    _annotation = ConsoleReporter ? Output.getName() : Output.getFullPathName()
end
```

**Rejected Design 4**

This design also retains the current behavior of storing the Output's name at connect time,
and also allows the user to specify a preference for whether to use short or long default column labels.
The `_annotations` member variable would be changed from a `std::vector<std::string>` to a `std::vector< std::pair<std::string, std::string> >`
and would be set at connect time as follows:
```
if (user provided an annotation)
    _annotation.first  = user's string
    _annotation.second = user's string
else
    _annotation.first  = Output.getName()
    _annotation.second = Output.getFullPathName()
```
The ConsoleReporter would use `_annotation.first` and the TableReporter would use `_annotation.second`,
but it would also be possible to include a flag that allows the user to switch between short and long default column labels at report time.

### Interfaces
The selected design has no effect on the interface.

### Bindings
There are no known implications for Python and Java bindings.

### Backwards Compatibility
The ConsoleReporter and TableReporter classes are new to 4.0, so there are no backwards compatibility issues.

### Lifecycle and Ownership / Memory Management
No new objects are being proposed.

### Performance Considerations
No substantial changes in performance are expected.

### Tests
There do not appear to be any tests for ConsoleReporter or TableReporter.
Tests will be created to ensure column labels are correct.

### Potential hurdles or roadblocks
There may be additional considerations related to how annotations are serialized.
Connectee addresses are currently serialized using full path names suffixed with `(annotation)`.
It may be prudent to switch to a more explicit layout to separate the path, output, channel, and annotation fields:
```
<ConnecteePath>
    <path_to_Component>uniped/thigh/kneeJoint/flexion</path_to_Component>
    <output_name>value</output_name>
    <channel_name></channel_name>
    <annotation>flexionAngle</annotation>
</ConnecteePath>
```

### Pull Requests
The selected design was implemented in [PR #1279](https://github.com/opensim-org/opensim-core/pull/1279).
