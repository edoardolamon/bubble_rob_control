# API reference

The public C++ API of the package is the hardware plugin. ros2_control loads it by the name
`bubble_rob_control/BubbleRobSystem` (see `bubble_rob_control.xml`); you do not call it
directly. The nodes and their topics and parameters are described on the [README page](readme.md).

```{doxygenclass} bubble_rob_control::BubbleRobSystem
:members:
```
