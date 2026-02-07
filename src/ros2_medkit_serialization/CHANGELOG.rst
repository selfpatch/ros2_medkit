^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package ros2_medkit_serialization
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
* Initial rosdistro release
* Runtime JSON to ROS 2 message serialization using vendored dynmsg C++ API
* TypeCache - thread-safe caching of ROS type introspection data with shared_mutex
  for read concurrency
* JsonSerializer - bidirectional JSON <-> ROS message conversion via dynmsg YAML bridge,
  including CDR serialization/deserialization for GenericClient/GenericSubscription
* ServiceActionTypes - helper utilities for resolving service and action internal types
  (request/response, goal/result/feedback)
* SerializationError exception hierarchy for structured error handling
* Contributors: Bartosz Burda
