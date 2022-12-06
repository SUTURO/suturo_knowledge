==========================
⚙️ MarkerObjectstate
==========================

This Script republishes all Markers from ``/visualization_marker_array`` to ``/object_state``.

TODO: I don't know why it uses the visualization markers and why this isn't done directly from prolog code. Before changing this i will search for a reason.

Manipulation uses the ``/object_state`` topic in the ``object_state_listener`` script to add the Objects into giskard.
