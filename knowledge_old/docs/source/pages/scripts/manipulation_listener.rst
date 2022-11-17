==========================
⚙️ ManipulationListener
==========================

Overview
----------
.. list-table::
    :widths: 50

    * - Subscribes to **object_in_gripper**
    * - Gets info about grasped object and evaluates it via *attach_object_to_gripper/1*
    * - Problem: In the most recent pick-and-place action, the placed object gets passed a not normalised quaternion