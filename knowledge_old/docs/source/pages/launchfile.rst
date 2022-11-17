=======================
⚙️ launchfile
=======================

--- launching
---------------
.. list-table:: launching the urdf
    :widths: 50

    * - there are 2 cases for the ParamServer.hsrb_lab:
    * - case (A) : standalone = false / default
    * - => here the ParamServer.hsrb_lab is not set at all, but this task is rather passed to the suturo_bringup
    * - => ParamServer.hsrb_lab <- suturo_bringup.launch (some xacro from suturo_resources)
    * - case (B) : standalone = true
    * - => here the ParamServer.hsrb_lab is set directly from a Node, which is started in this launch:
    * - => ParamServer.hsrb_lab <- knowledge.launch.suturo_urdf_publisher default=gazebo_E1
    * - the ParamServer.param_to_load_URDF_from is:
    * - => ParamServer.param_to_load_URDF_from <- knowledge.launch(<urdf_param, default="hsrb_lab">)

.. list-table:: setting launchfiles
    :widths: 50

    * - <param name="ParamServer.name" value="$(arg from somewhere above)">
    * - <param name="ParamServer.name" echo="$(some file in filesystem)">

--- suturo_urdf_publisher
---------------------------
.. list-table::
    :widths: 50

    * - sets ParamServer.hsrb_lab via cmd
    * - starts Node: JointStatePublisher veröffentlich die Standartwerte der Gelenke
    * - starts Node: RobotStatePublisher nimmt die JointStates, URDF und veröffentlicht den TF Status des Raums