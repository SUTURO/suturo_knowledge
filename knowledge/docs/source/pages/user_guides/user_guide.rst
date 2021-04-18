========================
️👨‍💻 User Guide
========================

Setting up the knowledge package
-----------------------------------
* Kommt noch

Setting up the knowledge ws
--------------------------------
* Kommt noch

--- planning o)--(o knowledge
----------------------------------------
* General interaction:
    * (1) Planning gives to Knowledge what is perceived --> ActionServer (knowledge_insertion_client?)
    * (2) Knowledge does its filtering
    * (wait)
    * (3) Planning wants to know something

        * (3.1) Planning gets it via knowledge_client.lsp / knowledge_insertion_client.lsp

* manipulation-functions:
    * (0) Knowledge weiß wo etwas steht
    * (1) Planning fragt nach next_graspable_object (Eg. [0]=pringles_obj_red)
    * (2) Planning frag nach `grasp_mode`
    * (3) Object is being grasped
    * (4) Planning fragt nach `target_surface`

.. code-block:: prolog

    % (4) Planning fragt nach target surface
    target_surface()
    % returns (shelf,0), (floor,1)
    -> object_goal_pose() % returns xyz
    -> prolog_target_pose()