==========================
⚙️ StoreObjectInfoServer
==========================

--- creating new objects
----------------------------

.. list-table::
    :widths: 50

    * - planning passes the perceived obj through the StoreObjectInfoServer / actually an ActionServer
    * - in store_object_info_server:
    * - (1) solutions <- knowrob.is_class("obj_class")
    * - (2) if not solutions: "obj_class" <- "Other"
    * - (3) get further info from perception
    * - (4) suturo_knowledge.is_legal_obj_position(...), .create_object(...)
    * - (5) .group_objects_at(...)
    * - (6) self._result <- success