==========================
⚙️ StoreObjectInfoServer
==========================

--- creating new objects
----------------------------

.. list-table::
    :widths: 50

    * - planning passes the perceived obj through the StoreObjectInfoServer / actually an ActionServer
    * - in store_object_info_server:
    * - (1) Convert class name with ``re.sub(r"^[0-9-_]+|_","",name).capitalize()``
    * - (2) solutions <- knowrob.is_class("obj_class")
    * - (3) if not solutions: "obj_class" <- "Other"
    * - (4) get further info from perception
    * - (5) suturo_knowledge.is_legal_obj_position(...), .create_object(...)
    * - (6) .group_objects_at(...)
    * - (7) self._result <- success
