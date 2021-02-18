===========================
⚙️ object_state.pl
===========================

# --- structure
-------------------
* module exports
* rdf_db registers
* rdf_meta
* hsr_existing_objects/1
* hsr_forget_object/1
* forget_objects_on_surface/1
* place_object/1
* create_object/9
* random_id_gen/2
* validate_confidence => 5x predicates
* object type handling/3
* set_dimension_semantics/4 => 6x predicates
* set_color_semantics/3 => ca. 10x predicates

create_object/9
-----------------------
* called in use case 1
* instantiates perceived object in our knowledge base
* params: (testweise)
    * PerceivedObjectType => 'link_to_owl#Object'
    * PercTypeConf => 1
    * Transform => ['map', [1,1,1], [0, 0, 0, 1]]
    * [W,D,H] => [0.5, 0.5, 0.5]
    * Shape => 'box'
    * PercShapeConf => 1
    * RGB => [...]
    * PercColorConf => 1
    * ObjID <= the saved value
* when "tell(is_at(ObjID,Transform))" is called, then rosprolog_node published it

.. code-block:: prolog

    ?- create_object('http://www.semanticweb.org/suturo/ontologies/2020/3/objects#PringlesOriginals', 1,  ['map', [1,1,1], [0, 0, 0, 0]], [0.5, 0.5, 0.5], 'box',1, [0,0,255], 1, ObjID)


hsr_existing_objects/1
------------------------
* Returns a list of all existing Objects
* Must be renamed
