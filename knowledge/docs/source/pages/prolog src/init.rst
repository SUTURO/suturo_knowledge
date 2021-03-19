===================
⚙️ init.pl
===================

loading owl files
------------------------
.. code-block:: prolog

    :- tripledb_load(
        'http://www.ontologydesignpatterns.org/ont/dul/DUL.owl',
        [ namespace(dul)
        ]).
    :- tripledb_load(
        'http://knowrob.org/kb/knowrob.owl',
        [ namespace(knowrob)
        ]).
    :- tripledb_load('package://knowledge/owl/objects.owl',
        [ namespace(hsr_objects)
        ]).
    :- tripledb_load(
        'http://knowrob.org/kb/URDF.owl',
        [ namespace(urdf, 'http://knowrob.org/kb/urdf.owl#')
        ]).