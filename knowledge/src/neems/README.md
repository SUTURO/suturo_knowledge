The neem directory
================

This is a collection of modules for the purpose of generating NEEMs & extracting valuable information from them.

####1. Logge auf höchsten Level eine Episode
    tell(is_episode(Ep))
    
####2. Logge immer wenn eine Action anfängt die Action
    tell(is_action(Act))
    % Wichtig: Dies machst du, damit du Subactions ihre Parentaction übergeben kannst (siehe Schritt 3), denn Subactions enden ja immer vor der Parentaction.


####3. Logger immer wenn eine Action endet die Action
    % Logge Start und Endzeit der Action. Start und End müssen Unix Time sein \
    tell(occurs(Act) during [Start,End])
   
    % Logge die Parentaction der aktuellen Action 
    tell(has_subevent(ParentAct,Act))
    
    % Logge die Task die ausgeführt wurde 
    has_type(Tsk,soma:'Grasping')
    executes_task(Act, Tsk) 
      
    % Tasks klassifizieren WAS in einer Episode passiert ist und sind daher wichtig damit in der Timeline steht was ausgeführt wurde.
    % Die generellste Task ist soma:'PhysicalTask' (Hinweis: Die Actions die für euch Relevant sein könnten sind wohl in SOMA-HOME.owl definiert
    % https://ease-crc.github.io/soma/owl/current/SOMA-HOME.owl). Aber Grundsätzlich solltest du dieses vermeiden, um so spezifisch wie möglich zu sein was ausgeführt wurde.
    % Wenn du keine Definition in SOMA-HOME findest müsstest du eine eigene Task definieren (als Subklasse von PhysicalTask).

####4. Visualizable NEEMs

Necessary data:

- URDF file of participating agents and location (e.g. kitchen, lab)
- OWL file of participating agents and location (e.g. kitchen, lab). Can be generated from the URDF file using
- Meshes of participating agents and location (e.g. kitchen, lab). They should correspond to the material in
- TF Log (created by Knowrob)

Necessary steps when starting the logging:

- Load the OWL Files: tripledb_load(\'package://knowrob/owl/robots/PR2.owl\')
- Load the URDF Files and link them to the corresponding robot/location from the OWL File: urdf_load(\'http://knowrob.org/kb/PR2.owl#PR2_0\', \'package://knowrob/urdf/pr2.urdf\', [load_rdf])
- Create the episode: tell(is_episode(Episode))
- Save some frame data for the episode: is_setting_for(Episode,\'http://knowrob.org/kb/PR2.owl#PR2_0\')

Logging highest action:

- Create action: tell(is_action(Act))
- Add the task that is executed during the action: tell([has_type(Tsk,soma:\'Transporting\'),executes_task(Act,Tsk)])
- Add start and endtime (as unix timestamps) to action: tell(occur(Act) during [Start, End])
- Link the action to the created Episode: tell(is_setting_for(Episode,Act))

Logging all other actions:

- Create action: tell(is_action(Act))
- Add the task that is executed during the action: tell([has_type(Tsk,soma:\'Transporting\'),executes_task(Act,Tsk)])
- Add start and endtime (as unix timestamps) to action: tell(occur(Act) during [Start, End])
- Link the action to it's parent-action: tell(has_subevent(ParentAct,Act))
