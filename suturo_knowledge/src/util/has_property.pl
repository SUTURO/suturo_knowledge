%% The has_property module enables to ask for a favourite drink, 
%	if a person is known to us or to save the information name and favourite drink.

:- module(has_property,
	  [
		what_object(+,r),
		what_object_transitive(?,r),
		is_fragile(+),
		is_perishable(+),
		have_same_class(+,+),
		preorlo_check(r, -),
		grasp_pose(+,-),
		has_position(+,-),
		has_value(+,r,-),
		is_light_or_heavy(r,?),
		save_person_data(+,+,+,+,+),
		save_field(+,r,+),
		call_person_data(?,?,?,?,?),
		call_person_data_with_options(?,?,?,-,?,?),
		has_predefined_location(+, -),
		has_likely_location(+,-,-,-),
		has_likely_location_in_room(+,+,-,-),
		check_shelf_layers_for_frame(+,-),
		check_tables_for_frame(+, -),
		navigability(+, -)
	  ]).


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% go up all superclasses of an object till you find a superclass with property 'Fragility' 
%% is_fragile(r ObjName)
is_fragile(ObjName) :-
	triple(O,_, suturo:hasPredefinedName), 
	triple(O, owl:hasValue, ObjName), 
	triple(Object,_,O),  
	triple(Object, transitive(rdfs:'subClassOf'), X),
	triple(X, _, suturo:'Fragility').

	%transitivee(Object).

%transitivee(r Object)
transitivee(Object) :- 
	triple(Object, _, suturo:'Fragility').

%transitivee(r Object)
transitivee(Object) :-
	subclass_of(Object, X),
	transitivee(X).

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 
%% is_light_or_heavy(r Object, ?Weight)
%
% is an object heavy or light
is_light_or_heavy(ObjName, Weight):-
	what_object(ObjName, Object),
	triple(Object, transitive(rdfs:'subClassOf'), X),
	triple(X, _, suturo:hasWeight),
	triple(X, owl:hasValue, Weight).


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 
%% is_perishable(+ObjName)
% 
% ask if object is perishable
is_perishable(ObjName):-
	what_object(ObjName, Object),
	triple(Object, transitive(rdfs:'subClassOf'), X),
	triple(X, _, suturo:'Perishable').


%% preorlo_check(r, -)
preorlo_check(ObjName, Object):-
	what_object(ObjName, Object),
	triple(O,_, suturo:hasOriginLocation),
	triple(Object, owl:onProperty, _), 
	triple(Object,_,O),
	!.

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% what_object(+ ObjName, r Object)
%
% get the Object that has the predefined name "ObjName"
%what_object(+,-)
what_object(ObjName, Object) :-
	triple(O,_, suturo:hasPredefinedName),
	triple(O, owl:hasValue, ObjName), 
	triple(Object,_,O).

%% what_object_transitive(?ObjName, ?Class) is nondet.
%
% Get the Class that has the predefined name "ObjName",
% and all subclasses of that class
%what_object_transitive(+,-)
what_object_transitive(ObjName, Class) :-
	atom(ObjName),
	var(Class),
	!,
	kb_call((
		triple(O, owl:hasValue, ObjName),
		triple(O, owl:onProperty, suturo:hasPredefinedName),
		% O already has a value, so rdfs:subClassOf is transitive here.
		triple(Class, rdfs:subClassOf, O)
	)).

what_object_transitive(ObjName, Class) :-
	var(ObjName),
	atom(Class),
	!,
	kb_call((
		triple(Class, transitive(rdfs:subClassOf), O),
		triple(O, owl:hasValue, ObjName),
		triple(O, owl:onProperty, suturo:hasPredefinedName)
	)).

what_object_transitive(ObjName, Class) :-
	kb_call((
		triple(O, owl:onProperty, suturo:hasPredefinedName),
		triple(O, owl:hasValue, ObjName),
		% O already has a value, so rdfs:subClassOf is transitive here.
		triple(Class, rdfs:subClassOf, O)
	)).

%% have_same_class(+, +)
%
% check, if two objects belong to the same class
%have_same_class(+,+)
have_same_class(ObjName1, ObjName2) :-
	what_object(ObjName1, X),
	what_object(ObjName2, Y),
	subclass_of(X, Z),
	subclass_of(Y, Z),
	!.

% returns the grasping pose for toya to grasp a certain object
%grasp_pose(+,-)
grasp_pose(ObjName , Pose) :-
	what_object(ObjName, Object),
	triple(Object, transitive(rdfs:'subClassOf'), X),
	triple(X, _, suturo:hasGraspPose),
	triple(X, owl:hasValue, Pose).

%has_position(+,-)
has_position(ObjName, PoseStamped):-
	what_object(ObjName, Object), 
	triple(Object, transitive(rdfs:'subClassOf'), Q),
	triple(Q, _, suturo:hasPosition),
	triple(Q, owl:hasValue, Pose), 
	has_type(Plate, soma:'Plate'),
	object_pose(Plate, [Frame, [X,Y,Z] , Rotation]),
	( Pose == 'right'
	-> NewY is Y - 0.2, 
		PoseStamped = [Frame, [X,NewY,Z] , Rotation]
	; Pose == 'left'
	-> NewY is Y + 0.2, 
		PoseStamped = [Frame, [X,NewY,Z] , Rotation]
	; Pose == 'top_right'
	-> NewX is X - 0.2,  NewY is Y + 0.2,
		PoseStamped = [Frame, [NewX,NewY,Z] , Rotation]
	).
	
%has_value(+,r,-)	
has_value(ObjName, Property, Value) :-
	what_object(ObjName, Object),
	triple(Object, transitive(rdfs:'subClassOf'), X),
	triple(X, _, Property),
	triple(X, owl:hasValue, Value).



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Saves the data with an ID 
% If the string is empty and there was already an information for this ID it just updates the changes.
%% save_person_data(+ID, +Name, +Drink, +Interest, +Profession)
save_person_data(ID, Name, Drink, Interest, Profession):-
    save_field(ID, 'http://www.ease-crc.org/ont/SUTURO.owl#hasCustomerName', Name),
    save_drink(ID, Drink),
    save_field(ID, 'http://www.ease-crc.org/ont/SUTURO.owl#hasInterest', Interest),
    save_field(ID, 'http://www.ease-crc.org/ont/SUTURO.owl#hasProfession', Profession).

%save_field(+,r,+)
save_field(ID, Predicate, Value) :-
    (Value \= '' ->  
        kb_unproject(triple(ID, Predicate, _)), 
        kb_project(triple(ID, Predicate, Value)) 
    ; 
        \+ kb_call(holds(ID, Predicate, _)) -> 
        kb_project(triple(ID, Predicate, Value))
    ; 
        true).

% save_drink(+,+)
save_drink(ID, Drink) :-
    (Drink \= '' ->  
        what_object(Drink, OwlDrink),
        kb_unproject(triple(ID, suturo:hasFavouriteDrink, _)), 
        kb_project(triple(ID, suturo:hasFavouriteDrink, OwlDrink)) 
    ; 
        \+ kb_call(holds(ID, suturo:hasFavouriteDrink, _)) -> 
        kb_project(triple(ID, suturo:hasFavouriteDrink, Drink))  % If empty the original string is saved
    ; 
        true).


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% has_likely_location(+Object, -Location, -LocObject, -Pose)
% likly robocup locations for objects:
%   fruits --> billy shelf
% 	cutlery --> on the dishwasher  
has_likely_location(Object, Location, LocObj, Pose) :-
	(has_predefined_location(Object, Loc),
	 Loc = 'http://www.ease-crc.org/ont/SOMA.owl#Dishwasher' ->
		findall(T, is_table(T), Tables),
		check_tables_for_frame(Tables, Table),
		object_pose(Table, [map,X,Y]),
		Pose = [map,X,Y],
		Location = 'Dishwasher',
		LocObj = Table
	;
	(has_predefined_location(Object, Loc),
	 Loc = 'http://www.ease-crc.org/ont/SUTURO.owl#Shelf'  ->
		findall(S, is_shelf_layer(S), ShelfLayer),
		check_shelf_layers_for_frame(ShelfLayer, SLayer),
		object_pose(SLayer, [map,X,Y]),
		Pose = [map,X,Y],
		Location = 'Billy Shelf',
		LocObj = SLayer
	)
	;
	 true
	).

check_tables_for_frame([], Table) :- fail.  
check_tables_for_frame([T | Next], Table) :-
	object_pose(T, [Frame, _, _]),
	(Frame = 'iai_kitchen/dishwasher_table:d_table:table_center'->
		Table = T
	;   
		check_tables_for_frame(Next, Table)
	).

check_shelf_layers_for_frame([], ShelfLayer) :- fail.  
check_shelf_layers_for_frame([S | Next], ShelfLayer) :-
	object_pose(S, [Frame, _, _]),
	(Frame = 'iai_kitchen/shelf_billy_corridor:cabinet:shelf_floor_0'->
		ShelfLayer = S
	;   
		check_shelf_layers_for_frame(Next, ShelfLayer)
	).


% has_likely_location_in_room(+Object, +Room, -Location, -Pose)
% 
has_likely_location_in_room(Object, Room, Location, Pose) :-
	(has_likely_location(Object, LLocation, LocObj, LPose) -> 
		writeln(LPose),
		(check_position_inside_room(LPose, Room) ->
			Pose = LPose,
			writeln(Pose),
			Location = LLocation
		; 
			(has_type(Room, RoomType),
			RoomType = 'http://www.ease-crc.org/ont/SOMA.owl#Kitchen' ->
            	Pose = [map,[3.4, -2.01, 0.0], [0.0,0.0,1.0,0.0]],
            	Location = 'Alternative'

			;
				(has_type(Room, RoomType),
				RoomType = 'http://www.ease-crc.org/ont/SUTURO.owl#LivingRoom' ->
					Pose = [map,[4.36, 1.47, 0.0], [0.0,0.0,1.0,0.0]],
					Location = 'Alternative')
				)
		)
	).
     
navigability(Room, Navigability) :-
	has_type(Room, RoomType),
	triple(RoomType, transitive(rdfs:'subClassOf'), Type),
	triple(Type, _, suturo:hasNavigability),
	triple(Type, owl:hasValue, Navigability).


% has_predefined_location(+Object, -Location)
has_predefined_location(Object, Location) :-
	what_object(Object, Obj),
	triple(Obj, transitive(rdfs:'subClassOf'), Type),
	triple(Type, _, suturo:hasPredefinedLocation),
	triple(Type, owl:allValuesFrom, Location).

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% call_person_data(?ID, ?Name, ?Drink, ?Interest, ?Profession)
call_person_data(ID, Name, Drink, Interest, Profession):-
	kb_call(holds(ID, suturo:hasCustomerName, Name)), % ID + Name 
	kb_call(holds(ID, suturo:hasFavouriteDrink, Drink)), % Drink
	kb_call(holds(ID, suturo:hasInterest, Interest)), % Interest
	kb_call(holds(ID, suturo:hasProfession, Profession)). % Profession


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% call_person_data_with_options(?ID, ?Name, ?Drink, -Option, ?Interest, ?Profession)
call_person_data_with_options(ID, Name, Drink, Option, Interest, Profession) :-
	kb_call(holds(ID, suturo:hasCustomerName, Name)), % ID + Name 
	kb_call(holds(ID, suturo:hasFavouriteDrink, Drink)), % Drink
	kb_call(holds(ID, suturo:hasInterest, Interest)), % Interest
	kb_call(holds(ID, suturo:hasProfession, Profession)), % Profession
	findall(Options, (subclass_of(Options, Drink)), Option).