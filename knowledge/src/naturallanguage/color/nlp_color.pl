:- module(nlp_color,[
    find_object_of_color_in_list/4,
    determine_color_name/2,
    set_nlp_color/1,
    object_color_hsv/2]).

rdf_meta.

%% Load the c++ library
:- use_foreign_library('libnlp_color.so').

:- ros_package_iri(knowledge, 'package://knowledge/owl/color.owl').

:- tripledb_load(
	'package://knowledge/owl/color.owl',
	[ namespace(suturo_color, 'http://www.semanticweb.org/suturo/ontologies/2021/6/color#')
	]).

:- rdf_db:rdf_register_ns(suturo_color, 'http://www.semanticweb.org/suturo/ontologies/2021/6/color#', [keep(true)]).


set_nlp_color(ObjID):-
    object_color_rgb(ObjID, RGB),
    rgb_to_hsv(RGB, HSV),
    tell(object_color_hsv(ObjID,HSV)),
    determine_color_name(ObjID,Color),
    tell(triple(ObjID, suturo_color:'has_color', Color)).


% + ListOfObjects, + TargetColor, - Object, - Conf
find_object_of_color_in_list(ListOfObjects, TargetColor, Object, Conf):-
    findall(Tuples, % Some Colors have more then one CVD (red)
    (
        ask((triple(TargetColor,suturo_color:'color_likelihood', CVD),
            triple(CVD, suturo_color:'hue_mean',Hue_Mean),
            triple(CVD, suturo_color:'hue_standard_deviation',Hue_SD),
            triple(CVD, suturo_color:'hue_multiplier',Hue_Multi),

            triple(CVD, suturo_color:'saturation_mean',Sat_Mean),
            triple(CVD, suturo_color:'saturation_standard_deviation',Sat_SD),
            triple(CVD, suturo_color:'saturation_multiplier',Sat_Multi),

            triple(CVD, suturo_color:'value_mean',Val_Mean),
            triple(CVD, suturo_color:'value_standard_deviation',Val_SD),
            triple(CVD, suturo_color:'value_multiplier',Val_Multi)
            )),

        findall(Tuple,
        (
            member(Obj, ListOfObjects),
            ask(object_color_hsv(Obj, [ObjH,ObjS,ObjV])),
            three_normal_distribution_propability_multi([ObjH,ObjS,ObjV],[Hue_Mean,Sat_Mean,Val_Mean],[Hue_SD,Sat_SD,Val_SD],[Hue_Multi,Sat_Multi,Val_Multi],Prop),
            Tuple = [Prop, Obj] % The propability has to go first for max_member to use it
        ),
        Tuples) % Contains the Conf and the Object for each Object
    ), Tupless), % Contains a list for each CVD containing the Conf and the Object for each Object
    my_flatten(Tupless, FlattenTuples),
    max_member([Conf,Object], FlattenTuples).





%%% HSV to ColorClass

determine_color_name(ObjID,Color):-
    not(
        ObjID = [_,_,_]
    ),
    ask(object_color_hsv(ObjID,HSV)),
    determine_color_name(HSV,Color).



determine_color_name([ObjH,ObjS,ObjV],Color):-
    determine_color_name_with_list([ObjH,ObjS,ObjV],Tuples),
    max_member([_,ColorValue], Tuples),
    triple(Color,suturo_color:'color_likelihood', ColorValue).


determine_color_name_with_list([ObjH,ObjS,ObjV],Tuples):-
    findall(ColorValueDistribution,
        subclass_of(ColorValueDistribution, suturo_color:'hsv_values'),
    ColorValueDistributions),

    findall(Tuple,
    (
        member(CVD, ColorValueDistributions),
        triple(CVD, suturo_color:'hue_mean',Hue_Mean),
        triple(CVD, suturo_color:'hue_standard_deviation',Hue_SD),
        triple(CVD, suturo_color:'hue_multiplier',Hue_Multi),

        triple(CVD, suturo_color:'saturation_mean',Sat_Mean),
        triple(CVD, suturo_color:'saturation_standard_deviation',Sat_SD),
        triple(CVD, suturo_color:'saturation_multiplier',Sat_Multi),

        triple(CVD, suturo_color:'value_mean',Val_Mean),
        triple(CVD, suturo_color:'value_standard_deviation',Val_SD),
        triple(CVD, suturo_color:'value_multiplier',Val_Multi),

        three_normal_distribution_propability_multi([ObjH,ObjS,ObjV],[Hue_Mean,Sat_Mean,Val_Mean],[Hue_SD,Sat_SD,Val_SD],[Hue_Multi,Sat_Multi,Val_Multi],Prop),
        Tuple = [Prop, CVD]
    ),
    Tuples).




% Flatten only once
my_flatten([], []).
my_flatten([A|B],L) :- is_list(A), my_flatten(B,B1), !, append(A,B1,L).
my_flatten([A|B],[A|B1]) :- my_flatten(B,B1).






%%%%%%%%%%%%%%%%%%%%%%%%%%%% Store HSV Values


%% object_color_hsv(?Obj, ?Col) is nondet.
%
% True if Col is the main color of Obj.
% Col is encoded as as [float hue, saturation, value], hue 0-360, saturation,value of 0-1.
%
% @param Obj object resource
% @param Col hsv color data
% 
object_color_hsv(Obj,[H,S,V]) ?>
  holds(Obj,soma:hasColor,Color),
  holds(Color,dul:hasRegion,Region),
  holds(Region,soma:hasHSVValue,ColorValue),
  { atomic_list_concat(ColorList, ',', ColorValue),
    maplist(atom_number, ColorList, [H,S,V]) },
  { ! }.

object_color_hsv(Obj, [H,S,V]) ?>
  holds(Obj,soma:hasHSVValue,ColorValue),
  { atomic_list_concat(ColorList, ',', ColorValue),
    maplist(atom_number, ColorList, [H,S,V]) },
  { ! }.
  
object_color_hsv(Obj,[H,S,V]) +>
  % get the color quality
  { holds(Obj,soma:hasColor,Color),
    atomic_list_concat([H,S,V],',', ColorValue)},
  % create a new region
  { universal_scope(US),
    tell([ has_type(Region,soma:'ColorRegion'),
           holds(Region,soma:hasHSVValue, ColorValue)
         ],[[],US])
  },
  % update the region of the color quality
  update(holds(Color,dul:hasRegion,Region)).
