:- module(nlp_color,[
    set_nlp_color/1]).

rdf_meta.

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
    tell(object_color_hsv(ObjID,[H,S,V])),
    determine_color_name(ObjID,Color),
    tell(triple(ObjID, suturo_color:'has_color', Color)).






determine_color_name(ObjID,Color):-
    ask(object_color_hsv(ObjID,[ObjH,ObjS,ObjV])),
    determine_color_name([_,ObjSat,ObjVal],Color).


determine_color_name([_,ObjSat,ObjVal],Color):-
    ObjSat < 0.3,
    ObjVal < 0.6,
    Color = 'http://www.semanticweb.org/suturo/ontologies/2021/6/color#grey',
    !.

determine_color_name([_,ObjSat,ObjVal],Color):-
    ObjSat < 0.12,
    Color = 'http://www.semanticweb.org/suturo/ontologies/2021/6/color#white',
    !.

determine_color_name([_,ObjSat,ObjVal],Color):-
    ObjVal < 0.3,
    Color = 'http://www.semanticweb.org/suturo/ontologies/2021/6/color#black',
    !.

determine_color_name([ObjH,_,_],Color):-

    findall(ColorValueDistribution,
        subclass_of(ColorValueDistribution, suturo_color:'hsv_values'),
    ColorValueDistributions),

    findall(Tuple,
    (
        member(CVD, ColorValueDistributions),
        triple(CVD, suturo_color:'hue',Hue),
        triple(CVD,suturo_color:'standard_deviation',SD),
        triple(CVD,suturo_color:'multiplier',Multi),
        normal_distribution_propability(Hue,SD,ObjH,Prop),
        PropM is Prop * Multi,
        Tuple = [PropM, CVD] % PropM first so max_member uses that
    ),
    Tuples),
    max_member([_,ColorValue], Tuples),
    triple(Color,suturo_color:'color_likelihood', ColorValue).



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
