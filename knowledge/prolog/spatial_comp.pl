:- module(spatial_comp,
    [
      hsr_lookup_transform/4,
      hsr_existing_object_at/3
    ]).

:- rdf_meta
    hsr_lookup_transform(r,r,?,?),
    hsr_existing_object_at(r,r,?).



hsr_lookup_transform(SourceFrame, TargetFrame, Translation, Rotation) :-
    tf_lookup_transform(SourceFrame, TargetFrame, pose(Translation,Rotation)).
%    tf_lookup_transform(SourceFrame, TargetFrame, PoseTerm),
%    owl_instance_from_class(knowrob:'Pose', [pose=PoseTerm], Pose),
%    transform_data(Pose,(Translation, Rotation)).

hsr_existing_object_at(Pose, Threshold, Instance) :-
    rdf(Instance, rdf:type, owl:'NamedIndividual', belief_state),
    rdfs_individual_of(Instance, hsr_objects:'Items'),
    object_pose(Instance, OldPose),
    transform_close_to(Pose, OldPose, Threshold).



