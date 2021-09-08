:- module(furnitures,
    [
        init_furnitures/0,
        is_furniture/1,
        all_furnitures/1,
        has_surface/2,
        furniture_surfaces/2,
        furniture_pose/2
    ]).



init_furnitures :-
    get_urdf_id(URDF),
    urdf_link_names(URDF, Links),
    findall(FurnitureLink, 
    (
        member(FurnitureLink, Links),
        (
            sub_string(FurnitureLink,_,_,_,"table_front_edge_center");
            sub_string(FurnitureLink,_,_,_,"shelf_base_center");
            sub_string(FurnitureLink,_,_,_,"bucket_front_edge_center")
        )
    ),
    FurnitureLinks),
    forall(member(FurnitureLink2, FurnitureLinks),
    (
        split_string(FurnitureLink2, ":", "", [_, Type, Shape]),
        create_furniture(Type, Furniture),
        tell(triple(Furniture, urdf:'hasURDFName', FurnitureLink2)),
        tell(has_type(FurnitureLocation, soma:'Location')),
        tell(triple(Furniture, dul:'hasLocation', FurnitureLocation)),
        assign_surfaces(Furniture, FurnitureLink2, Shape),
        init_visit_state(Furniture)
    )).



create_furniture(FurnitureType, Furniture) :-
    sub_string(FurnitureType,_,_,_,"armchair"),
    tell(has_type(Furniture, hsr_rooms:'Armchair')),
    !.

create_furniture(FurnitureType, Furniture) :-
    sub_string(FurnitureType,_,_,_,"bed"),
    tell(has_type(Furniture, hsr_rooms:'Bed')),
    !.

create_furniture(FurnitureType, Furniture) :-
    sub_string(FurnitureType,_,_,_,"bucket"),
    tell(has_type(Furniture, hsr_rooms:'Bucket')),
    !.

create_furniture(FurnitureType, Furniture) :-
    sub_string(FurnitureType,_,_,_,"container"),
    tell(has_type(Furniture, hsr_rooms:'Container')),
    !.

create_furniture(FurnitureType, Furniture) :-
    sub_string(FurnitureType,_,_,_,"couch"),
    tell(has_type(Furniture, hsr_rooms:'Couch')),
    !.

create_furniture(FurnitureType, Furniture) :-
    sub_string(FurnitureType,_,_,_,"cabinet"),
    tell(has_type(Furniture, hsr_rooms:'Cabinet')),
    !.

create_furniture(FurnitureType, Furniture) :-
    sub_string(FurnitureType,_,_,_,"dishwasher"),
    tell(has_type(Furniture, hsr_rooms:'Dishwasher')),
    !.

create_furniture(FurnitureType, Furniture) :-
    sub_string(FurnitureType,_,_,_,"fridge"),
    tell(has_type(Furniture, hsr_rooms:'Fridge')),
    !.

create_furniture(FurnitureType, Furniture) :-
    sub_string(FurnitureType,_,_,_,"shelf"),
    tell(has_type(Furniture, hsr_rooms:'Shelf')),
    !.

create_furniture(FurnitureType, Furniture) :-
    sub_string(FurnitureType,_,_,_,"sideboard"),
    tell(has_type(Furniture, hsr_rooms:'Sideboard')),
    !.

create_furniture(FurnitureType, Furniture) :-
    sub_string(FurnitureType,_,_,_,"sidetable"),
    tell(has_type(Furniture, hsr_rooms:'Sidetable')),
    !.

create_furniture(FurnitureType, Furniture) :-
    sub_string(FurnitureType,_,_,_,"sink"),
    tell(has_type(Furniture, hsr_rooms:'Sink')),
    !.

create_furniture(FurnitureType, Furniture) :-
    sub_string(FurnitureType,_,_,_,"table"),
    tell(has_type(Furniture, hsr_rooms:'Table')),
    !.


is_furniture(Furniture) :-
    has_type(Furniture, soma:'DesignedFurniture').


all_furnitures(Furnitures) :-
    findall(Furniture, is_furniture(Furniture), Furnitures).


has_surface(Furniture, Surface) :-
    triple(Furniture, hsr_rooms:'hasSurface', Surface).


furniture_surfaces(Furniture, Surfaces) :-
    findall(Surface, has_surface(Furniture, Surface), Surfaces).


furniture_pose(Furniture, Pose) :-
    has_urdf_name(Furniture, URDFName),
    tf_lookup_transform(Furniture, 'map', pose(FurniturePosition, FurnitureRotation)),
    Pose = [URDFName, 'map', FurniturePosition, FurnitureRotation].