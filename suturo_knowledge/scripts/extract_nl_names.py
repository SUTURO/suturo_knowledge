import sys
import os
import yaml
from rdflib import Graph, URIRef, Namespace, RDF, RDFS

def get_transitive_subclasses(g, superclasses, exclusions):
    subclasses = set()
    for superclass in superclasses:
        for subclass in g.transitive_subjects(RDFS.subClassOf, superclass):
            if(subclass not in exclusions):
                subclasses.add(subclass)
        # We also want to get the names from superclass if it exists
        if(subclass not in exclusions):
            subclasses.add(superclass)
    return subclasses

def extract_has_value_restrictions(owl_file_path, data_property, superclass_uris, exclusion_uris):
    # Load the ontology
    g = Graph()
    g.parse(owl_file_path)
    # We hardcode the loading of the imported ontologies for now
    g.parse("http://www.ease-crc.org/ont/SOMA-HOME.owl")
    g.parse("http://www.ease-crc.org/ont/SOMA.owl")
    g.parse("http://www.ease-crc.org/ont/DUL.owl")

    # Define namespaces
    OWL = Namespace("http://www.w3.org/2002/07/owl#")
    DATA_PROPERTY = URIRef(data_property)  # Create a URIRef for the data property

    # Create the resulting map
    results = {}
    for superclass_label, superclass_uris in superclass_uris.items():
        superclass_uris = [URIRef(uri) for uri in superclass_uris]
        exclusions_with_subclasses = []
        if (superclass_label in exclusion_uris):
            exclusion_uris = [URIRef(uri) for uri in exclusion_uris[superclass_label]]
            exclusions_with_subclasses = get_transitive_subclasses(g, exclusion_uris, [])
        subclasses = get_transitive_subclasses(g, superclass_uris, exclusions_with_subclasses)

        results[superclass_label] = {'entities': []}

        for cls in subclasses:
            for cls, _, restriction in g.triples((cls, RDFS.subClassOf, None)):
                if (restriction, RDF.type, OWL.Restriction) in g:
                    if (restriction, OWL.onProperty, DATA_PROPERTY) in g:
                        for _, _, value in g.triples((restriction, OWL.hasValue, None)):
                            results[superclass_label]['entities'].append(str(value))

    return results

def check_missing_names(entity, owl_file_path, results, data_property):

    g = Graph()
    g.parse(owl_file_path)
    g.parse("http://www.ease-crc.org/ont/SOMA-HOME.owl")
    g.parse("http://www.ease-crc.org/ont/SOMA.owl")
    g.parse("http://www.ease-crc.org/ont/DUL.owl")
    ENTITY = URIRef(entity)
    subclasses = get_transitive_subclasses(g, [ENTITY], [])
    missing = []

    # Define namespaces
    OWL = Namespace("http://www.w3.org/2002/07/owl#")
    DATA_PROPERTY = URIRef(data_property)  # Create a URIRef for the data property

    cls_with_dataprop = set()

    for cls in subclasses:
        for cls, _, restriction in g.triples((cls, RDFS.subClassOf, None)):
            if (restriction, RDF.type, OWL.Restriction) in g:
                if (restriction, OWL.onProperty, URIRef(data_property)) in g:
                    for _, _, value in g.triples((restriction, OWL.hasValue, None)):
                        cls_with_dataprop.add(value)
    
    for labels in cls_with_dataprop:
        missingflag = True
        for superclass_label, entities in results.items():
            if str(labels) in entities['entities']:
                missingflag = False
        if(missingflag):
            missing.append(labels)

    return missing

def save_to_yaml(data, yaml_file_path):
    with open(yaml_file_path, 'w') as yaml_file:
        yaml.dump(data, yaml_file, default_flow_style=False)

def main():
    if len(sys.argv) != 2:
        print("Usage: python extract_nl_names.py <path_to_owl_file>")
        sys.exit(1)

    owl_file_path = sys.argv[1]  # Path to the OWL file from command-line argument
    yaml_file_path = 'nlp_entities.yml'  # Specify the output YAML file path
    data_property = 'http://www.ease-crc.org/ont/SUTURO.owl#hasPredefinedName'  # Specify the data property to look for

    # Dict of category in NLP to classes (and their subclasses)
    superclass_uris = {
        'Clothing': [
            'http://www.ease-crc.org/ont/SUTURO.owl#Clothing'
        ],
        'NaturalPerson': [
            'http://www.ontologydesignpatterns.org/ont/dul/DUL.owl#NaturalPerson'
        ],
        'DesignedFurniture': [
            'http://www.ease-crc.org/ont/SOMA.owl#DesignedFurniture',
            'http://www.ease-crc.org/ont/SOMA.owl#Door',
            'http://www.ease-crc.org/ont/SOMA.owl#Appliance',
            'http://www.ease-crc.org/ont/SOMA.owl#Drawer'
        ],
        'Room': [
            'http://www.ease-crc.org/ont/SOMA.owl#Room',
        ],
        'Transportable': [
            'http://www.ease-crc.org/ont/SOMA.owl#Dish',
            'http://www.ease-crc.org/ont/SUTURO.owl#Fish',
            'http://www.ease-crc.org/ont/SUTURO.owl#Fruit',
            'http://www.ease-crc.org/ont/SUTURO.owl#Soup',
            'http://www.ease-crc.org/ont/SUTURO.owl#Meat',
            'http://www.ease-crc.org/ont/SUTURO.owl#ProcessedFood',
            'http://www.ease-crc.org/ont/SUTURO.owl#Vegetable'
            'http://www.ease-crc.org/ont/SUTURO.owl#Spice',
            'http://www.ease-crc.org/ont/SUTURO.owl#Drink',
            'http://www.ease-crc.org/ont/SOMA.owl#DesignedTool',
            'http://www.ease-crc.org/ont/SOMA.owl#DesignedContainer'
        ]
    }

    exclusion_uris = {
        'Transportable': [
            'http://www.ease-crc.org/ont/SOMA.owl#Appliance',
            'http://www.ease-crc.org/ont/SOMA.owl#Drawer',
            'http://www.ease-crc.org/ont/SOMA.owl#Cupboard',
            'http://www.ease-crc.org/ont/SOMA.owl#DesignedHandle'
        ]
    }

    # Check if the file exists
    if not os.path.isfile(owl_file_path):
        print(f"File not found: {owl_file_path}")
        sys.exit(1)

    # extract the predefinedName relations according to the given dict
    entities = extract_has_value_restrictions(owl_file_path, data_property, superclass_uris, exclusion_uris)
    # Get the rest of the physical artifacts, and save them as Transportable
    missing_physical_artifacts = [str(name) for name in check_missing_names('http://www.ontologydesignpatterns.org/ont/dul/DUL.owl#Entity', owl_file_path, entities, data_property)]
    # add missing physical artifacts to transportable
    entities["Transportable"]["entities"].extend(missing_physical_artifacts)

    # hardcoded pronouns
    entities["Transportable"]["entities"].extend(["it"])
    entities["DesignedFurniture"]["entities"].extend(["there"])
    entities["NaturalPerson"]["entities"].extend(["them", "they", "him", "her", "me"])

    # Save the yaml file
    save_to_yaml(entities, yaml_file_path)
    print(f"Entities saved to {yaml_file_path}")

    # This prints out all subclasses of Entity with predefinedName
    # to check if something is missing 
    missing_names = check_missing_names('http://www.ontologydesignpatterns.org/ont/dul/DUL.owl#Entity', owl_file_path,entities, data_property)
    if missing_names:
        print("Missing classes with hasPredefinedName property:")
        for artifact in missing_names:
            print(artifact)
    else:
        print("No missing classes with hasPredefinedName property.")

if __name__ == "__main__":
    main()