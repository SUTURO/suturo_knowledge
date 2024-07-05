import sys
import os
import yaml
from rdflib import Graph, URIRef, Namespace, RDF, RDFS

def get_transitive_subclasses(g, superclass):
    subclasses = set()
    for subclass in g.transitive_subjects(RDFS.subClassOf, superclass):
        subclasses.add(subclass)
    # We also ant to get the names from superclass if it exist
    subclasses.add(superclass)
    return subclasses

def extract_has_value_restrictions(owl_file_path, data_property, superclass_uris):
    # Load the ontology
    g = Graph()
    g.parse(owl_file_path)
    g.parse("http://www.ease-crc.org/ont/SOMA-HOME.owl")
    g.parse("http://www.ease-crc.org/ont/SOMA.owl")
    g.parse("http://www.ease-crc.org/ont/DUL.owl")

    # Define namespaces
    OWL = Namespace("http://www.w3.org/2002/07/owl#")
    DATA_PROPERTY = URIRef(data_property)  # Create a URIRef for the data property

    results = {}
    for superclass_label, superclass_uri in superclass_uris.items():
        superclass_uri = URIRef(superclass_uri)
        subclasses = get_transitive_subclasses(g, superclass_uri)
        
        results[superclass_label] = {'entities': []}

        for cls in subclasses:
            for cls, _, restriction in g.triples((cls, RDFS.subClassOf, None)):
                if (restriction, RDF.type, OWL.Restriction) in g:
                    if (restriction, OWL.onProperty, DATA_PROPERTY) in g:
                        for _, _, value in g.triples((restriction, OWL.hasValue, None)):
                            results[superclass_label]['entities'].append(str(value))

    return results

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

    superclass_uris = {
        'Food': 'http://www.ease-crc.org/ont/SUTURO.owl#Food',
        'Drink': 'http://www.ease-crc.org/ont/SUTURO.owl#Drink',
        'NaturalPerson': 'http://www.ontologydesignpatterns.org/ont/dul/DUL.owl#NaturalPerson',
        'PhysicalArtifact': 'http://www.ontologydesignpatterns.org/ont/dul/DUL.owl#PhysicalArtifact',
        'PhysicalPlace': 'http://www.ease-crc.org/ont/SOMA.owl#Room'
    }

    # Check if the file exists
    if not os.path.isfile(owl_file_path):
        print(f"File not found: {owl_file_path}")
        sys.exit(1)

    entities = extract_has_value_restrictions(owl_file_path, data_property, superclass_uris)
    save_to_yaml(entities, yaml_file_path)
    print(f"Entities saved to {yaml_file_path}")

if __name__ == "__main__":
    main()
