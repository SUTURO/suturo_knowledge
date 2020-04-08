# The NLP part of the SUTURO 19/20 Project

## Installation

Installation of the HSR included Julius components:
```
sudo apt-get update
sudo apt-get install ros-kinetic-tmc-desktop-full
```
Installation of Python modules
```
sudo -H pip install tinyrpc==0.9.4 zmq
sudo -H pip3 install simplenlg sling tinyrpc==0.9.4 zmq
```

## Provided Packages

### suturo_julius
Contains dictionaries, julius configuration files and launchfiles.
It's only purpose is to start julius.
### suturo_nlg
Contains the nlg_ros node. This node interfaces with different python3 scripts producing sentences.
Also contains the knowledge_interface. This node takes does rosprolog queries to answer basic questions.
### suturo_textparser
contains the sling parser,its ros interfacing node and a textparser thats simplyfiies and reduces the julius output.
### suturo_speechparser
Just contains some launchfiles starting node from all the above packages.