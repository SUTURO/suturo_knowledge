# neem-logging-guide
This guide will give a comprehensive overview of the **suturo-neem-logger** and its practices, alongside an useful introduction into the concepts of **NEEMs**.
Please notice that a similar kind of content already exists in the **Final Report 20/21**. This file is to be seen as an hands-on developer-guide for beginners.  
![logging format](images/logging_format.png)
![soma](images/SOMA.png)
-----------------

### About NEEMs
The term NEEM is an abbreviation for **N**arrarive **E**nabled **E**pisodic **M**emories. 
<br>
Conceptionally, it represents the robots memories about its past experiences. 
For instance the memory of a plan that was executed before, where every action can be seen as a task and the whole plan as an Episode.
<br>
Technically speaking it is a set of logged data within the **mongo DB** that consists of rich semantical & numerical data.
<br>
During plan execution, the robot is able to log its perceived environment and to export the memorised logs into a NEEM-folder. This folder can be later loaded into any kind of **mongo DB** as a new Collection.
<br>
For further guidance please visit the KnowRob webiste: http://knowrob.org and consult the NEEM-Handbook:  https://ease-crc.github.io/soma/owl/current/NEEM-Handbook.pdf
<br>
For further guidance on Episodic Memory please visit the Wikipedia article: https://en.wikipedia.org/wiki/Episodic_memory

### About SOMA

The term SOMA is an abbreviation for  **S**ocio-**p**hysical **M**odel of **A**ctivities. 
<br>
It is an Ontology that contains an extensive amount of knowledge, necessary to represent actions & relations in the context of household robotics.
<br>
The SOMA ontologies can be used to log the robots actions in an expressive way. For instance to correctly represent the affordances and roles of objects and actors within the perceived environment of the agent.
<br>
For further guidance please visit the webiste:
https://ease-crc.github.io/soma/

-----------------

### suturo-neem-logger

The **suturo-neem-logger** is a set of modules and features with which one can generate NEEMS from SUTURO plans.
<br>
Currently, it is represented within the two branches of **neem-testing** (planning) & **suturo-neem-logger** (knowledge). 

#### neem-testing branch (planning)
##### execute-cleanup.lisp
- This file is the main entry point of the plan execution
- Currently we worked on the **clean up** plan to test out the NEEM-logging, however it can be applied to any other plan
- In here we start the logging & execute the actions 
- Please consider that there are also multiple **execute-neem-plan-x.lisp**, which consist of smaller plans to test the log functionality

##### prolog-neem-queries.lisp
- This file contains the low-level-interface (llif) to knowledge
- All predicates called in the **execute-cleanup.lisp** are defined here

#### suturo-neem-logger branch (knowledge)

##### general_logs.pl
- This module holds all necessary functions to start & conclude the logging  

##### manipulation_logs.pl
- This module contains the logging formats for whenever the robot manipulates its environment

##### mental_logs.pl
- This module contains the logging formats for all knowledge related tasks

##### navigation_logs.pl
- This module contains the logging formats for whenever the robot moves around its environment

##### perception_logs.pl
- This module contains the logging formats for image recognition tasks

#### neem-team
- The **neem-team** is a role in the SUTURO project and consists of 1 planning member and 1 knowledge member
- The two members work closely together in order to generate NEEMs from existing SUTURO plans
- Please consider that the role is not mandatory, and that you should primarily focus on the existing SUTURO tasks

### logging a plan (overview)
- In order to log a plan execution, the **neem-team** should have a good understanding of the planning & knowledge repository
- The team should work closely together to avoid misunderstandings & to evaluate the generated results
- In general, the procedure should look as follows:
- (1) In the **execute.lisp**, go through all the actions of the plan you wish to log 
- (2) Mark down the actions that are necessary for logging
- (3) After that, make the appropriate logging calls 
- (4) Execute the plan & evaluate the generated NEEM folder 

### logging a plan (technical)
- Since learning by example is often the best way to go, we will show a quick example of how to log an action
- Lets say you go through an existing plan (in the **execute.lisp**), and you come across an action like ``move-hsr``
- In order to log this kind of action, you need to take a look at the existing functions from the **prolog-neem-query**
- When you find the appropriate log, ``log-move-hsr``, you go back to the **execute.lisp**
- Now above the aforementioned ``move-hsr`` you will now call the ``log-move-hsr``, and below you need to call ``log-end-action``
- So that it looks like this:  
<br> 
(Please notice how we added comments and empty space for an easier overview) 
<br> 

   ``
   ;; # ------------- move-hsr -------------#
   ``
   <br>
   `` 
   (setf action-id (llif::log-move-hsr episode-id)) 
   ``   
   ``
   (comf::move-hsr nav-pose)
   ``
   <br>
   `` 
   (llif::log-ending-action action-id)
   ``

### developing new logs
- In case you come across an action that is not represented correctly by the existing logs, you are free to create a new one
- For this, you need to do 2 things: (1) Define a new logging predicate (2) Create a suitable function in the **prolog-neem-query.lisp**
- Explanation:
- (1.1) In the **neems** directory within knowledge, select a module that represent the appropriate logging use case
- (1.2) Write a new predicate corresponding to the following structure: 
    - main logs 
    - type logs 
    - execute logs 
    - further main logs 
    - (those calls can be made differently, however it helps as a good practice)
- (1.3) Consider the naming conventions: **some_action_logging(EpisodeID,ActionID) :-**
- (2.1) In the **prolog-neem-query.lisp** create a function to call the aforementioned predicate
- (2.2) Consider the naming conventions: **(defun log-some-action (id)**

-----------------

### Q&A
##### Why do we need to log it that way?
- For a NEEM to correctly represent the agents experiences, it needs to consist of one **Episode** and the corresponding **Actions**
- Every **Action** must be referenced to the **Episode**, and in addition contain the information on how long it took
- We realise this by passing the necessary **IDs** into every logging predicate:
    - ``tell(is_setting_for(EpisodeID, ActionID))``
    - ``begin_action_logging(ActionID)``
    - ``end_action_logging(ActionID)``
- However this must be managed from CRAM, in that the **execute.lisp** calls all necessary functions
- This is why we make use of two variables:
    - ``(defvar episode-id NIL)``
    - ``(defvar action-id NIL)``
- Those **IDs** are then passed into the functions of **prolog-neem-query.lisp** and are returned respectively
    
##### Is it the only way to log NEEMs ?
- No, the actual tool is the CRAM Cloud Logger (ccl)
- However due to technical complications we decided to create an own implementation on the context of SUTURO
- You should however take a look at the ccl and try it out yourself 

##### Does the suturo-neem-logger cover all use cases? 
- No, the **suturo-neem-logger** covers the existing plans of SUTURO but not further possible use cases
- Additionally, further data (eg. from other components such as Giskard/RoboSherlock) is not being logged but could however be potentially useful
- Please feel free to extend the functionality of this application, thereby trying out new aspects of the SOMA ontology and other components 
    - You can also extend and alter the parameters of the predicates, since they currently only pass IDs
    - However consult your tutors in case they would rather want you to make use of the aforementioned ccl


