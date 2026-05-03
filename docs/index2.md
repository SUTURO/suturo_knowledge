<div class="overview-header">
  <img src="assets/images/suturo-knowledge-full-1000.png" width="256">
</div>

## Overview

The SUTURO Knowledge Team develops the semantic and knowledge-based components that enable our robots to understand, interpret, and reason about their environment.  
Our work focuses on building structured, queryable world models that support manipulation, navigation, and high-level decision-making in everyday household tasks.

In previous years, the SUTURO stack relied heavily on **KnowRob** for symbolic reasoning and semantic world representation.  
Today, our system is built around the **Semantic Digital Twin (semdt)** framework — a modern, Python-based approach to semantic scene modeling that integrates geometry, kinematics, and meaning into a unified representation.

---

## Semantic Digital Twin (semdt)

**Semantic Digital Twin** provides a powerful interface for constructing, annotating, and querying semantic world models.  
It allows our robots to work not just with raw coordinates, but with *interpretable concepts* such as containers, shelves, handles, drawers, and task-relevant regions.

### Key Capabilities

- **Full Kinematic World Modeling**  
  Represent articulated objects, bodies, joints, and degrees of freedom as first-class entities.

- **Semantic Views & Meaning**  
  Enrich raw geometry with actionable concepts (e.g., “this is a shelf”, “this is a graspable handle”).

- **High-Level Querying**  
  Retrieve meaningful entities using expressive queries like  
  *“the handle of the drawer that is currently open”*.

- **Persistence & Replay**  
  Serialize entire semantic worlds into SQL for reproducible experiments and data pipelines.

- **Composable Scene Authoring**  
  Use factories and dataclasses to build complex environments with minimal boilerplate.

- **Reliable Kinematics**  
  Compute transforms, forward kinematics, and inverse kinematics consistently across the world model.

- **Real-Time Synchronization**  
  Keep multiple processes or agents aligned with a shared, synchronized world state.

- **Flexible Visualization**  
  Inspect semantic worlds in RViz2, notebooks, or simulation environments.

---

## Getting Started

### Installation Guide  
A new installation guide for the SUTURO Knowledge stack (based on semdt) is currently being prepared.  
It will cover:

- setting up the `semantic_digital_twin` Python environment  
- integrating SUTURO resources and models  
- connecting semdt with CRAM / PyCRAM workflows  
- running example scenes and queries  

**TODO:** Add updated installation guide.

---

## Recommended Code Editors

### PyCharm (recommended)
- Excellent Python support  
- Built-in tools for virtual environments, debugging, and refactoring  
- Great for working with semdt’s Python-based API

### Visual Studio Code
- Lightweight and flexible  
- Python extension provides linting, autocompletion, and debugging  
- Good for mixed-language projects

### What We Work On

The Knowledge Team contributes to:
* semantic modeling of household environments
* object and container semantics (shelves, cupboards, drawers, surfaces)
* reasoning about accessibility, containment, and affordances
* maintaining consistent world states during manipulation tasks
* integrating semantic knowledge with CRAM / PyCRAM planning
* building reusable semantic assets for the entire SUTURO project
Our goal is to give the robot a **structured understanding of the world**, enabling robust and intelligent behavior in complex environments.
