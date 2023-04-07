# suturo_knowledge

We decided to restart the knowledge package in WiSe 2022/23.

The old code stays in [knowledge_old](knowledge_old) for reference.

## Getting started
The following list is intended as a guideline how to get started using SUTURO Knowledge and KnowRob. The first steps aim at getting an overview of what is available and how to use the existing modules.

### Install the system
Follow the [installation guide](https://github.com/suturo21-22/suturo-installation) to set up the SUTURO Knowledge system on your computer.
_TODO: Provide updated installtion guide_

### Documentation
There is a number of README files formatted in markup description that are part of the SUTURO Knowledge repository. They document the organization of the knowledge modules into different sub-components, and what interfaces they provide.

The documentation is also deployed to the [SUTURO Knowledge Website](https://suturo.github.io/suturo_knowledge/) _TODO: [#165](https://github.com/lheinbokel/SUTURO-documentation/issues/165)_

### Recommended code editors

- *Emacs*

   If you want to edit Prolog with Emacs, use `find -iname '*.pl' -exec etags --lang=prolog {} +` in the `src` directory.
   This will generate a TAGS file that Emacs can use to jump to the definition of a predicate.  
   You can also specify paths to be searched between `find` and `iname`. `find ../../.. -iname '*.pl' -exec etags --lang=prolog {} +` for example indexes everything from the folder 3 levels up

- *Visual Studio Code*

  Install the `VSC-Prolog` extension
