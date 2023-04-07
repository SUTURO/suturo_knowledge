# suturo_knowledge

We decided to restart the knowledge package in WiSe 2022/23.

The old code stays in [knowledge_old](knowledge_old) for reference.

## Getting started

### Recommended code editors

- *Emacs*

   If you want to edit Prolog with Emacs, use `find -iname '*.pl' -exec etags --lang=prolog {} +` in the `src` directory.
   This will generate a TAGS file that Emacs can use to jump to the definition of a predicate.  
   You can also specify paths to be searched between `find` and `iname`. `find ../../.. -iname '*.pl' -exec etags --lang=prolog {} +` for example indexes everything from the folder 3 levels up

- *Visual Studio Code*

  Install the `VSC-Prolog` extension
