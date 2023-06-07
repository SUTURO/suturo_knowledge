<div class="overview-header">
  <img src="assets/images/suturo-knowledge-full-1000.png" width="256">
</div>

# Overview

The SUTURO Knowledge stack is based on [KnowRob](https://github.com/knowrob/knowrob), a knowledge processing system, which provides functionalities to represent knowledge and reasoning methods. We are using it to store, represent and infer rich information about the robots state and its environment.

## Getting started

### Installation guide
Follow the [installation guide](https://github.com/suturo21-22/suturo-installation) to set up the SUTURO Knowledge system on your computer.  
_TODO: Provide updated installation guide_

### Recommended code editors

- **Emacs**  
  If you want to edit Prolog with Emacs, use the following command in in the `src` directory:
  ```bash
  find -iname '*.pl' -exec etags --lang=prolog {} +  
  ```
  This will generate a TAGS file that Emacs can use to jump to the definition of a predicate.  
  You can also specify paths to be searched between `find` and `-iname`.  
  The following for example indexes everything from the folder 3 levels up:
  ```bash
  find ../../.. -iname '*.pl' -exec etags --lang=prolog {} +
  ```

- **Visual Studio Code**  
  Install the [`VSC-Prolog`](https://marketplace.visualstudio.com/items?itemName=arthurwang.vsc-prolog) extension to get syntax highlighting and code completion for Prolog in VS Code.

---

<div class="overview-footer" >
  <a href="https://www.uni-bremen.de/">
    <img src="assets/images/uni-bremen-logo-footer.png">
  </a>
  <a href="https://github.com/suturo">
    <img src="assets/images/suturo-logo-footer.png">
  </a>
</div>