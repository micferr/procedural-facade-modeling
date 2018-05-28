Procedural Facade Modeling
==========================

External Dependencies
--------------------

The project depends on LibIGL and CGAL, which need to be installed for it to build. Cork can be used instead of CGAL, but references to the <code>igl::copyleft::cgal</code> namespace in source files must be replaced with the equivalent functionalities in the <code>igl::copyleft::cork</code>.
However, CGAL is the recommended choice as, although slower than Cork, it is more robust in its computations.