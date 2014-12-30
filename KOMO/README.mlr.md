# MLR Repository

Doxygen documentation: http://sully.informatik.uni-stuttgart.de:8080/job/MLR-share-doxygen/doxygen/

Jenkins status: http://sully.informatik.uni-stuttgart.de:8080/


## Install on Ubuntu
Install the mlr system by executing:

  ./install/INSTALL

In "share/make-config" you can change the compile settings (un)set
dependencies.


## Install on Arch

What to do to install MLR on Arch Linux?

First install all dependencies. You need some things from AUR, so first install
yaourt or something similar to handle AUR packages.

    $ yaourt -S opencv plib ann lapack blas qhull ode

In order to get a working cblas library we need atlas-lapack, but be warned.
This literally will take hours of compile time (something like 5-10 h depending
on your system).

    $ yaourt -S atlas-lapack

(You can either drink a lot of coffee in the meantime or put LAPACK = 0 in your
make-config until it is finished...)

MLR needs a .so-file called libANN, ann provides one, but names it libann so we
link it:

    $ sudo ln -s /usr/lib/libANN.so /usr/lib/libann.so
    $ sudo ldconfig

qhull is newer in arch than in ubuntu and changes some pointer/reference things.
You need to put 

   ARCH_LINUX = 1

in your make-config. This fixes the qhull change.

Everything else is equal to ubuntu.


## QtCreator tips

* Enable Debugging helpers:
  .gdbinit -> git/mlr/tools/qt_mlr_types.py

* set formatting:
  Options > C++ > Import...   git/mlr/tools/qt_coding_style.xml
  
* Change signal handling of gdb
  Open 'Debugger Log' window
  write gdb commands:
  handle SIGINT pass

* append source directory to all "*.files" files:
echo "echo '/home/mtoussai/git/mlr/share/src' >> $1" > nogit_append_path
find . -name '\.*\.includes' -exec ./nogit_append_path {} \;

## Getting started

The best way to learn about this code and understand what is can do it to go through all examples.
and take a look at the Doxygen documentation http://sully.informatik.uni-stuttgart.de:8080/job/MLR-share-doxygen/doxygen/

For the examples start with

 - examples/Core
 - examples/Gui
 - examples/Ors

All of them should work. Going through the main should be instructive on how you could use the features. In examples/Ors some demos depend on linking to Physx which is turned off on default.

Continue with more specialized packages:

 - examples/Optim
 - examples/Motion
 - examples/Algo
 - examples/Infer

which implement algorithmic methods.

Finally examples/Hardware contains drivers/recorders for Kinect, Cams, G4.

examples/System is code for designing parallized modular systems (using Core/module.h) - which I think works robustly but needs more tools for others to be usable.

examples/relational is Tobias relational RL code


## Documentation

Create the doxygen documentation with:

    cd share
    make doc

