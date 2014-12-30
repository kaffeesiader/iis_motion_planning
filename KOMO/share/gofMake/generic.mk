# CXX=clang++
# this is a generic make file
# supposed to be called from lib, test and projects makefiles
#
# user configuration options are set in config.mk -- not here!

.SECONDARY:

BASE_REAL = $(shell realpath $(BASE))

################################################################################
#
# load user options from the local make-config
#
################################################################################
-include $(BASE)/gofMake/config.mk
-include $(BASE)/gofMake/z.mk


################################################################################
#
# standard objects to be compiled, output file
#
################################################################################
ifndef OBJS
OBJS = main.o
endif
ifndef OUTPUT
OUTPUT = x.exe
endif


################################################################################
#
# basic compiler settings
#
################################################################################
# (a tag like `OPTIM=fast' in the local Makefiles changes default debug mode)

ifndef CXX
CXX	= g++
CC	= gcc
endif
MOC = moc-qt4
UIC = uic-qt4
YACC = bison -d

ifndef MLR_LIBPATH
MLR_LIBPATH = /home/lib
endif

LINK	= $(CXX)
CPATHS	+= $(BASE)/include $(BASE)/src $(MLR_LIBPATH)/include
LPATHS	+= $(BASE)/lib $(MLR_LIBPATH)/lib
LIBS += -lrt
SHAREFLAG = -shared #-Wl,--warn-unresolved-symbols #-Wl,--no-allow-shlib-undefined

CXXFLAGS += -fPIC
CXXFLAGS += $(EXTRA_CXXFLAGS)

CFLAGS += -fPIC

ifndef MLR_NO_CXX11
CXXFLAGS += -std=c++0x
endif

ifndef OPTIM
OPTIM = debug
endif

ifeq ($(OPTIM),debug)
CXXFLAGS := -g -Wall $(CXXFLAGS)#-Wno-int-to-pointer-cast#-Wno-invalid-offsetof
endif
ifeq ($(OPTIM),fast_debug)
CXXFLAGS := -g -O3 -Wall $(CXXFLAGS)
endif
ifeq ($(OPTIM),penibel)
CXXFLAGS := -g -Wall -Wextra $(CXXFLAGS)
endif
ifeq ($(OPTIM),fast)
CXXFLAGS := -O3 -Wall -DMT_NOCHECK $(CXXFLAGS)
endif
ifeq ($(OPTIM),prof)
CXXFLAGS := -O3 -pg -Wall -DMT_NOCHECK -fno-inline $(CXXFLAGS)
LDFLAGS += -pg
endif
ifeq ($(OPTIM),callgrind)
CXXFLAGS := -O3 -g -Wall -DMT_NOCHECK -fno-inline $(CXXFLAGS)
endif


################################################################################
#
# load defines for linking to external libs
#
################################################################################
-include $(BASE)/gofMake/defines.mk


################################################################################
#
# VARS for SWIG wrappers
#
################################################################################

ifndef MLR_PATH
	MLR_PATH=$(HOME)/git/mlr
endif

MODULE_NAME=$(shell echo $(notdir $(CURDIR)) | tr A-Z a-z)

SWC_FLAGS=-std=c++0x -g
SWC_INCLUDES=-I$(MLR_PATH)/share/src -I/usr/include/python2.7 
SWC_LIB_PATH=-L$(MLR_PATH)/share/lib -Xlinker -rpath $(MLR_PATH)/share/lib
SWC_CXXFLAGS=-c $(SWC_FLAGS) -fPIC $(SWC_INCLUDES) -DSWIG_TYPE_TABLE=mlr
SWC_LDFLAGS=$(SWC_FLAGS) -shared $(SWC_LIB_PATH) $(SWC_INCLUDES) $(DEPEND:%=-l%) -l$(notdir $(CURDIR))

SWIG=swig2.0
SWIG_INCLUDE=-I$(MLR_PATH)/share/src -I$(MLR_PATH)/share/include/numpy
SWIG_FLAGS=-c++ -python $(SWIG_INCLUDE)


################################################################################
#
# depending on a local component
#
################################################################################

BUILDS := $(DEPEND:%=$(BASE)/lib/lib%.so) $(BUILDS)
LIBS := $(DEPEND:%=-l%) $(LIBS)
CXXFLAGS := $(DEPEND:%=-DMT_%) $(CXXFLAGS)


################################################################################
#
# export Linux/MSVC include/lib paths
#
################################################################################
CPATH := $(CPATH):$(CPATHS:%=:%:)
LPATH := $(LPATH):$(LPATHS:%=:%:)
LDFLAGS += $(LPATHS:%=-L%)
LD_RUN_PATH := $(LD_RUN_PATH):$(LPATH)
LD_LIBRARY_PATH := $(LD_LIBRARY_PATH):$(LPATH)
export CPATH
export LPATH
export LD_RUN_PATH
export LD_LIBRARY_PATH
export MSVC_CPATH
export MSVC_LPATH


################################################################################
#
# concrete make targets
#
################################################################################

default: $(OUTPUT)

clean:
	rm -f $(OUTPUT) $(OBJS) $(PREOBJS) callgrind.out.* .lastMake $(CLEAN)
	@find $(BASE) -type d -name 'Make.lock' -delete -print
	@rm -f $(MODULE_NAME)_wrap.* $(MODULE_NAME)py.so $(MODULE_NAME)py.py

cleanLocks: force
	@find $(BASE) -type d -name 'Make.lock' -delete -print
	@find $(BASE) -type f -name '.lastMake' -delete -print

depend: generate_Makefile.dep

info: force
	@echo; echo ----------------------------------------
	@echo "     " "environment configuration (see make-generic file)";
	@echo ----------------------------------------; echo
	@echo "  PWD =" "$(PWD)"
	@echo "  BASE =" "$(BASE)"
	@echo "  BASE_REAL =" "$(BASE_REAL)"
	@echo "  NAME =" "$(NAME)"
	@echo "  LIBPATH =" "$(LIBPATH)"
	@echo "  EXTERNALS =" "$(EXTERNALS)"
	@echo "  CXX =" "$(CXX)"
	@echo "  OPTIM =" "$(OPTIM)"
	@echo "  CXXFLAGS =" "$(CXXFLAGS)"
	@echo "  LINK =" "$(LINK)"
	@echo "  LDFLAGS =" "$(LDFLAGS)"
	@echo "  CPATH =" "$(CPATH)"
	@echo "  LPATHS =" "$(LPATHS)"
	@echo "  LPATH =" "$(LPATH)"
	@echo "  LD_RUN_PATH =" "$(LD_RUN_PATH)"
	@echo "  MSVC_CPATH =" "$(MSVC_CPATH)"
	@echo "  MSVC_LPATH =" "$(MSVC_LPATH)"
	@echo "  SRCS =" "$(SRCS)"
	@echo "  OBJS =" "$(OBJS)"
	@echo "  LIBS =" "$(LIBS)"
	@echo "  PREOBJS =" "$(PREOBJS)"
	@echo "  OUTPUT =" "$(OUTPUT)"
	@echo "  DEPEND =" "$(DEPEND)"
	@echo "  BUILDS =" "$(BUILDS)"
	@echo "  MAKEMODE =" "$(MAKEMODE)"
	@echo

all: default


################################################################################
#
# optionally include dependencies
#
################################################################################
-include Makefile.dep


################################################################################
#
# SWIG rules
#
################################################################################

# keep the actual wrapper
.SECONDARY: $(MODULE_NAME)_wrap.cxx  

%_wrap.cxx: %.i
	$(SWIG) $(SWIG_FLAGS) $<

%_wrap.o: %_wrap.cxx
	$(CXX) $(SWC_CXXFLAGS) $<

%py.so: %_wrap.o
	$(CXX) $(SWC_LDFLAGS) $< -o $(MODULE_NAME)py.so

%py.py: 

pywrapper: $(OUTPUT) $(MODULE_NAME)py.so $(MODULE_NAME)py.py
	install -D $(MODULE_NAME)py.py ~/.local/lib/python2.7/site-packages/$(MODULE_NAME)py.py
	install -D $(MODULE_NAME)py.so ~/.local/lib/python2.7/site-packages/_$(MODULE_NAME)py.so


################################################################################
#
# rules
#
################################################################################

%.exe: $(PREOBJS) $(BUILDS) $(OBJS)
	$(LINK) $(LDFLAGS) -o $@ $(OBJS) $(LIBS)
#	@echo $(PWD:$(BASE_REAL)/%=%/$@)

## this RULE only applies to $(NAME).so
## other %.so files are created by calling make in their directory
$(BASE)/lib/$(NAME).so: $(PREOBJS) $(BUILDS) $(OBJS)
	$(LINK) $(LDFLAGS) -o $@ $(OBJS) $(LIBS) $(SHAREFLAG)

#%_test.so: $(PREOBJS) $(BUILDS) $(OBJS)
#	$(LINK) $(LDFLAGS) -o $@ $(OBJS) $(LIBS) $(SHAREFLAG)

%.lib: $(PREOBJS) $(BUILDS) $(OBJS)
	$(LINK) $(LDFLAGS) -o $@ $(OBJS) $(LIBS) -static ### $(SHAREFLAG)

%.a: $(PREOBJS) $(BUILDS) $(OBJS)
	ar -crvs $@ $(OBJS)

%.mexglx: $(PREOBJS) $(OBJS)
	mex -cxx $(LDFLAGS) -o $@ $(OBJS) $(LIBS)

ifeq ($(CUDA),1)
%_cuda.o: %_cuda.cpp
	if test ! -L $*_cuda.cu; then ln -s -f $*_cuda.cpp $*_cuda.cu; fi;
	$(NXX) $(NXXFLAGS) -o $@ -c $*_cuda.cu
else
%_cuda.o: %_cuda.cpp
	$(CXX) $(CXXFLAGS) -o $@ -c $<
endif

%.o: %.cpp
	$(CXX) $(CXXFLAGS) -o $@ -c $<
#	@echo $(PWD:$(BASE_REAL)/%=%/$<)

%.o: %.cxx
	$(CXX) $(CXXFLAGS) -o $@ -c $<

%.o: %.c
	$(CC) $(CFLAGS) -o $@ -c $<

%.obj: %.cpp
	$(CXX) $(CXXFLAGS) -o $@ -c $<

%.obj: %.cxx
	$(CXX) $(CXXFLAGS) -o $@ -c $<

%_ui.h: %.ui
	$(UIC) -o $*_ui.h $<

%_moc.cpp: %.h
	$(MOC) -o $*_moc.cpp $*.h

%_moc.cxx: %.h
	$(MOC) -o $*_moc.cxx $*.h

%.tab.cpp: %.ypp
	$(YACC) $<

%.tab.h: %.ypp
	$(YACC) --defines=$@ $<

## generate a make dependency file
generate_Makefile.dep: $(SRCS)
	-$(CXX) -MM $(SRCS) $(CXXFLAGS) > Makefile.dep

includeAll.cxx: force
	find . -maxdepth 1 -name '*.cpp' -exec echo "#include \"{}\"" \; > includeAll.cxx


################################################################################
#
# meta rules -- for calling make in other directories
#
################################################################################

$(BASE)/lib/libextern_%.so: $(BASE)/src/extern/%
	+@-$(BASE)/gofMake/make-path.sh $<

$(BASE)/lib/libHardware_%.so: $(BASE)/src/Hardware/%
	+@-$(BASE)/gofMake/make-path.sh $<

$(BASE)/lib/lib%.so: $(BASE)/src/%
	+@-$(BASE)/gofMake/make-path.sh $<

makePath/%: %
	+@-$(BASE)/gofMake/make-path.sh $<

runPath/%: %
	+@-$(BASE)/gofMake/run-path.sh $<

makePythonPath/%: %
	make --directory=$< pywrapper

zip::
	cd ..;  rm -f $(NAME).tgz;  tar cvzf $(NAME).tgz $(NAME) --dereference --exclude-vcs --exclude-from tar.exclude --exclude-from $(NAME)/tar.exclude


force:	;

# vim:ft=make:
