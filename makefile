CWD = $(shell pwd)

all: catkin

catkin: komo
	cd $(CWD)/../..; catkin_make;

komo:
	cd $(CWD)/KOMO/share; make;
	cd $(CWD)/KOMO/share/examples/KOMO/easy; make;
	cd $(CWD)/KOMO/share/examples/KOMO/ikea_chair; make;
	cd $(CWD)/KOMO/share/examples/Ors/ors_editor; make;