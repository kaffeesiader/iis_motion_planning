BASE   = ../..
NAME   = lib$(shell basename `pwd`)
OUTPUT := $(BASE)/lib/$(NAME).so

DEPEND = Core

SRC_CPP = $(shell find . -maxdepth 1 -name '*.cpp')
SRC_C   = $(shell find . -maxdepth 1 -name '*.c')
SRCS = $(SRC_CPP) $(SRC_C)
OBJS = $(SRC_CPP:%.cpp=%.o) $(SRC_C:%.c=%.o)

include $(BASE)/gofMake/generic.mk
