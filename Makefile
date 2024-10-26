# Copyright (C) 2023 Ethan Uppal. All rights reserved.

SRCDIR		:= src
INCLUDEDIR	:= src

CC			:= $(shell which g++ || which clang++)
# CC			:= $(shell which clang++)
PY			:= $(shell which python3 || which python)
CFLAGS		:= -std=c++17 -pedantic -Wall -Wextra -I $(INCLUDEDIR)
CDEBUG		:= -g
CRELEASE	:= -O3 -DRELEASE_BUILD #-fno-fast-math
TARGET		:= main
LIBNAME		:= libcevicp.a

# follow instructions in README to install in /usr/local
LDFLAGS		:= $(shell sdl2-config --libs) \
			   /usr/local/lib/libcmdapp.a \
			   /usr/local/lib/libsdlwrapper.a \
			   /usr/local/lib/libconfig.a
CFLAGS		+= $(shell sdl2-config --cflags) \
			   -I/usr/local/include/cmdapp \
			   -I/usr/local/include/config \
			   -I/usr/local/include/sdlwrapper \
			   -I/usr/local/include/simple_test \
			   -I/usr/local/include/eigen3

# CFLAGS 		+= $(CRELEASE)
CFLAGS 		+= $(CDEBUG)

SRC			:= $(shell find $(SRCDIR) -name "*.cpp")
OBJ			:= $(SRC:.cpp=.o)
DEPS 		:= $(OBJS:.o=.d) 


LIBSRC		:= $(shell find $(SRCDIR)/icp -name "*.cpp" -type f) \
			   $(shell find $(SRCDIR)/algo -name "*.cpp" -type f)
LIBOBJ		:= $(LIBSRC:.cpp=.o)

-include $(DEPS)

# Config parameters
N		:= 1
METHOD	:= trimmed

$(TARGET): main.cpp $(OBJ)
	$(CC) $(CFLAGS) $^ $(LDFLAGS) -o $@
	@make readme

# .PHONY: run
# run: $(TARGET)
# 	./$(TARGET)

.PHONY: test
test: test.cpp $(OBJ)
	@$(CC) $(CFLAGS) -DTEST -o _temp $^ $(LDFLAGS)
	@echo 'Running tests...'
	@./_temp
	@rm -f ./_temp

.PHONY: aux
aux: $(TARGET) 
	./$(TARGET) -mvanilla

.PHONY: view
view: $(TARGET)
	./$(TARGET) -S ex_data/scan$(N)/first.conf -D ex_data/scan$(N)/second.conf --method $(METHOD) --gui

.PHONY: bench
bench: $(TARGET) 
	./$(TARGET) -S ex_data/scan$(N)/first.conf -D ex_data/scan$(N)/second.conf --method $(METHOD) --bench

%.o: %.cpp
	@echo 'Compiling $@'
	$(CC) $(CFLAGS) -MMD -MP $< -c -o $@

.PHONY: clean
clean:
	rm -rf $(OBJ) $(TARGET) $(TARGET) $(DEPS) $(shell find . -name "*.dSYM") $(shell find . -name "*.d") docs

.PHONY: docs 
docs:
	@make readme
	$(PY) script/icp_doc_builder.py src/icp/ book/icp_descr/
	cd book; \
		pdflatex icp.tex; \
		rm *.aux *.log *.out
	doxygen
	mv book/icp.pdf docs
	cp book/desmos.txt docs

.PHONY: cloc
cloc:
	cloc . --include-lang=c++,"c/c++ header" --by-file

.PHONY: readme
readme:
	cd script; $(PY) readme.py `curl https://api.github.com/repos/cornellev/icp/releases/latest | jq .name`

.PHONY: math
math:
	@cd math; $(PY) ./icp_math.py

ifeq ($(shell uname), Darwin)
AR 		:= /usr/bin/libtool
AR_OPT 	:= -static
else
AR 		:= ar
AR_OPT 	:= rcs $@ $^
endif

$(LIBNAME): $(LIBOBJ)
	$(AR) $(AR_OPT) $^ -o $@
