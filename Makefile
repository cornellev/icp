# Copyright (C) 2023 Ethan Uppal. All rights reserved.

INCLUDEDIR	:= include
LIBDIR		:= lib
SRCDIR		:= src
TESTDIR		:= tests

CC			:= $(shell which g++ || which clang++)
PY			:= $(shell which python3 || which python)
CFLAGS		:= -std=c++17 -pedantic -Wall -Wextra -I $(INCLUDEDIR)
CDEBUG		:= -g
CRELEASE	:= -O3 -DRELEASE_BUILD
LIBNAME		:= libcevicp.a
MAINNAME	:= main
TESTNAME	:= test

ifeq ($(OPT), RELEASE)
CFLAGS 		+= $(CRELEASE)
else
CFLAGS 		+= $(CDEBUG)
endif

LIBSRC		:= $(shell find $(LIBDIR) -name "*.cpp" -type f)
LIBINCLUDE  := -I/usr/include/eigen3
LIBOBJ		:= $(LIBSRC:.cpp=.o)
LIBDEPS		:= $(LIBOBJ:.o=.d)
$(LIBNAME): CFLAGS += $(LIBINCLUDE)

LIBINSTALL 		:= /usr/local/lib
HEADERINSTALL	:= /usr/local/include
INSTALLNAME		:= cev_icp

MAINSRC		:= $(shell find $(SRCDIR) -name "*.cpp" -type f)
MAININCLUDE := $(shell sdl2-config --cflags) \
				-I/usr/include/eigen3 \
			   	-I/usr/local/include/cmdapp \
			   	-I/usr/local/include/config \
			   	-I/usr/local/include/sdlwrapper
MAINLD 		:= $(shell sdl2-config --libs) \
			   	/usr/local/lib/libcmdapp.a \
			   	/usr/local/lib/libsdlwrapper.a \
			   	/usr/local/lib/libconfig.a
MAINOBJ		:= $(MAINSRC:.cpp=.o)
MAINDEPS	:= $(MAINOBJ:.o=.d)
$(MAINNAME): CFLAGS += $(MAININCLUDE)
$(MAINNAME): LDFLAGS += $(MAINLD)

TESTSRC		:= $(shell find $(TESTDIR) -name "*.cpp" -type f)
TESTINCLUDE := -I/usr/include/eigen3 \
			   -I/usr/local/include/simple_test
TESTOBJ		:= $(TESTSRC:.cpp=.o)
TESTDEPS	:= $(TESTOBJ:.o=.d)
$(TESTNAME): CFLAGS += $(TESTINCLUDE)
$(TESTNAME): CFLAGS += -DTEST

-include $(LIBDEPS)
-include $(MAINDEPS)
-include $(TESTDEPS)

N		:= 3
METHOD	:= vanilla

ifeq ($(shell uname), Darwin)
AR 		:= /usr/bin/libtool
AR_OPT 	:= -static
else
AR 		:= ar
AR_OPT 	:= rcs $@ $^
endif

$(LIBNAME): $(LIBOBJ)
	$(AR) $(AR_OPT) -o $@ $^

$(MAINNAME): $(MAINOBJ) $(LIBNAME)
	$(CC) $(CFLAGS) $^ $(LDFLAGS) -o $@

$(TESTNAME): $(TESTOBJ) $(LIBNAME)
	@$(CC) $(CFLAGS) $^ $(LDFLAGS) -o $@
	@./$(TESTNAME)
	@rm $(TESTNAME)

%.o: %.cpp
	@echo 'Compiling $@'
	$(CC) $(CFLAGS) -MMD -MP $< -c -o $@

.PHONY: clean
clean:
	@rm -f $(LIBOBJ) $(LIBDEPS) $(LIBNAME) $(MAINOBJ) $(MAINDEPS) $(MAINNAME)

.PHONY: view
view: $(MAINNAME)
	./$(MAINNAME) -S ex_data/scan$(N)/first.conf -D ex_data/scan$(N)/second.conf --method $(METHOD) --gui

.PHONY: bench
bench: $(MAINNAME)
	./$(MAINNAME) -S ex_data/scan$(N)/first.conf -D ex_data/scan$(N)/second.conf --method $(METHOD) --bench

.PHONY: install
install: $(LIBNAME)
	mkdir -p $(LIBINSTALL)
	mkdir -p $(HEADERINSTALL)/$(INSTALLNAME)
	cp $(LIBNAME) $(LIBINSTALL)
	cp -r $(INCLUDEDIR)/* $(HEADERINSTALL)/$(INSTALLNAME)

.PHONY: uninstall
uninstall:
	rm -r $(LIBINSTALL)/$(LIBNAME) $(HEADERINSTALL)/$(INSTALLNAME)

# Not building book rn, add these commands to build
# cd book; \
  pdflatex icp.tex; \
  rm *.aux *.log *.out \
  mv book/icp.pdf docs
.PHONY: docs 
docs:
	@make readme
	$(PY) script/icp_doc_builder.py src/icp/ book/icp_descr/
	doxygen
	cp book/desmos.txt docs

.PHONY: cloc
cloc:
	cloc . --include-lang=c++,"c/c++ header" --by-file

.PHONY: readme
readme:
	cd script; $(PY) readme.py

.PHONY: math
math:
	@cd math; $(PY) ./icp_math.py
