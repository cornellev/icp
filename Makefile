# Copyright (C) 2023 Ethan Uppal. All rights reserved.
INCLUDE_DIR := include
LIB_DIR := lib
SRC_DIR := src
TEST_DIR := tests

CC := $(shell which g++ || which clang++)
CFLAGS := -std=c++17 -pedantic -Wall -Wextra -I $(INCLUDE_DIR)
CDEBUG := -g
CRELEASE := -O3 -DRELEASE_BUILD
LIB_NAME := libcevicp.a
MAIN_NAME := main
TEST_NAME := test

ifeq ($(OPT), RELEASE)
CFLAGS += $(CRELEASE)
else
CFLAGS += $(CDEBUG)
endif

LIB_SRC := $(shell find $(LIB_DIR) -name "*.cpp" -type f)
LIB_INCLUDE := -I/usr/include/eigen3
LIB_OBJ := $(LIB_SRC:.cpp=.o)
LIB_DEPS := $(LIB_OBJ:.o=.d)
$(LIB_NAME): CFLAGS += $(LIB_INCLUDE)

LIB_INSTALL := /usr/local/lib
HEADER_INSTALL := /usr/local/include
INSTALL_NAME := cev_icp

MAIN_SRC := $(shell find $(SRC_DIR) -name "*.cpp" -type f)
MAIN_INCLUDE := $(shell sdl2-config --cflags) \
				-I/opt/homebrew/Cellar/eigen/3.4.0_1/include/eigen3 \
				-I/usr/local/include/cmdapp \
				-I/usr/local/include/config \
				-I/usr/local/include/sdlwrapper
MAIN_LD := $(shell sdl2-config --libs) \
			/usr/local/lib/libcmdapp.a \
			/usr/local/lib/libsdlwrapper.a \
			/usr/local/lib/libconfig.a
MAIN_OBJ := $(MAIN_SRC:.cpp=.o)
MAIN_DEPS := $(MAIN_OBJ:.o=.d)
$(MAIN_NAME): CFLAGS += $(MAIN_INCLUDE)
$(MAIN_NAME): LDFLAGS += $(MAIN_LD)

TEST_SRC := $(shell find $(TEST_DIR) -name "*.cpp" -type f)
TEST_INCLUDE := -I/opt/homebrew/Cellar/eigen/3.4.0_1/include/eigen3 \
				-I/usr/local/include/simple_test
TEST_OBJ := $(TEST_SRC:.cpp=.o)
TEST_DEPS := $(TEST_OBJ:.o=.d)
$(TEST_NAME): CFLAGS += $(TEST_INCLUDE)
$(TEST_NAME): CFLAGS += -DTEST

-include $(LIB_DEPS)
-include $(MAIN_DEPS)
-include $(TEST_DEPS)

N := 3
METHOD := trimmed

ifeq ($(shell uname), Darwin)
AR := /usr/bin/libtool
AR_OPT := -static
else
AR := ar
AR_OPT := rcs $@ $^
endif

$(LIB_NAME): $(LIB_OBJ)
	$(AR) $(AR_OPT) -o $@ $^

$(MAIN_NAME): $(MAIN_OBJ) $(LIB_NAME)
	$(CC) $(CFLAGS) $^ $(LDFLAGS) -o $@

$(TEST_NAME): $(TEST_OBJ) $(LIB_NAME)
	@$(CC) $(CFLAGS) $^ $(LDFLAGS) -o $@
	@./$(TEST_NAME)
	@rm $(TEST_NAME)

%.o: %.cpp
	@echo 'Compiling $@'
	$(CC) $(CFLAGS) -MMD -MP $< -c -o $@

.PHONY: clean
clean:
	@rm -f $(LIB_OBJ) $(LIB_DEPS) $(LIB_NAME) $(MAIN_OBJ) $(MAIN_DEPS) $(MAIN_NAME) $(TEST_OBJ) $(TEST_DEPS) $(TEST_NAME)

.PHONY: view
view: $(MAIN_NAME)
	./$(MAIN_NAME) -S ex_data/scan$(N)/first.conf -D ex_data/scan$(N)/second.conf --method $(METHOD) --gui

.PHONY: bench
bench: $(MAIN_NAME)
	./$(MAIN_NAME) -S ex_data/scan$(N)/first.conf -D ex_data/scan$(N)/second.conf --method $(METHOD) --bench

.PHONY: install
install: $(LIB_NAME)
	mkdir -p $(LIB_INSTALL)
	mkdir -p $(HEADER_INSTALL)/$(INSTALL_NAME)
	cp $(LIB_NAME) $(LIB_INSTALL)
	cp -r $(INCLUDE_DIR)/* $(HEADER_INSTALL)/$(INSTALL_NAME)

.PHONY: uninstall
uninstall:
	rm -r $(LIB_INSTALL)/$(LIB_NAME) $(HEADER_INSTALL)/$(INSTALL_NAME)

# Not building book rn, add these commands to build
# cd book; \
  pdflatex icp.tex; \
  rm *.aux *.log *.out \
  mv book/icp.pdf docs

SCRIPT_DIR := script
RUN_SCRIPT := cd $(SCRIPT_DIR); uv venv; source .venv/bin/activate; uv sync; python3

.PHONY: docs 
docs:
	@make readme
	$(RUN_SCRIPT) icp_doc_builder.py ../lib/icp ../book/icp_descr/ ../book/main.md
	doxygen

.PHONY: cloc
cloc:
	cloc $(INCLUDE_DIR) $(LIB_DIR) $(SRC_DIR) --include-lang=c++,"c/c++ header" --by-file

.PHONY: readme
readme:
	$(RUN_SCRIPT) readme.py

.PHONY: math
math:
	$(RUN_SCRIPT) icp_math.py
