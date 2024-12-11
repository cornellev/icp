# Copyright (C) 2023 Ethan Uppal. All rights reserved.
BUILD_DIR := build
LIB_TARGET := cevicp
MAIN_TARGET := main
TEST_TARGET := test_suite

N := 1
METHOD := vanilla

OPT := Debug
LIB_INSTALL := /usr/local/lib
HEADER_INSTALL := /usr/local/include
CMAKE_FLAGS := -DCMAKE_BUILD_TYPE=$(OPT) -DCMAKE_INSTALL_LIBDIR=$(LIB_INSTALL) -DCMAKE_INSTALL_INCLUDEDIR=$(HEADER_INSTALL)

MAKE_FLAGS := -j $(shell nproc)

.PHONY: build_all
build_all: configure
	cmake --build $(BUILD_DIR) -- $(MAKE_FLAGS)

.PHONY: configure
configure:
	cmake -S . -B $(BUILD_DIR) $(CMAKE_FLAGS)

.PHONY: $(LIB_TARGET)
$(LIB_TARGET): configure
	cmake --build $(BUILD_DIR) --target $(LIB_TARGET) -- $(MAKE_FLAGS)

.PHONY: $(MAIN_TARGET)
$(MAIN_TARGET): configure
	cmake --build $(BUILD_DIR) --target $(MAIN_TARGET) -- $(MAKE_FLAGS)

.PHONY: $(TEST_TARGET)
$(TEST_TARGET): configure
	cmake --build $(BUILD_DIR) --target $(TEST_TARGET) -- $(MAKE_FLAGS)
	
.PHONY: test
test: $(TEST_TARGET)
	./$(BUILD_DIR)/$(TEST_TARGET)

.PHONY: view
view: $(MAIN_TARGET)
	./$(BUILD_DIR)/$(MAIN_TARGET) -S ex_data/scan$(N)/first.conf -D ex_data/scan$(N)/second.conf --method $(METHOD) --gui
	
.PHONY: bench
bench: $(MAIN_TARGET)
	./$(BUILD_DIR)/$(MAIN_TARGET) -S ex_data/scan$(N)/first.conf -D ex_data/scan$(N)/second.conf --method $(METHOD) --bench

.PHONY: clean
clean: configure
	cmake --build $(BUILD_DIR) --target clean

.PHONY: install
install: configure $(LIB_TARGET)
	cmake --install $(BUILD_DIR)

.PHONY: uninstall
uninstall: configure
	cmake --build $(BUILD_DIR) --target uninstall

SCRIPT_DIR := script
RUN_SCRIPT := cd $(SCRIPT_DIR); uv venv; source .venv/bin/activate; uv sync; python3

.PHONY: docs 
docs:
	$(RUN_SCRIPT) icp_doc_builder.py ../lib/icp ../book/icp_descr/ ../book/main.md
	doxygen

.PHONY: cloc
cloc:
	cloc $(INCLUDE_DIR) $(LIB_DIR) $(SRC_DIR) --include-lang=c++,"c/c++ header" --by-file

.PHONY: math
math:
	$(RUN_SCRIPT) icp_math.py
