# Copyright (C) 2023 Ethan Uppal.
# SPDX-License-Identifier: MIT

BUILD_DIR := build
LIB_TARGET := cevicp
MAIN_TARGET := main
TEST_TARGET := test_suite
BENCH_TARGET := bench_suite
TEST3D_TARGET := test_suite_3d
TEST_PLY_TARGET := test_suite_ply
TEST_PLY_INPUT_A := ex_data/ply/queen_transformed.ply
TEST_PLY_INPUT_B := ex_data/ply/queen.ply
TEST_PLY_OUTPUT := ex_data/ply/queen_result.ply

N := 1
METHOD := vanilla

OPT := Debug

LIB_INSTALL := /usr/local/lib
HEADER_INSTALL := /usr/local/include
CMAKE_FLAGS := -DCMAKE_BUILD_TYPE=$(OPT) -DCMAKE_INSTALL_LIBDIR=$(LIB_INSTALL) -DCMAKE_INSTALL_INCLUDEDIR=$(HEADER_INSTALL) -DCMAKE_EXPORT_COMPILE_COMMANDS=ON

MAKE_FLAGS := -j $(shell nproc || sysctl -n hw.logicalcpu)

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

.PHONY: $(BENCH_TARGET)
$(BENCH_TARGET): configure
	cmake --build $(BUILD_DIR) --target $(BENCH_TARGET) -- $(MAKE_FLAGS)

.PHONY: $(TEST3D_TARGET)
$(TEST3D_TARGET): configure
	cmake --build $(BUILD_DIR) --target $(TEST3D_TARGET) -- $(MAKE_FLAGS)

.PHONY: $(TEST_PLY_TARGET)
$(TEST_PLY_TARGET): configure
	cmake --build $(BUILD_DIR) --target $(TEST_PLY_TARGET) -- $(MAKE_FLAGS)

.PHONY: test
test: $(TEST_TARGET)
	./$(BUILD_DIR)/$(TEST_TARGET)

.PHONY: test_ply
test_ply: $(TEST_PLY_TARGET)
	./$(BUILD_DIR)/$(TEST_PLY_TARGET) $(TEST_PLY_INPUT_A) $(TEST_PLY_INPUT_B) $(TEST_PLY_OUTPUT)
	
.PHONY: test_3d
test_3d: $(TEST3D_TARGET)
	./$(BUILD_DIR)/$(TEST3D_TARGET)

.PHONY: view
view: $(MAIN_TARGET)
	./$(BUILD_DIR)/$(MAIN_TARGET) -S ex_data/scan$(N)/first.csv -D ex_data/scan$(N)/second.csv --method $(METHOD)
	
.PHONY: bench
bench: $(BENCH_TARGET)
	DYLD_LIBRARY_PATH=/usr/local/lib ./$(BUILD_DIR)/$(BENCH_TARGET)

.PHONY: tidy
tidy: configure  # needed for compile commands database
	@if [ -z "$(CI)" ]; then \
		find . -iname '*.h' -o -iname '*.cpp' \
		-o -path ./$(BUILD_DIR) -prune -false \
		-o -path ./script -prune -false \
		-o -path ./test -prune -false \
		| xargs clang-tidy -p ./$(BUILD_DIR); \
	else \
		find . -iname '*.h' -o -iname '*.cpp' \
		-o -path ./$(BUILD_DIR) -prune -false \
		-o -path ./script -prune -false \
		-o -path ./test -prune -false \
		| xargs clang-tidy -p ./$(BUILD_DIR) -warnings-as-errors='*'; \
	fi

.PHONY: clean
clean: configure
	cmake --build $(BUILD_DIR) --target clean

.PHONY: install
install: configure $(LIB_TARGET)
	cmake --install $(BUILD_DIR)

.PHONY: uninstall
uninstall: configure
	cmake --build $(BUILD_DIR) --target uninstall

INCLUDE_DIR := include
LIB_DIR := lib
VIS_DIR := vis
SCRIPT_DIR := script
RUN_SCRIPT := cd $(SCRIPT_DIR); uv venv; source .venv/bin/activate; uv sync; python3

.PHONY: docs 
docs:
	$(RUN_SCRIPT) icp_doc_builder.py ../$(LIB_DIR)/icp ../book/icp_descr/ ../book/main.md
	doxygen

.PHONY: cloc
cloc:
	cloc $(INCLUDE_DIR) $(LIB_DIR) $(VIS_DIR) --include-lang=c++,"c/c++ header" --by-file

.PHONY: math
math:
	$(RUN_SCRIPT) icp_math.py
