# Copyright (C) 2023 Ethan Uppal. All rights reserved.

SRCDIR		:= src
INCLUDEDIR	:= src
LIBDIR		:= $(SRCDIR)/sdl-wrapper
LIB			:= $(LIBDIR)/libsdlwrapper.so

CC			:= $(shell which g++-13 || \
					   which g++ || which clang)
CFLAGS		:= -std=c++17 -pedantic -Wall -Wextra -I $(INCLUDEDIR)
CDEBUG		:= -g
CRELEASE	:= -O2 -DRELEASE_BUILD
TARGET		:= main.out
TARGETSH	:= main

LDFLAGS		:= -L $(LIBDIR) -lsdlwrapper -I $(LIBDIR)/src -L /usr/local/lib/ -lcmdapp
CFLAGS		+= -I /usr/local/include

# CFLAGS 		+= $(CRELEASE)
CFLAGS 		+= $(CDEBUG)

# use SDL
CFLAGS		+= $(shell sdl2-config --cflags --libs)

# use sdlwrapper and cmdapp2
CFLAGS		+=  $(LDFLAGS)

SRC			:= main.cpp $(shell find $(SRCDIR) -name "*.cpp" -type f -not -path "$(SRCDIR)/sdl-wrapper/*")
OBJ			:= $(SRC:.cpp=.o)
DEPS 		:= $(OBJS:.o=.d) 

-include $(DEPS)

.PHONY: driver
driver: $(LIB) /usr/local/lib/libcmdapp.a
	make $(TARGET)
	@printf '#!/bin/bash\nDYLD_LIBRARY_PATH=$$DYLD_LIBRARY_PATH:$(shell echo $$PWD)/$(LIBDIR) ./$(TARGET) "$$@"\n' > $(TARGETSH)
	@chmod u+x ./$(TARGETSH)

.PHONY: run 
run: driver 
	./$(TARGETSH) --gui	

.PHONY: gui_debug
gui_debug: driver
	echo "run" | lldb $(TARGET) -- -g

# inv: $(LIB) is built
$(TARGET): $(OBJ)
	$(CC) $(CFLAGS) $^ -o $@

$(LIB):
	cd $(LIBDIR); make libsdlwrapper.so

%.o: %.cpp $(LIB)
	@echo 'Compiling $@'
	$(CC) $(CFLAGS) -MMD -MP $< -c -o $@

.PHONY: clean
clean:
	rm -rf $(OBJ) $(TARGET) $(TARGET2) $(DEPS) $(shell find . -name "*.dSYM") $(shell find . -name "*.d")
	cd $(LIBDIR); make clean
