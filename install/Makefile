# Copyright (C) 2024 Ethan Uppal.
# TODO: move to main makefile

LIB		:= /usr/local/lib
HEADER	:= /usr/local/include
NAME	:= cev_icp
LIBNAME	:= libcevicp.a

.PHONY: install
install:
	cd ..; make $(LIBNAME); mv ./$(LIBNAME) install
	mkdir -p $(LIB)
	mkdir -p $(HEADER)/$(NAME)
	mv $(LIBNAME) $(LIB)
	cd ..; \
		cp -r src/icp $(HEADER)/$(NAME)/icp; \
		cp -r src/algo $(HEADER)/$(NAME)/algo

.PHONY: uninstall
uninstall:
	rm -rf $(LIB)/$(LIBNAME) $(HEADER)/$(NAME)
