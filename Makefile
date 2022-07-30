# Generic Makefile for compiling LIRC plugins out of tree.
# Copyright: public domain

# Following two or three lines should be adjusted

# Where are the LIRC sources located (needed for include files only)
LIRC_SRC=/usr/include/lirc

# Where are out plugins to be installed
PLUGINDIR := $(shell pkg-config --variable=plugindir lirc-driver)

# Where are plugin docs to be installed
PLUGINDOCSDIR := $(shell pkg-config --variable=plugindocs lirc-driver)

# Where are configs to be installed
CONFIGDIR := $(shell pkg-config --variable=configdir lirc-driver)

# Some extra includes and/or libraries might be needed
EXTRA_INCLUDES := $(shell pkg-config libusb lirc-driver --cflags)
EXTRA_LIBS := $(shell pkg-config libusb lirc-driver --libs)

MACHINE := -m64
INCLUDE := -I$(LIRC_SRC)/lib -I$(LIRC_SRC) $(EXTRA_INCLUDES)
OPTIMIZE := -O2
DEBUG := -g
SHARED :=-shared -fPIC
WARNINGS=-Wall
CC := gcc
CPP := g++

# Rule for compiling C
%.so: %.c
	@echo PLUGINDIR=$(PLUGINDIR)
	@echo EXTRA_INCLUDES=$(EXTRA_INCLUDES)
	@echo EXTRA_LIBS=$(EXTRA_LIBS)
	@echo CC=$(CC)
	$(CC)  $(WARNINGS) $(INCLUDE) $(MACHINE) $(OPTIMIZE) $(DEBUG) $(SHARED) -o $@ $< $(EXTRA_LIBS)

# Rule for compiling C++
%.so: %.cpp
	$(CPP) $(WARNINGS) $(INCLUDE) $(MACHINE) $(OPTIMIZE) $(DEBUG) $(SHARED) -o $@ $< $(EXTRA_LIBS)

default:
	@echo "There is no default target in this makefile."
	@echo "Type \"make plugin.so\" to compile the plugin named plugin,"
	@echo "and \"make install\" to install it"

install:
	install -Dm755 *.so $(DESTDIR)$(PLUGINDIR)
	install -Dm755 *.conf $(DESTDIR)$(CONFIGDIR)

clean:
	rm -f *.so
