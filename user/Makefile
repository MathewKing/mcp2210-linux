#CROSS_COMPILE	=/usr/bin/armv6j-hardfloat-linux-gnueabi-
CONFIG		= -DCONFIG_MCP2210_SPI=1 -DCONFIG_MCP2210_GPIO=1 \
		  -DCONFIG_MCP2210_EEPROM=1 -DCONFIG_MCP2210_DEBUG=1 \
		  -DCONFIG_MCP2210_DEBUG_VERBOSE=1 -DCONFIG_MCP2210_CREEK=1
CFLAGS		+= -O0 -pipe -g3 -fPIC
CPPFLAGS	+= -I.. $(CONFIG)
WARN_FLAGS	 = -Wall -Wextra -Werror -Wcast-align -Wno-unused-parameter -Wno-missing-field-initializers -Werror-implicit-function-declaration -Wstrict-prototypes -Wundef
# -Wno-pointer-sign -Wno-trigraphs
# -Wno-unused-variable
CFLAGS		+= $(WARN_FLAGS)
CC		= $(CROSS_COMPILE)gcc
CXX		= $(CROSS_COMPILE)g++

LIB		= libmcp2210.so
LIB_OBJ_FILES	= mcp2210-lib.o mcp2210-user.o
UTIL_OBJ_FILES	= mcp2210.o tests.o
ALL_OBJ_FILES	= $(LIB_OBJ_FILES) $(UTIL_OBJ_FILES)
HEADER_FILES	= mcp2210-user.h ../mcp2210.h ../mcp2210-creek.h ../mcp2210-debug.h
SYMLINK_FILES	= mcp2210-lib.c

all: $(LIB) mcp2210

links: $(SYMLINK_FILES)

mcp2210-lib.c:
	ln -fs ../mcp2210-lib.c .

$(ALL_OBJ_FILES): $(HEADER_FILES)

$(LIB): $(LIB_OBJ_FILES)
	$(CC) $(CFLAGS) $(LIB_OBJ_FILES) -shared -o $(LIB)

# FIXME: mcp2210 target doesn't need -fPIC in cc on obj files & link
mcp2210: links $(UTIL_OBJ_FILES) $(HEADER_FILES) $(LIB)
	$(CC) $(CXXFLAGS) $(UTIL_OBJ_FILES) -lmcp2210 -L. -o mcp2210

.PHONY : clean
clean:
	rm -f $(SYMLINK_FILES) *.o mcp2210 libmcp2210.so



