CFLAGS		?= -O2 -pipe -g3
CXXFLAGS	?= $(CFLAGS)
WARN_FLAGS	 = -Wall -Wextra -Werror -Wcast-align -Wno-unused-parameter \
		   -Wno-missing-field-initializers -Wstrict-prototypes \
		   -Werror-implicit-function-declaration -Wundef
CFLAGS		+= -fPIC $(WARN_FLAGS)
CXXFLAGS	+= -fPIC $(WARN_FLAGS)
CPPFLAGS	+= -I.. -D__USER__ -include out-of-tree-autoconf.h
CC		 = $(CROSS_COMPILE)gcc
CXX		 = $(CROSS_COMPILE)g++

LIB		= libmcp2210.so
LIB_OBJ_FILES	= mcp2210-lib.o mcp2210-user.o
UTIL_OBJ_FILES	= mcp2210.o tests.o
ALL_OBJ_FILES	= $(LIB_OBJ_FILES) $(UTIL_OBJ_FILES)
HEADER_FILES	= mcp2210-user.h settings.h ../mcp2210.h ../mcp2210-creek.h \
		  ../mcp2210-debug.h ../out-of-tree-autoconf.h
SYMLINK_FILES	= mcp2210-lib.c

all: $(LIB) mcp2210-util

links: $(SYMLINK_FILES)

mcp2210-lib.c:
	ln -fs ../mcp2210-lib.c .

$(ALL_OBJ_FILES): $(HEADER_FILES)

$(LIB): $(LIB_OBJ_FILES)
	$(CC) $(CFLAGS) $(LIB_OBJ_FILES) -shared -o $(LIB)

# Create settings.h from example file if it doesn't exist
settings.h:
	cp -n settings-example.h settings.h

# FIXME: mcp2210 target doesn't need -fPIC in cc on obj files & link
mcp2210-util: links $(UTIL_OBJ_FILES) $(HEADER_FILES) $(LIB)
	$(CC) $(CFLAGS) $(UTIL_OBJ_FILES) -lmcp2210 -L. -o mcp2210-util

.PHONY : clean
clean:
	rm -f $(SYMLINK_FILES) *.o mcp2210 libmcp2210.so



