OBJECTS += $(patsubst %.c,%.o,$(wildcard src/*.c))

CPPFLAGS += -Iinclude -I. -I../
CFLAGS += -fPIC

LDFLAGS += -L../gimxlog
LDLIBS += -lgimxlog

LDFLAGS += -lusb-1.0

ifeq ($(OS),Windows_NT)
LDFLAGS += -L../gimxtimer -L../gimxpoll
LDLIBS += -lgimxtimer -lgimxpoll
endif

include Makedefs
