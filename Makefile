APP_NAME=ublox_localtime
PREFIX ?= /usr/local
INSTALL_BIN ?= $(PREFIX)/bin/
INSTALL ?= install

CFLAGS += -MMD -MP -std=c99 -g -Og -Wall -pedantic -D_DEFAULT_SOURCE
LDFLAGS += -g
LIBS +=

ifeq ($(DEBUG),1)
	TARGET = debug
	CFLAGS += -O0 -funwind-tables -fasynchronous-unwind-tables -DDEBUG
	CFLAGS += -fsanitize=address,leak,undefined,float-divide-by-zero,float-cast-overflow,shift,integer-divide-by-zero,unreachable,vla-bound,null,return,signed-integer-overflow,bounds,bounds-strict
	LDFLAGS += -lasan -lubsan

	ifeq ($(ARCH),arm)
		CFLAGS += -mapcs-frame
	else
		CFLAGS += -fno-omit-frame-pointer
	endif
else
	TARGET = release
	CFLAGS += -DNDEBUG
endif

OBJDIR = .build/$(TARGET)
SOURCE := $(wildcard *.c)
OBJECTS := $(patsubst %.c,$(OBJDIR)/%.o,$(SOURCE))

all: $(APP_NAME)

run: all
	faketime '1970-01-01 00:00:00' ./$(APP_NAME) -d /dev/gps?

$(APP_NAME): .target $(OBJECTS)
	$(CC) $(LDFLAGS) -o $(APP_NAME) $(OBJECTS) $(LIBS)

$(OBJDIR)/%.o: %.c Makefile .target | create_obj_dir
	$(CC) $(CFLAGS) -o $@ -c $<

check: $(APP_NAME)
	cppcheck --enable=all -q .

dis: $(APP_NAME)
	objdump -S $(APP_NAME) | less

clean:
	rm -rf .build .target

distclean: clean
	rm -f $(APP_NAME) .gdb_history

install: all
	$(INSTALL) --directory $(INSTALL_BIN)
	$(INSTALL) $(APP_NAME) -m755 $(INSTALL_BIN)

uninstall:
	rm -f $(INSTALL_BIN)/$(APP_NAME)

help:
	@echo "all, install, clean, distclean  ... the usual"
	@echo "run   ... run under faketime control to avoid contamination with system time"
	@echo "check ... check code with cppcheck"
	@echo "dis   ... open disassembly in pager"
	@echo
	@echo "make DEBUG=1     (doesn't work with 'make run' because of faketime's LD_PRELOAD)"
	@echo

create_obj_dir:
	@mkdir -p $(OBJDIR)

.target: FORCE
	@test "$$(cat $@ 2>/dev/null)" = "$(TARGET): $(CFLAGS); $(LDFLAGS)" || echo "$(TARGET): $(CFLAGS); $(LDFLAGS)" >$@

FORCE:

-include $(OBJS:.o=.d)
