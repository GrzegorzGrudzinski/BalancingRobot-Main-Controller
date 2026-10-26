# SiL (software in the loop): the firmware controller core built for the PC, without HAL.
# The sources come straight from ../code/Core, so the PC runs the same code as the STM32.
#
#   make -C sil test    build and run the unit tests (test_core.c)
#   make -C sil lib     build libcontrol_core.so for Python (control_core.py, simulator)
#   make -C sil check   compile only: the core (with controllers_config.c) must not need HAL
#   make -C sil clean   remove the build outputs
CORE   := ../code/Core
INC    := -I$(CORE)/Inc/App/control -I$(CORE)/Inc/App/config -I.
SRC    := $(CORE)/Src/App/control/pid.c $(CORE)/Src/App/control/cascade_pid.c \
          $(CORE)/Src/App/control/pid_wrapper.c $(CORE)/Src/App/control/control.c
CFLAGS := -std=c11 -O2 -Wall -Wextra -Werror $(INC)

.PHONY: all lib test check clean
all: lib

# Shared library for ctypes; only the CTRL_API functions are exported (-fvisibility=hidden)
lib: libcontrol_core.so
libcontrol_core.so: $(SRC) control_api.c control_api.h
	$(CC) $(CFLAGS) -fPIC -fvisibility=hidden -shared $(SRC) control_api.c -o $@ -lm

test: test_core
	./test_core
test_core: $(SRC) control_api.c test_core.c
	$(CC) $(CFLAGS) $^ -o $@ -lm

# Compile only: proves that the core, together with controllers_config.c, needs neither HAL nor the protocol
check:
	$(CC) $(CFLAGS) -fsyntax-only $(SRC) $(CORE)/Src/App/control/controllers_config.c

clean:
	rm -f libcontrol_core.so test_core
