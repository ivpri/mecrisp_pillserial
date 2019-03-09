ifneq ($(V), 1)
MFLAGS += --no-print-dir
Q := @
endif

TARGETS = TARGETS=stm32/f1

all:
	$(Q)if [ ! -f libopencm3/Makefile ]; then \
		echo "Initialising git submodules..." ;\
		git submodule init ;\
		git submodule update ;\
	fi
	$(Q)$(MAKE) $(TARGETS) $(MFLAGS) -C libopencm3 lib
	$(Q)$(MAKE) $(TARGETS) $(MFLAGS) -C src

clean:
	$(Q)$(MAKE) $(MFLAGS) -C libopencm3 $@
	$(Q)$(MAKE) $(MFLAGS) -C src $@

