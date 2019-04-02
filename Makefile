# For parameters details check src/Makefile
# Example: build no forth (usb to 3 uarts brifge only) for Maple Mini
#          and flash it using st-flash utility
# make FORTH=0 BOARD=maplemini flash

ifneq ($(V), 1)
MFLAGS += --no-print-dir
Q := @
endif

TARGETS = TARGETS='stm32/f1'
# TARGETS = TARGETS='stm32/f1 stm32/f4'

all:
	$(Q)if [ ! -f libopencm3/Makefile ]; then \
		echo "Initialising git submodules..." ;\
		git submodule init ;\
		git submodule update ;\
	fi
	$(Q)$(MAKE) $(TARGETS) $(MFLAGS) -C libopencm3 lib
	$(Q)$(MAKE) $(MFLAGS) -C src

clean:
	$(Q)$(MAKE) $(MFLAGS) -C libopencm3 $@
	$(Q)$(MAKE) $(MFLAGS) -C src $@

purge:	clean
	$(Q)echo "  PURGE"
	-$(Q)$(RM) -rf libopencm3 mecrisp-stellaris

flash:
	$(Q)$(MAKE) $(MFLAGS) -C src $@
