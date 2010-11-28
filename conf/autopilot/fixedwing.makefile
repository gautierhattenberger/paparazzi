# Hey Emacs, this is a -*- makefile -*-
#
# fixedwing.makefile
#
#



CFG_FIXEDWING=$(PAPARAZZI_SRC)/conf/autopilot/subsystems/fixedwing


SRC_FIXEDWING=.
SRC_ARCH=arch/$(ARCH)
SRC_FIXEDWING_TEST=$(SRC_FIXEDWING)/

SRC_FIRMWARE=firmwares/fixedwing
SRC_SUBSYSTEMS=subsystems

FIXEDWING_INC = -I$(SRC_FIRMWARE) -I$(SRC_FIXEDWING) -I$(SRC_FIXEDWING_ARCH)



# Standard Fixed Wing Code
include $(CFG_FIXEDWING)/autopilot.makefile


ifeq ($(ACTUATORS),)
  ifeq ($(BOARD),tiny)
    ifeq ($(BOARD_VERSION),1.1)
      include $(CFG_FIXEDWING)/actuators_4015.makefile
    else
      ifeq ($(BOARD_VERSION),0.99)
        include $(CFG_FIXEDWING)/actuators_4015.makefile
      else
        include $(CFG_FIXEDWING)/actuators_4017.makefile
      endif
    endif
  endif
  ifeq ($(BOARD),twog)
	include $(CFG_FIXEDWING)/actuators_4017.makefile
  endif

  ifeq ($(BOARD),lisa_l)
    include $(CFG_FIXEDWING)/actuators_direct.makefile
  endif

else
  include $(CFG_FIXEDWING)/$(ACTUATORS).makefile
endif



