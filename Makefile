override BOARD = capybara
export BOARD

export BOARD_MAJOR = 1
export BOARD_MINOR = 1


#export MEASURE

TOOLS = \

TOOLCHAINS = \
	gcc \
	clang \

include ext/maker/Makefile

# Paths to toolchains if different from defaults

TOOLCHAIN_ROOT = /opt/ti/mspgcc3
