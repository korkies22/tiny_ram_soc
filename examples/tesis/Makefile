FIRMWARE_DIR = ../../firmware
HDL_DIR = ../../hdl
INCLUDE_DIR = ../../libraries
VERILOG_FILES = \
           $(HDL_DIR)/top.v \
           $(HDL_DIR)/clock.v \
           $(HDL_DIR)/pwm.v \
           $(HDL_DIR)/encoder.v \
           $(HDL_DIR)/picosoc/uart/simpleuart.v \
           $(HDL_DIR)/picosoc/picosoc.v \
           $(HDL_DIR)/picorv32/picorv32.v \
           $(HDL_DIR)/picosoc/spi_master/spi_master.v \
           $(HDL_DIR)/picosoc/gpio/gpio.v \

PCF_FILE = $(HDL_DIR)/pcbsd.pcf
LDS_FILE = $(FIRMWARE_DIR)/firmware.lds
START_FILE = $(FIRMWARE_DIR)/firmware.S
C_FILES = main.c \
          $(INCLUDE_DIR)/uart/uart.c

include $(HDL_DIR)/tiny_soc.mk

