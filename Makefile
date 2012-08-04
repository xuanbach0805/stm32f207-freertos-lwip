# Optimization level, can be [0, 1, 2, 3, s]. 
#     0 = turn off optimization. s = optimize for size.
# 

OPT = 0

# Object files directory
# Warning: this will be removed by make clean!
#
OBJDIR = obj

# Target file name (without extension)
TARGET = $(OBJDIR)/freegate

# Define all C source files (dependencies are generated automatically)
#
SOURCES += src/uart.c
SOURCES += src/ustime.c
SOURCES += src/main.c
SOURCES += src/startup_stm32f2xx.s
SOURCES += src/system_stm32f2xx.c
SOURCES += src/syscalls.c

SOURCES += ../FreeRTOS/Source/tasks.c
SOURCES += ../FreeRTOS/Source/queue.c
SOURCES += ../FreeRTOS/Source/list.c
SOURCES += ../FreeRTOS/Source/croutine.c
SOURCES += ../FreeRTOS/Source/portable/GCC/ARM_CM3/port.c 

SOURCES += ../MyARMLib/STM32/STM32F2xx_StdPeriph_Driver/src/misc.c
#SOURCES += ./MyARMLib/STM32/STM32F2xx_StdPeriph_Driver/src/stm32f2xx_adc.c
SOURCES += ../MyARMLib/STM32/STM32F2xx_StdPeriph_Driver/src/stm32f2xx_can.c
SOURCES += ../MyARMLib/STM32/STM32F2xx_StdPeriph_Driver/src/stm32f2xx_crc.c
#SOURCES += ./MyARMLib/STM32F4xx_StdPeriph_Driver/src/stm32f2xx_cryp.c
#SOURCES += ./MyARMLib/STM32F4xx_StdPeriph_Driver/src/stm32f2xx_cryp_aes.c
#SOURCES += ./MyARMLib/STM32F4xx_StdPeriph_Driver/src/stm32f2xx_cryp_des.c
#SOURCES += ./MyARMLib/STM32F4xx_StdPeriph_Driver/src/stm32f2xx_cryp_tdes.c
#SOURCES += ./MyARMLib/STM32F4xx_StdPeriph_Driver/src/stm32f2xx_dac.c
#SOURCES += ./MyARMLib/STM32F4xx_StdPeriph_Driver/src/stm32f2xx_dbgmcu.c
#SOURCES += li./MyARMLibbs/STM32F4xx_StdPeriph_Driver/src/stm32f2xx_dcmi.c
SOURCES += ../MyARMLib/STM32/STM32F2xx_StdPeriph_Driver/src/stm32f2xx_dma.c
SOURCES += ../MyARMLib/STM32/STM32F2xx_StdPeriph_Driver/src/stm32f2xx_exti.c
#SOURCES += libs/STM32F4xx_StdPeriph_Driver/src/stm32f2xx_flash.c
#SOURCES += libs/STM32F4xx_StdPeriph_Driver/src/stm32f2xx_fsmc.c
SOURCES += ../MyARMLib/STM32/STM32F2xx_StdPeriph_Driver/src/stm32f2xx_gpio.c
#SOURCES += libs/STM32F4xx_StdPeriph_Driver/src/stm32f2xx_hash.c
#SOURCES += libs/STM32F4xx_StdPeriph_Driver/src/stm32f2xx_hash_md5.c
#SOURCES += libs/STM32F4xx_StdPeriph_Driver/src/stm32f2xx_hash_sha1.c
#SOURCES += libs/STM32F4xx_StdPeriph_Driver/src/stm32f2xx_i2c.c
#SOURCES += libs/STM32F4xx_StdPeriph_Driver/src/stm32f2xx_iwdg.c
SOURCES += ../MyARMLib/STM32/STM32F2xx_StdPeriph_Driver/src/stm32f2xx_pwr.c
SOURCES += ../MyARMLib/STM32/STM32F2xx_StdPeriph_Driver/src/stm32f2xx_rcc.c
#SOURCES += libs/STM32F4xx_StdPeriph_Driver/src/stm32f2xx_rng.c
SOURCES += ../MyARMLib/STM32/STM32F2xx_StdPeriph_Driver/src/stm32f2xx_rtc.c
SOURCES += ../MyARMLib/STM32/STM32F2xx_StdPeriph_Driver/src/stm32f2xx_sdio.c
#SOURCES += libs/STM32F4xx_StdPeriph_Driver/src/stm32f2xx_spi.c
SOURCES += ../MyARMLib/STM32/STM32F2xx_StdPeriph_Driver/src/stm32f2xx_syscfg.c
SOURCES += ../MyARMLib/STM32/STM32F2xx_StdPeriph_Driver/src/stm32f2xx_tim.c
SOURCES += ../MyARMLib/STM32/STM32F2xx_StdPeriph_Driver/src/stm32f2xx_usart.c
#SOURCES += libs/STM32F4xx_StdPeriph_Driver/src/stm32f2xx_wwdg.c

#lwip
SOURCES += $(addprefix libs/lwip/src/api/,api_lib.c api_msg.c err.c netbuf.c netdb.c netifapi.c sockets.c tcpip.c)
SOURCES += $(addprefix libs/lwip/src/core/ipv4/,autoip.c icmp.c igmp.c ip4_addr.c ip4.c ip_frag.c)
SOURCES += $(addprefix libs/lwip/src/core/,def.c dhcp.c dns.c inet_chksum.c init.c mem.c memp.c netif.c pbuf.c raw.c stats.c sys.c tcp.c tcp_in.c tcp_out.c timers.c udp.c)
SOURCES += ../MyARMLib/lwip/src/netif/etharp.c

SOURCES += src/netconf.c
SOURCES += ../MyARMLib/lwip_port/sys_arch.c
SOURCES += ../MyARMLib/lwip_port/ethernetif.c
#SOURCES += src/usart_if.c
SOURCES += src/eth/stm32f2x7_eth.c
SOURCES += src/eth/stm32f2x7_eth_bsp.c
SOURCES += src/eth/stm32f2x7_eth_irq.c

#qei
#SOURCES += src/qei.c

#shell
#SOURCES += src/shell/shell.c

#rs485 gate
#SOURCES += src/gate_rs485.c

OBJECTS  = $(addprefix $(OBJDIR)/,$(addsuffix .o,$(basename $(SOURCES))))

# Place -D, -U or -I options here for C and C++ sources
CPPFLAGS += -I./inc
CPPFLAGS += -I../FreeRTOS/Source/include
CPPFLAGS += -I../FreeRTOS/Source/portable/GCC/ARM_CM3
CPPFLAGS += -I../MyARMLib/CMSIS/include
CPPFLAGS += -I../MyARMLib/STM32/STM32F2xx/include
CPPFLAGS += -I../MyARMLib/STM32/STM32F2xx_StdPeriph_Driver/inc
CPPFLAGS += -I../MyARMLib/lwip/src/include
CPPFLAGS += -I../MyARMLib/lwip/src/include/ipv4
CPPFLAGS += -I../MyARMLib/lwip/src/include/ipv6
CPPFLAGS += -I../MyARMLib/lwip_port
CPPFLAGS += -I../MyARMLib/lwip/src/include/lwip
CPPFLAGS += -Isrc/shell
CPPFLAGS += -Isrc/eth


#---------------- Compiler Options C ----------------
#  -g*:          generate debugging information
#  -O*:          optimization level
#  -f...:        tuning, see GCC documentation
#  -Wall...:     warning level
#  -Wa,...:      tell GCC to pass this to the assembler.
#    -adhlns...: create assembler listing
CFLAGS  = -O$(OPT)
CFLAGS += -std=gnu99
#CFLAGS += -gdwarf-2
CFLAGS += -ggdb3
CFLAGS += -ffunction-sections
CFLAGS += -fdata-sections
CFLAGS += -Wall
#CFLAGS += -Wextra
#CFLAGS += -Wpointer-arith
#CFLAGS += -Wstrict-prototypes
#CFLAGS += -Winline
#CFLAGS += -Wunreachable-code
#CFLAGS += -Wundef
CFLAGS += -g3
CFLAGS += -Wa,-adhlns=$(OBJDIR)/$(*F).lst

# Optimize use of the single-precision FPU
#
######CFLAGS += -fsingle-precision-constant

# Misc defines
CFLAGS += -DGCC_ARMCM4F
CFLAGS += -DUSE_STDPERIPH_DRIVER
#CFLAGS += -DHSE_VALUE=\(\(uint32_t\)8000000\)
#CFLAGS += -DVECT_TAB_SRAM

# misc defines
# CFLAGS += -DNULL_DBG

# This will not work without recompiling libs
#
# CFLAGS += -fshort-double

#---------------- Compiler Options C++ ----------------
#
CXXFLAGS  = $(CFLAGS)

#---------------- Assembler Options ----------------
#  -Wa,...:   tell GCC to pass this to the assembler
#  -adhlns:   create listing
#
ASFLAGS = -Wa,-adhlns=$(OBJDIR)/$(*F).lst


#---------------- Linker Options ----------------
#  -Wl,...:     tell GCC to pass this to linker
#    -Map:      create map file
#    --cref:    add cross reference to  map file
LDFLAGS += -lm
LDFLAGS += -Wl,-Map=$(TARGET).map,--cref
LDFLAGS += -Wl,--gc-sections
#without bootloader
LDFLAGS += -Tsrc/stm32_flash.ld
#with bootloader
#LDFLAGS += -Tsrc/stm32_flash_bootloader.ld
#CFLAGS += -DUSE_BOOTLOADER

#============================================================================


# Define programs and commands
TOOLCHAIN = arm-none-eabi
CC        = $(TOOLCHAIN)-gcc
OBJCOPY   = $(TOOLCHAIN)-objcopy
OBJDUMP   = $(TOOLCHAIN)-objdump
SIZE      = $(TOOLCHAIN)-size
NM        = $(TOOLCHAIN)-nm
OPENOCD   = openocd
DOXYGEN   = doxygen
STLINK    = tools/ST-LINK_CLI.exe


ifeq (AMD64, $(PROCESSOR_ARCHITEW6432))
  SUBWCREV = tools/SubWCRev64.exe
else
  SUBWCREV = tools/SubWCRev.exe
endif


# Compiler flags to generate dependency files
GENDEPFLAGS = -MMD -MP -MF $(OBJDIR)/$(*F).d


# Combine all necessary flags and optional flags
# Add target processor to flags.
#
CPU = -mcpu=cortex-m3 -mthumb -mfloat-abi=soft
#CPU = -mcpu=cortex-m4 -mthumb 
#

#CPU = -mcpu=cortex-m4 -mthumb -mfloat-abi=softfp -mfpu=fpv4-sp-d16
#CPU = -march=armv7-m -mthumb -mfloat-abi=softfp -mfpu=fpv4-sp-d16

CFLAGS   += $(CPU)
CXXFLAGS += $(CPU)
ASFLAGS  += $(CPU)
LDFLAGS  += $(CPU)

# Default target.
all:  gccversion createdirs build showsize

build: elf hex lss sym bin

elf: $(TARGET).elf
hex: $(TARGET).hex
lss: $(TARGET).lss
sym: $(TARGET).sym
bin: $(TARGET).bin


doxygen:
	@echo
	@echo Creating Doxygen documentation
	@$(DOXYGEN)

# Display compiler version information
gccversion: 
	@$(CC) --version

createdirs:
	mkdir -p obj/libs/lwip/src/api
	mkdir -p obj/libs/lwip/src/core/ipv4
	mkdir -p obj/libs/lwip_port
	mkdir -p obj/libs/lwip/src/netif

# Show the final program size
showsize: elf
	@echo
	@$(SIZE) $(TARGET).elf 2>/dev/null


# Flash the device
flash: elf
	$(OPENOCD) -s /usr/share/openocd/scripts -f board/stm32f4discovery.cfg -c "flash write_image sss.elf 0 elf;"
	#$(STLINK) -c SWD -P $(TARGET).hex -Run


# Target: clean project
clean:
	@echo Cleaning project:
	rm -rf $(OBJDIR)
	rm -rf docs/html


# Create extended listing file from ELF output file
%.lss: %.elf
	@echo
	@echo Creating Extended Listing: $@
	$(OBJDUMP) -h -S -z $< > $@


# Create a symbol table from ELF output file
%.sym: %.elf
	@echo
	@echo Creating Symbol Table: $@
	$(NM) -n $< > $@


# Link: create ELF output file from object files
.SECONDARY: $(TARGET).elf
.PRECIOUS:  $(OBJECTS)
$(TARGET).elf: $(OBJECTS)
	@echo
	@echo Linking: $@
	$(CC) $^ $(LDFLAGS) --output $@ 


# Create final output files (.hex, .eep) from ELF output file.
%.hex: %.elf
	@echo
	@echo Creating hex file: $@
	$(OBJCOPY) -O ihex $< $@

# Create final output files (.bin) from ELF output file.
%.bin: %.elf
	@echo
	@echo Creating bin file: $@
	$(OBJCOPY) -O binary $< $@

# Compile: create object files from C source files
$(OBJDIR)/%.o : %.c
	@echo
	@echo Compiling C: $<
	$(CC) -c $(CPPFLAGS) $(CFLAGS) $(GENDEPFLAGS) $< -o $@ 


# Compile: create object files from C++ source files
$(OBJDIR)/%.o : %.cpp
	@echo
	@echo Compiling CPP: $<
	$(CC) -c $(CPPFLAGS) $(CXXFLAGS) $(GENDEPFLAGS) $< -o $@ 


# Assemble: create object files from assembler source files
$(OBJDIR)/%.o : %.s
	@echo
	@echo Assembling: $<
	$(CC) -c $(CPPFLAGS) $(ASFLAGS) $< -o $@


# Create object file directories
$(shell mkdir -p $(OBJDIR) 2>/dev/null)
$(shell mkdir -p $(OBJDIR)/src 2>/dev/null)
$(shell mkdir -p $(OBJDIR)/src/shell 2>/dev/null)
$(shell mkdir -p $(OBJDIR)/src/eth 2>/dev/null)
$(shell mkdir -p $(OBJDIR)/FreeRTOS/Source/ 2>/dev/null)
$(shell mkdir -p $(OBJDIR)/FreeRTOS/Source/portable/GCC/ARM_CM4F 2>/dev/null)
$(shell mkdir -p $(OBJDIR)/libs/STM32F4xx_StdPeriph_Driver/src 2>/dev/null)

# Include the dependency files
-include $(wildcard $(OBJDIR)/*.d)


# Listing of phony targets
.PHONY: all build flash clean \
        doxygen elf lss sym \
        showsize gccversion
