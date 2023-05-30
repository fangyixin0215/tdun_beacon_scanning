
BUILD_DIR = build
TARGET    = BK3435_APP_CENTRAL

AR = arm-none-eabi-gcc-ar
CC = arm-none-eabi-gcc

CCFLAGS  = -Os -mcpu=arm968e-s -march=armv5te -mthumb -mthumb-interwork -ffunction-sections -fdata-sections
CCFLAGS += -flto -fno-strict-aliasing -fgnu89-inline -ffast-math -fmessage-length=0
CCFLAGS += -Wall
CCFLAGS += -DCONFIG_AOS_CLI

LKFLAGS  = -Os -mcpu=arm968e-s -mthumb -mthumb-interwork -flto -ffunction-sections -fdata-sections
LKFLAGS += -nostartfiles

CCFLAGS += -I./config \
      -I./app \
      -I../../sdk/plactform/arch \
      -I../../sdk/plactform/arch/compiler \
      -I../../sdk/plactform/arch/ll \
      -I../../sdk/plactform/arch/boot \
      -I../../sdk/plactform/arch/main \
      -I../../sdk/plactform/driver/pwm \
      -I../../sdk/plactform/driver/adc \
      -I../../sdk/plactform/driver/audio \
      -I../../sdk/plactform/driver/wdt \
      -I../../sdk/plactform/driver/rtc \
      -I../../sdk/plactform/driver/i2c \
      -I../../sdk/plactform/driver/utc \
      -I../../sdk/plactform/driver/ir \
      -I../../sdk/plactform/driver/spi \
      -I../../sdk/plactform/driver/plf \
      -I../../sdk/plactform/driver/counter \
      -I../../sdk/plactform/driver/gpio \
      -I../../sdk/plactform/driver/intcntl \
      -I../../sdk/plactform/driver/icu \
      -I../../sdk/plactform/driver/intc \
      -I../../sdk/plactform/driver/flash \
      -I../../sdk/plactform/driver/timer \
      -I../../sdk/plactform/driver/reg \
      -I../../sdk/plactform/driver/uart \
      -I../../sdk/plactform/driver/emi \
      -I../../sdk/ble_stack/central/com/rwble \
      -I../../sdk/ble_stack/central/com/rwble_hl \
      -I../../sdk/ble_stack/central/com/rwble \
      -I../../sdk/ble_stack/central/com/rwip/api \
      -I../../sdk/ble_stack/central/com/rwble_hl \
      -I../../sdk/ble_stack/central/src \
      -I../../sdk/plactform/reg \
      -I../../sdk/plactform/driver/syscntl \
      -I../../sdk/plactform/rom/hci \
      -I../../sdk/ble_stack/central/inc \
      -I../../sdk/ble_stack/central/inc/hci \
      -I../../sdk/ble_stack/central/inc/h4tl \
      -I../../sdk/ble_stack/central/inc/ke \
      -I../../sdk/ble_stack/central/inc/nvds \
      -I../../sdk/ble_stack/central/inc/ea \
      -I../../sdk/ble_stack/central/inc/em \
      -I../../sdk/ble_stack/central/inc/ahi \
      -I../../sdk/ble_stack/central/inc/ble/hl/gap/gapc \
      -I../../sdk/ble_stack/central/inc/ble/hl/gap/gapm \
      -I../../sdk/ble_stack/central/inc/ble/hl/gap/smpc \
      -I../../sdk/ble_stack/central/inc/ble/hl/gap/smpm \
      -I../../sdk/ble_stack/central/inc/ble/hl/gatt/attc \
      -I../../sdk/ble_stack/central/inc/ble/hl/gatt/attm \
      -I../../sdk/ble_stack/central/inc/ble/hl/gatt/atts \
      -I../../sdk/ble_stack/central/inc/ble/hl/gatt/gattc \
      -I../../sdk/ble_stack/central/inc/ble/hl/gatt/gattm \
      -I../../sdk/ble_stack/central/inc/ble/hl/l2c/l2cc \
      -I../../sdk/ble_stack/central/inc/ble/hl/l2c/l2cm \
      -I../../sdk/ble_stack/central/inc/ble/ll/em \
      -I../../sdk/ble_stack/central/inc/ble/ll/llc \
      -I../../sdk/ble_stack/central/inc/ble/ll/lld \
      -I../../sdk/ble_stack/central/inc/ble/ll/llm \
      -I../../sdk/ble_stack/central/com/rwip/api \
      -I../../sdk/ble_stack/central/inc/ble/hl/gap \
      -I../../sdk/ble_stack/central/inc/ble/hl/gatt \
      -I../../sdk/ble_stack/common/prf \
      -I../../sdk/ble_stack/common/profiles/dis/diss/src \
      -I../../sdk/ble_stack/common/profiles/bas/bass/src \
      -I../../sdk/ble_stack/common/profiles/ancs/ancsc/api \
      -I../../sdk/ble_stack/common/profiles/ancs/ancsc/src \
      -I../../sdk/ble_stack/common/profiles/ancs \
      -I../../sdk/ble_stack/common/profiles/FCC0/api \
      -I../../sdk/ble_stack/common/profiles/FCC0/src \
      -I../../sdk/ble_stack/common/profiles/FCC0/util \
      -I../../sdk/ble_stack/common/profiles/FEE0/api \
      -I../../sdk/ble_stack/common/profiles/FEE0/src \
      -I../../sdk/ble_stack/common/profiles/hogp \
      -I../../sdk/ble_stack/common/profiles/hogp/hogpd/api \
      -I../../sdk/ble_stack/common/profiles/hogp/hogpd/src \
      -I../../libs \
      -I../../sdk/ble_stack/common/profiles/wechat/api \
      -I../../sdk/ble_stack/common/profiles/wechat/src \
      -I./wechat \
      -I../../sdk/ble_stack/common/profiles/bas/bass \
      -I../../sdk/ble_stack/common/profiles/bas/bass/api \
      -I../../sdk/ble_stack/common/profiles/dis/diss \
      -I../../sdk/ble_stack/common/profiles/dis/diss/api \
      -I../../sdk/ble_stack/common/profiles/FFF0/api \
      -I../../sdk/ble_stack/common/profiles/FFF0 \
      -I../../sdk/ble_stack/common/profiles/oad/api \
      -I../../sdk/ble_stack/common/profiles/sdp/api \
      -I../../sdk/ble_stack/common/profiles/sdp/src \
      -I../../sdk/plactform/com/api \
      -I../../sdk/plactform/driver/rf \
      -I../../sdk/ble_stack/central/com/rwble \
      -I../../sdk/ble_stack/central/com/rwble_hl \
      -I../../sdk/ble_stack/central/com/rwip/api \
      -I../../sdk/ble_stack/central/inc/dbg \
      -I../../sdk/plactform/util \
      -I../../sdk/plactform/driver/usb \
      -I../../sdk/plactform/driver/usb/include \
      -I../../sdk/plactform/driver/usb/include/class \
      -I../../sdk/plactform/driver/usb/src/cd \
      -I../../sdk/plactform/driver/usb/src/drivers/comm \
      -I../../sdk/plactform/driver/usb/src/drivers/msd \
      -I../../sdk/plactform/driver/usb/src/examples \
      -I../../sdk/plactform/driver/usb/src/functions/trans_fn \
      -I../../sdk/plactform/driver/usb/src/systems/none \
      -I../../sdk/plactform/driver/usb/src/systems/none/afs \
      -I../../sdk/plactform/driver/usb/src/test \
      -I../../sdk/plactform/driver/usb/src/test/msd \
      -I../../sdk/ble_stack/central/com/prf \
      -I../../sdk/ble_stack/common/profiles/dis/disc/api


LKFLAGS += -L../../libs
LKFLAGS += -T../../sdk/project_files/link_app.lds
#LKFLAGS += -Wl,--gc-sections
LKFLAGS += -Wl,--just-symbols=../../sdk/project_files/BK3435_CENTER_STACK.elf
LKFLAGS += --specs=nosys.specs

ASMS = 
ASM1 = 
ASM2 = sdk/plactform/arch/boot/startup_boot.S
SRCS = app/arch_main.c \
       app/app.c \
       app/app_sdp.c \
       app/app_task.c \
       app/central_api.c \
       app/app_sec.c \
       app/app_wlist.c
SRC1 = 	
SRC2 = sdk/ble_stack/central/com/rwble_hl/rwble_hl.c \
       sdk/ble_stack/central/com/rwble/rwble.c \
       sdk/ble_stack/central/com/rwip/src/rwip.c \
       sdk/plactform/driver/icu/icu.c \
       sdk/plactform/driver/intc/intc.c \
       sdk/plactform/driver/uart/uart.c \
       sdk/plactform/driver/emi/emi.c \
       sdk/plactform/driver/flash/flash.c \
       sdk/plactform/driver/timer/timer.c \
       sdk/plactform/driver/rf/rf_xvr.c \
       sdk/plactform/driver/wdt/wdt.c \
       sdk/plactform/driver/gpio/gpio.c \
	   sdk/plactform/driver/utc/utc_clock.c \
	   sdk/plactform/driver/usb/driver_usb.c \
       sdk/plactform/driver/usb/src/cd/mu_cntlr.c \
       sdk/plactform/driver/usb/src/cd/mu_descs.c \
       sdk/plactform/driver/usb/src/cd/mu_drc.c \
       sdk/plactform/driver/usb/src/cd/mu_fadma.c \
       sdk/plactform/driver/usb/src/cd/mu_fc.c \
       sdk/plactform/driver/usb/src/cd/mu_fdr.c \
       sdk/plactform/driver/usb/src/cd/mu_fun.c \
       sdk/plactform/driver/usb/src/cd/mu_funex.c \
       sdk/plactform/driver/usb/src/cd/mu_hc.c \
       sdk/plactform/driver/usb/src/cd/mu_hdr.c \
       sdk/plactform/driver/usb/src/cd/mu_hst.c \
       sdk/plactform/driver/usb/src/cd/mu_list.c \
       sdk/plactform/driver/usb/src/cd/mu_pip.c \
       sdk/plactform/driver/usb/src/drivers/msd/mu_mapi.c \
       sdk/plactform/driver/usb/src/drivers/msd/mu_mbot.c \
       sdk/plactform/driver/usb/src/drivers/msd/mu_mscsi.c \
       sdk/plactform/driver/usb/src/examples/msd/mu_msdfn.c \
       sdk/plactform/driver/usb/src/functions/trans_fn/mu_tfunc.c \
       sdk/plactform/driver/usb/src/lib/mu_bits.c \
       sdk/plactform/driver/usb/src/lib/mu_ramfl.c \
       sdk/plactform/driver/usb/src/lib/mu_stack.c \
       sdk/plactform/driver/usb/src/lib/mu_stdio.c \
       sdk/plactform/driver/usb/src/lib/mu_strng.c \
       sdk/plactform/driver/usb/src/systems/none/plat_uds.c \
       sdk/plactform/driver/usb/src/systems/none/afs/board.c \
       sdk/plactform/driver/usb/src/test/msd/tst_msd.c \
	   sdk/ble_stack/common/profiles/sdp/src/sdp_service.c \
	   sdk/ble_stack/common/profiles/sdp/src/sdp_service_task.c \
       sdk/ble_stack/central/com/prf/prf.c \
       sdk/ble_stack/central/com/prf/prf_utils.c \
	   libs/sbc_decoder.c \
	   libs/sbc_common.c \
	   sdk/plactform/util/enc_key.c \
	   sdk/plactform/util/util_encrypt.c
	   
OBJS := $(SRCS:%.c=$(BUILD_DIR)/%.o)
OBJS += $(SRC1:%.c=$(BUILD_DIR)/dd1/%.o)
OBJS += $(SRC2:%.c=$(BUILD_DIR)/dd2/%.o)
OBJS += $(ASMS:%.S=$(BUILD_DIR)/%.o)
OBJS += $(ASM1:%.S=$(BUILD_DIR)/dd1/%.o)
OBJS += $(ASM2:%.S=$(BUILD_DIR)/dd2/%.o)
DEPS := $(OBJS:.o=.d)

target: $(BUILD_DIR)/$(TARGET).elf $(BUILD_DIR)/$(TARGET).bin

$(BUILD_DIR)/$(TARGET).elf : $(OBJS)
	@echo Build target: $@
	@$(CC) $(LKFLAGS) -o $@ $(OBJS) $(LKLIBS)
	@echo 'Finished building target: $@'
	@echo ' '

%.bin : %.elf
	@echo Build target: $@
	@arm-none-eabi-objcopy -O binary $< $@
	@arm-none-eabi-objdump -date     $<    > $(<:%.elf=%.dmp)
	@arm-none-eabi-nm                $<    > $(<:%.elf=%.map)
	
ifeq ($(OS),Windows_NT)	
	@arm-none-eabi-readelf.exe       $< -a > $(<:%.elf=%.txt)
	@encrc -bc -o $(@:%.bin=%_crc.bin) $@
	@cp -R ./build/BK3435_APP_CENTRAL.bin ./output/app
	@./output/BinConvert_3435_OAD.exe  -oad ./output/bim/BK3435_BIM.bin  ./output/stack/BK3435_CENTER_STACK.bin ./output/app/BK3435_APP_CENTRAL.bin -m 0x1F00 -l 0x1f200 -v 0x1220 -rom_v 0x1235 -e 00000000 00000000 00000000 00000000
else
	@arm-none-eabi-readelf           $< -a > $(<:%.elf=%.txt)
	@cp -R ./build/BK3435_APP_CENTRAL.bin ./output/app
	@./BinConvert_3435_OAD_Linux  -oad ./output/bim/BK3435_BIM.bin  ./output/stack/BK3435_CENTER_STACK.bin ./output/app/BK3435_APP_CENTRAL.bin -m 0x1F00 -l 0x1f200 -v 0x1220 -rom_v 0x1235 -e 00000000 00000000 00000000 00000000
endif

	@echo 'Finished building target: $@'
	@echo ' '

clean:
	@echo Clean project: $(TARGET)
	@-rm -rf $(OBJS) $(DEPS) $(BUILD_DIR)/$(TARGET).elf $(BUILD_DIR)/$(TARGET).bin
	-@echo ' '

prev-build:
	-@echo 'Pre-building'
	-@rm -rf $(BUILD_DIR)/$(TARGET)*.*
	-@echo ' '

post-build:
	-@echo 'Post-building'
	-@echo ' '

$(BUILD_DIR)/%.o : %.c
	@echo Compiling $<
	@mkdir -p $(dir $@)
	@$(CC) -c $(CCFLAGS) -MMD -MP -MF$(@:%.o=%.d) -MT$(@) $< -o $@

$(BUILD_DIR)/dd1/%.o : ../%.c
	@echo Compiling $<
	@mkdir -p $(dir $@)
	@$(CC) -c $(CCFLAGS) -MMD -MP -MF$(@:%.o=%.d) -MT$(@) $< -o $@

$(BUILD_DIR)/dd2/%.o : ../../%.c
	@echo Compiling $<
	@mkdir -p $(dir $@)
	@$(CC) -c $(CCFLAGS) -MMD -MP -MF$(@:%.o=%.d) -MT$(@) $< -o $@

$(BUILD_DIR)/%.o : %.S
	@echo Compiling $<
	@mkdir -p $(dir $@)
	@$(CC) -c $(CCFLAGS) -MMD -MP -MF$(@:%.o=%.d) -MT$(@) $< -o $@

$(BUILD_DIR)/dd1/%.o : ../%.S
	@echo Compiling $<
	@mkdir -p $(dir $@)
	@$(CC) -c $(CCFLAGS) -MMD -MP -MF$(@:%.o=%.d) -MT$(@) $< -o $@

$(BUILD_DIR)/dd2/%.o : ../../%.S
	@echo Compiling $<
	@mkdir -p $(dir $@)
	@$(CC) -c $(CCFLAGS) -MMD -MP -MF$(@:%.o=%.d) -MT$(@) $< -o $@

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(DEPS)),)
-include $(DEPS)
endif
endif

.PHONY : all clean pre-build post-build
