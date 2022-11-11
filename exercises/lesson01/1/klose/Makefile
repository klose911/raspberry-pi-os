uname_M := $(shell sh -c 'uname -m 2>/dev/null || echo not')

ifneq (,$(findstring arm,$(uname_M))) # for arm raspbian 
	ARMGNU := aarch64-linux-gnu
else # for x64 
	ifeq ($(OS),Windows_NT) 
		ARMGNU := aarch64-none-linux-gnu
	else 
		uname_S := $(shell sh -c 'uname -s 2>/dev/null || echo not')

		ifeq ($(uname_S),Linux)
			DISTR_ID := $(shell lsb_release -si)
			ifeq ($(DISTR_ID), Gentoo)
				ARMGNU := aarch64-none-linux-gnu
			else
				ARMGNU := aarch64-linux-gnu
			endif
		endif

		ifeq ($(uname_S),FreeBSD)
			ARMGNU := aarch64-none-elf
		endif	

		ifeq ($(uname_S),NetBSD)
			ARMGNU := aarch64-none-elf
		endif

		ifeq ($(uname_S),Darwin)
			ARMGNU := aarch64-none-elf
		endif
	endif
endif

COPS = -Wall -nostdlib -nostartfiles -ffreestanding -Iinclude -mgeneral-regs-only
ASMOPS = -Iinclude 

BUILD_DIR = build
SRC_DIR = src

all : kernel8.img

clean :
	rm -rf $(BUILD_DIR) *.img 

$(BUILD_DIR)/%_c.o: $(SRC_DIR)/%.c
	mkdir -p $(@D)
	$(ARMGNU)-gcc $(COPS) -MMD -c $< -o $@

$(BUILD_DIR)/%_s.o: $(SRC_DIR)/%.S
	$(ARMGNU)-gcc $(ASMOPS) -MMD -c $< -o $@

C_FILES = $(wildcard $(SRC_DIR)/*.c)
ASM_FILES = $(wildcard $(SRC_DIR)/*.S)
OBJ_FILES = $(C_FILES:$(SRC_DIR)/%.c=$(BUILD_DIR)/%_c.o)
OBJ_FILES += $(ASM_FILES:$(SRC_DIR)/%.S=$(BUILD_DIR)/%_s.o)

DEP_FILES = $(OBJ_FILES:%.o=%.d)
-include $(DEP_FILES)

kernel8.img: $(SRC_DIR)/linker.ld $(OBJ_FILES)
	$(ARMGNU)-ld -T $(SRC_DIR)/linker.ld -o $(BUILD_DIR)/kernel8.elf  $(OBJ_FILES)
	$(ARMGNU)-objcopy $(BUILD_DIR)/kernel8.elf -O binary kernel8.img
