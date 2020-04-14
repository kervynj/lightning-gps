# makefile for Compiling uClinux user-space application on STM32F7 target
# Includes STM32 standard peripheral driver library

# Cross compile settings  
CC = /home/jim/Documents/STM32F769I-disco_Buildroot/buildroot/output/host/usr/bin/arm-buildroot-uclinux-uclibcgnueabi-gcc
CFLAGS =  -mlittle-endian #--specs=nosys.specs
CFLAGS += -mthumb -mcpu=cortex-m4 -msoft-float 
CFLAGS += -mthumb-interwork -mfloat-abi=soft
CFLAGS += -mfpu=fpv4-sp-d16
#CFLAGS += -I/home/jim/workspace/stm32f429_mainline_linux/linux-5.6-rc5/usr/include/
CFLAGS += -L../../stm32f4_uClinux_std-periph-driver_env/uclibc/lib/libc.a
CFLAGS += -Wall
LDFLAGS = -static

# header files dependencies, add include files here
#DEPS = gpio_control.h stm32f4xx_gpio.h stm32f4xx_rcc.h stm32f4xx.h system_stm32f4xx.h stm32f4xx_conf.h 
OBJ = gps.o 

# Compile C files before creating object files
#%.o: %.c $(DEPS)
%.o: %.c
	$(CC) -c -o $@ $< $(CFLAGS) $(LDFLAGS)

# rule to generate executable
gps: $(OBJ)
	$(CC) -o $@ $^ $(CFLAGS) $(LDFLAGS)

#rule to clean
clean: 
	rm *.o
