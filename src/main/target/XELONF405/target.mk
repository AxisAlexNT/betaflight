F405_TARGETS    += $(TARGET)
FEATURES        += VCP SDCARD MSC

TARGET_SRC = \
            drivers/accgyro/accgyro_mpu.c \
            drivers/accgyro/accgyro_mpu6500.c \
            drivers/compass/compass_ak8963.c \
            drivers/accgyro/accgyro_fake.c \
            drivers/accgyro/accgyro_mpu.c \
            drivers/accgyro/accgyro_spi_mpu6500.c \
            drivers/barometer/barometer_bmp280.c \
            drivers/rangefinder/rangefinder_hcsr04.c \



TARGET_SRC += \
            msc/usbd_storage_sd_spi.c \
            msc/usbd_msc_desc.c
