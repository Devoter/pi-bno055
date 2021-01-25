#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <fcntl.h>
#include "libbno055.h"

/* ------------------------------------------------------------ *
 * get_i2cbus() - Enables the I2C bus communication. Raspberry  *
 * Pi 2 uses i2c-1, RPI 1 used i2c-0, NanoPi also uses i2c-0.   *
 * ------------------------------------------------------------ */
int get_i2cbus(loglevel_t loglevel, char *i2cbus, char *i2caddr, int* i2cfd) {
   int fd;

   if((fd = open(i2cbus, O_RDWR)) < 0) {
      if (loglevel) printf("Error failed to open I2C bus [%s].\n", i2cbus);
      return 1;
   }

   if(loglevel == level_verbose) printf("Debug: I2C bus device: [%s]\n", i2cbus);

   /* --------------------------------------------------------- *
    * Set I2C device (BNO055 I2C address is  0x28 or 0x29)      *
    * --------------------------------------------------------- */
   int addr = (int)strtol(i2caddr, NULL, 16);

   if (loglevel == level_verbose) printf("Debug: Sensor address: [0x%02X]\n", addr);

   if(ioctl(fd, I2C_SLAVE, addr) != 0) {
      if(loglevel) printf("Error can't find sensor at address [0x%02X].\n", addr);

      close(fd);

      return 2;
   }

   /* --------------------------------------------------------- *
    * I2C communication test is the only way to confirm success *
    * --------------------------------------------------------- */
   char reg = BNO055_CHIP_ID_ADDR;

   if(write(fd, &reg, 1) != 1) {
      if(loglevel) printf("Error: I2C write failure register [0x%02X], sensor addr [0x%02X]?\n", reg, addr);
      close(fd);
      return 3;
   }

   *i2cfd = fd;

   return 0;
}

/* ------------------------------------------------------------ *
 * set_page0() - Set page ID = 0 to set default register access *
 * ------------------------------------------------------------ */
int set_page0(loglevel_t loglevel, int i2cfd) {
   char data[2] = {0};
   data[0] = BNO055_PAGE_ID_ADDR;
   data[1] = 0x0;

   if(loglevel == level_verbose) printf("Debug: write page-ID: [0x%02X] to register [0x%02X]\n", data[1], data[0]);

   if(write(i2cfd, data, 2) != 2) {
      if(loglevel) printf("Error: I2C write failure for register 0x%02X\n", data[0]);
      return 1;
   }
   
   return 0;
}

/* ------------------------------------------------------------ *
 * set_page1() - Set page ID = 1 to switch the register access  *
 * ------------------------------------------------------------ */
int set_page1(loglevel_t loglevel, int i2cfd) {
   char data[2] = {0};
   data[0] = BNO055_PAGE_ID_ADDR;
   data[1] = 0x1;

   if(loglevel == level_verbose) printf("Debug: write page-ID: [0x%02X] to register [0x%02X]\n", data[1], data[0]);

   if(write(i2cfd, data, 2) != 2) {
      if(loglevel) printf("Error: I2C write failure for register 0x%02X\n", data[0]);
      return 1;
   }

   return 0;
}

/* ------------------------------------------------------------ *
 * get_mode() - returns sensor operational mode register 0x3D   *
 * Reads 1 byte from Operations Mode register 0x3d, and uses    *
 * only the lowest 4 bit. Bits 4-7 are unused, stripped off     *
 * ------------------------------------------------------------ */
int get_mode(loglevel_t loglevel, int i2cfd, opmode_t *mode) {
   int reg = BNO055_OPR_MODE_ADDR;

   if(write(i2cfd, &reg, 1) != 1) {
      if(loglevel) printf("Error: I2C write failure for register 0x%02X\n", reg);
      return 1;
   }

   unsigned int data = 0;

   if(read(i2cfd, &data, 1) != 1) {
      if(loglevel) printf("Error: I2C read failure for register data 0x%02X\n", reg);
      return 2;
   }

   if(loglevel == level_verbose) printf("Debug: Operation Mode: [0x%02X]\n", data & 0x0F);

   *mode = data & 0x0F;

   return 0;  // only return the lowest 4 bits
}

/* ------------------------------------------------------------ *
 * set_mode() - set the sensor operational mode register 0x3D   *
 * The modes cannot be switched over directly, first it needs   *
 * to be set to "config" mode before switching to the new mode. *
 * ------------------------------------------------------------ */
int set_mode(loglevel_t loglevel, int i2cfd, opmode_t newmode) {
   char data[2] = {0};
   data[0] = BNO055_OPR_MODE_ADDR;
   opmode_t oldmode;

   int ret_code = get_mode(loglevel, i2cfd, &oldmode);

   if(ret_code) return 1 | (ret_code << 8);

   if(oldmode == newmode) return 0; // if new mode is the same
   else if(oldmode > 0 && newmode > 0) {  // switch to "config" first
      data[1] = 0x0;

      if(loglevel == level_verbose) printf("Debug: Write opr_mode: [0x%02X] to register [0x%02X]\n", data[1], data[0]);

      if(write(i2cfd, data, 2) != 2) {
         if(loglevel) printf("Error: I2C write failure for register 0x%02X\n", data[0]);
         return 2;
      }

      /* --------------------------------------------------------- *
       * switch time: any->config needs 7ms + small buffer = 10ms  *
       * --------------------------------------------------------- */
      usleep(10 * 1000);
   }

   data[1] = newmode;

   if(loglevel == level_verbose) printf("Debug: Write opr_mode: [0x%02X] to register [0x%02X]\n", data[1], data[0]);

   if(write(i2cfd, data, 2) != 2) {
      if(loglevel) printf("Error: I2C write failure for register 0x%02X\n", data[0]);
      return 3;
   }

   /* --------------------------------------------------------- *
    * switch time: config->any needs 19ms + small buffer = 25ms *
    * --------------------------------------------------------- */
   usleep(25 * 1000);

   opmode_t currmode;

   ret_code = get_mode(loglevel, i2cfd, &currmode);

   if(ret_code) return 4 | (ret_code << 8);

   return currmode == newmode ? 0 : 3;
}

/* ------------------------------------------------------------ *
 * get_calstatus() gets the calibration state from the sensor. *
 * Calibration status has 4 values, encoded as 2bit in reg 0x35 *
 * ------------------------------------------------------------ */
int get_calstatus(loglevel_t loglevel, int i2cfd, struct bnocal *bno_ptr) {
   char reg = BNO055_CALIB_STAT_ADDR;

   if(write(i2cfd, &reg, 1) != 1) {
      if(loglevel) printf("Error: I2C write failure for register 0x%02X\n", reg);
      return 1;
   }

   char data = 0;

   if(read(i2cfd, &data, 1) != 1) {
      if(loglevel) printf("Error: I2C read failure for register 0x%02X\n", reg);
      return 2;
   }

   bno_ptr->scal_st = (data & 0b11000000) >> 6; // system calibration status
   bno_ptr->gcal_st = (data & 0b00110000) >> 4; // gyro calibration
   bno_ptr->acal_st = (data & 0b00001100) >> 2; // accel calibration status
   bno_ptr->mcal_st = (data & 0b00000011);      // magneto calibration status

   if(loglevel == level_verbose) {
      printf("Debug: sensor system calibration: [%d]\n\
Debug:     gyroscope calibration: [%d]\n\
Debug: accelerometer calibration: [%d]\n\
Debug:  magnetometer calibration: [%d]\n", bno_ptr->scal_st, bno_ptr->gcal_st,
         bno_ptr->acal_st, bno_ptr->mcal_st);
   }

   return 0;
}

/* ------------------------------------------------------------ *
 * Calibration offset is stored in 3x6 (18) registers 0x55~0x66 *
 * plus 4 registers 0x67~0x6A accelerometer/magnetometer radius *
 * ------------------------------------------------------------ */
int get_caloffset(loglevel_t loglevel, int i2cfd, struct bnocal *bno_ptr) {
   /* --------------------------------------------------------- *
    * Registers may not update in fusion mode, switch to CONFIG *
    * --------------------------------------------------------- */
   opmode_t oldmode;
   int ret_code = get_mode(loglevel, i2cfd, &oldmode);

   if(ret_code) return 1 | (ret_code << 8); // skip current return codes

   ret_code = set_mode(loglevel, i2cfd, config);

   if(ret_code) return 2 | (ret_code << 8); // skip current and get_mode return codes

   char reg = ACC_OFFSET_X_LSB_ADDR;

   if(write(i2cfd, &reg, 1) != 1) {
      if(loglevel) printf("Error: I2C write failure for register 0x%02X\n", reg);
      return 3;
   }

   if(loglevel == level_verbose) printf("Debug: I2C read %d bytes starting at register 0x%02X\n", CALIB_BYTECOUNT, reg);

   char data[CALIB_BYTECOUNT] = {0};

   if(read(i2cfd, data, CALIB_BYTECOUNT) != CALIB_BYTECOUNT) {
      if(loglevel) printf("Error: I2C calibration data read from 0x%02X\n", reg);
      return 4;
   }

   if(loglevel == level_verbose) {
      printf("Debug: Calibrationset:");

      for(int i = 0; i < CALIB_BYTECOUNT; ++i) printf(" %02X", data[i]);

      printf("\n");
   }

   /* ------------------------------------------------------------ *
    * assigning accelerometer X-Y-Z offset, range per G-range      *
    * 16G = +/-16000, 8G = +/-8000, 4G = +/-4000, 2G = +/-2000     *
    * ------------------------------------------------------------ */
   bno_ptr->aoff_x = ((int16_t)data[1] << 8) | data[0];
   bno_ptr->aoff_y = ((int16_t)data[3] << 8) | data[2];
   bno_ptr->aoff_z = ((int16_t)data[5] << 8) | data[4];

   /* ------------------------------------------------------------ *
    * assigning magnetometer X-Y-Z offset, offset range is +/-6400 *
    * ------------------------------------------------------------ */
   bno_ptr->moff_x = ((int16_t)data[7] << 8) | data[6];
   bno_ptr->moff_y = ((int16_t)data[9] << 8) | data[8];
   bno_ptr->moff_z = ((int16_t)data[11] << 8) | data[10];

   /* ------------------------------------------------------------ *
    * assigning gyroscope X-Y-Z offset, range depends on dps value *
    * 2000 = +/-32000, 1000 = +/-16000, 500 = +/-8000, etc         *
    * ------------------------------------------------------------ */
   bno_ptr->goff_x = ((int16_t)data[13] << 8) | data[12];
   bno_ptr->goff_y = ((int16_t)data[15] << 8) | data[14];
   bno_ptr->goff_z = ((int16_t)data[17] << 8) | data[16];

   /* ------------------------------------------------------------ *
    * assigning accelerometer radius, range is +/-1000             *
    * ------------------------------------------------------------ */
   bno_ptr->acc_rad = ((int16_t)data[19] << 8) | data[18];

   /* ------------------------------------------------------------ *
    * assigning magnetometer radius, range is +/-960               *
    * ------------------------------------------------------------ */
   bno_ptr->mag_rad = ((int16_t)data[21] << 8) | data[20];

   if(loglevel == level_verbose) {
      printf("Debug: accelerometer offset: [%d] [%d] [%d] (X-Y-Z)\n\
Debug:  magnetometer offset: [%d] [%d] [%d] (X-Y-Z)\n\
Debug:     gyroscope offset: [%d] [%d] [%d] (X-Y-Z)\n\
Debug: accelerometer radius: [%d] (+/-1000)\n\
Debug:  magnetometer radius: [%d] (+/- 960)\n",
         bno_ptr->aoff_x, bno_ptr->aoff_y, bno_ptr->aoff_z,
         bno_ptr->moff_x, bno_ptr->moff_y, bno_ptr->moff_z,
         bno_ptr->goff_x, bno_ptr->goff_y, bno_ptr->goff_z,
         bno_ptr->acc_rad,
         bno_ptr->mag_rad);
   }
   
   ret_code = set_mode(loglevel, i2cfd, oldmode);

   if(ret_code) return 5 | (ret_code << 8);

   return 0;
}

/* ------------------------------------------------------------ *
 * get_power() returns the sensor power mode from register 0x3e *
 * Only the lowest 2 bit are used, ignore the unused bits 2-7.  *
 * ------------------------------------------------------------ */
int get_power(loglevel_t loglevel, int i2cfd, power_t *mode) {
   int reg = BNO055_PWR_MODE_ADDR;

   if(write(i2cfd, &reg, 1) != 1) {
      if(loglevel) printf("Error: I2C write failure for register 0x%02X\n", reg);
      return 1;
   }

   unsigned int data = 0;
   
   if(read(i2cfd, &data, 1) != 1) {
      if(loglevel) printf("Error: I2C read failure for register data 0x%02X\n", reg);
      return 2;
   }

   if(loglevel == level_verbose) printf("Debug:     Power Mode: [0x%02X] 2bit [0x%02X]\n", data, data & 0x03);

   *mode = data & 0x03; // only return the lowest 2 bits

   return 0;
}

/* ------------------------------------------------------------ *
 * set_power() - set the sensor power mode in register 0x3E.    *
 * The power modes cannot be switched over directly, first the  *
 * ops mode needs to be "config"  to write the new power mode.  *
 * ------------------------------------------------------------ */
int set_power(loglevel_t loglevel, int i2cfd, power_t pwrmode) {
   char data[2] = {0};

   /* ------------------------------------------------------------ *
   * Check what operational mode we are in                        *
   * ------------------------------------------------------------ */
  
   opmode_t oldmode;
   int ret_code = get_mode(loglevel, i2cfd, &oldmode);

   if(ret_code) return 1 | (ret_code << 8);

   /* ------------------------------------------------------------ *
   * If ops mode wasn't config, switch to "CONFIG" mode first     *
   * ------------------------------------------------------------ */
   if(oldmode > 0) {
      data[0] = BNO055_OPR_MODE_ADDR;
      data[1] = 0x0;

      if(loglevel == level_verbose) printf("Debug: Write opr_mode: [0x%02X] to register [0x%02X]\n", data[1], data[0]);

      if(write(i2cfd, data, 2) != 2) {
         if(loglevel) printf("Error: I2C write failure for register 0x%02X\n", data[0]);
         return 2;
      }

      usleep(30 * 1000);
   }  // now we are in config mode

   /* ------------------------------------------------------------ *
   * Set the new power mode                                       *
   * ------------------------------------------------------------ */
   data[0] = BNO055_PWR_MODE_ADDR;
   data[1] = pwrmode;

   if(loglevel == level_verbose) printf("Debug: Write opr_mode: [0x%02X] to register [0x%02X]\n", data[1], data[0]);

   if(write(i2cfd, data, 2) != 2) {
      if(loglevel) printf("Error: I2C write failure for register 0x%02X\n", data[0]);
      return 3;
   }

   usleep(30 * 1000);

   /* ------------------------------------------------------------ *
   * If ops mode wasn't config, switch back to original ops mode  *
   * ------------------------------------------------------------ */
   if(oldmode > 0) {
      data[0] = BNO055_OPR_MODE_ADDR;
      data[1] = oldmode;

      if(loglevel == level_verbose) printf("Debug: Write opr_mode: [0x%02X] to register [0x%02X]\n", data[1], data[0]);

      if(write(i2cfd, data, 2) != 2) {
         if(loglevel) printf("Error: I2C write failure for register 0x%02X\n", data[0]);
         return 4;
      }

      usleep(30 * 1000);
   }  // now the previous mode is back

   power_t currpwrmode;
   
   ret_code = get_power(loglevel, i2cfd, &currpwrmode);

   if(ret_code) return 5 | (ret_code << 8);

   return currpwrmode == pwrmode ? 0 : 6;
}

/* ------------------------------------------------------------ *
 * get_remap() returns axis remap data from registers 0x41 0x42 *
 * ------------------------------------------------------------ */
int get_remap(loglevel_t loglevel, int i2cfd, char mode, int *remap) {
   int reg;

   if(mode == 'c') reg = BNO055_AXIS_MAP_CONFIG_ADDR;
   else if(mode == 's') reg = BNO055_AXIS_MAP_SIGN_ADDR;
   else {
      if(loglevel) printf("Error: Unknown remap function mode %c.\n", mode);
      return 1;
   }

   if(write(i2cfd, &reg, 1) != 1) {
      if(loglevel) printf("Error: I2C write failure for register 0x%02X\n", reg);
      return 2;
   }

   unsigned int data = 0;
   
   if(read(i2cfd, &data, 1) != 1) {
      if(loglevel) printf("Error: I2C read failure for register data 0x%02X\n", reg);
      return 3;
   }

   if(loglevel == level_verbose) printf("Debug: Axis Remap '%c': [0x%02X]\n", mode, data);

   *remap = data;

   return 0;
}

/* ------------------------------------------------------------ *
 * get_inf() queries the BNO055 and write the info data into    *
 * the global struct bnoinf defined in getbno055.h              *
 * ------------------------------------------------------------ */
int get_inf(loglevel_t loglevel, int i2cfd, struct bnoinf *bno_ptr) {
   char reg = 0x00;

   if(write(i2cfd, &reg, 1) != 1) {
      if(loglevel) printf("Error: I2C write failure for register 0x%02X\n", reg);
      return 1;
   }

   char data[7] = {0};

   if(read(i2cfd, data, 7) != 7) {
      if(loglevel) printf("Error: I2C read failure for register data 0x00-0x06\n");
      return 2;
   }

   /* --------------------------------------------------------- *
    * 1-byte chip ID in register 0x00, default: 0xA0            *
    * --------------------------------------------------------- */
   bno_ptr->chip_id = data[0];

   /* --------------------------------------------------------- *
    * 1-byte Accelerometer ID in register 0x01, default: 0xFB   *
    * --------------------------------------------------------- */
   bno_ptr->acc_id = data[1];

   /* --------------------------------------------------------- *
    * 1-byte Magnetometer ID in register 0x02, default 0x32     *
    * --------------------------------------------------------- */
   bno_ptr->mag_id = data[2];

   /* --------------------------------------------------------- *
    * 1-byte Gyroscope ID in register 0x03, default: 0x0F       *
    * --------------------------------------------------------- */
   bno_ptr->gyr_id = data[3];

   /* --------------------------------------------------------- *
    * 1-byte SW Revsion ID LSB in register 0x04, default: 0x08  *
    * --------------------------------------------------------- */
   bno_ptr->sw_lsb = data[4];

   /* --------------------------------------------------------- *
    * 1-byte SW Revision ID MSB in register 0x05, default: 0x03 *
    * --------------------------------------------------------- */
   bno_ptr->sw_msb = data[5];

   /* --------------------------------------------------------- *
    * 1-byte BootLoader Revision ID register 0x06, no default   *
    * --------------------------------------------------------- */
   bno_ptr->bl_rev = data[6];

   if(loglevel == level_verbose) {
      printf("Debug: Sensor CHIP ID: [0x%02X]\n\
Debug: Sensor  ACC ID: [0x%02X]\n\
Debug: Sensor  MAG ID: [0x%02X]\n\
Debug: Sensor  GYR ID: [0x%02X]\n\
Debug: SW  Rev-ID LSB: [0x%02X]\n\
Debug: SW  Rev-ID MSB: [0x%02X]\n\
Debug: Bootloader Ver: [0x%02X]\n",
         data[0], data[1], data[2], data[3], data[4], data[5], data[6]);
   }

   /* --------------------------------------------------------- *
    * Read the operations mode with get_mode(), default: 0x0    *
    * --------------------------------------------------------- */
   opmode_t mode;
   int ret_code = get_mode(loglevel, i2cfd, &mode);

   if(ret_code) return 3 | (ret_code << 8);

   bno_ptr->opr_mode = mode;

   /* --------------------------------------------------------- *
    * Read the power mode with get_power(), default: 0x0        *
    * --------------------------------------------------------- */
   power_t pwrmode;

   ret_code = get_power(loglevel, i2cfd, &pwrmode);

   if(ret_code) return 4 | (ret_code << 8);

   bno_ptr->pwr_mode = pwrmode;

   /* --------------------------------------------------------- *
    * Read the axis remap config get_remap('c'), default: 0x24  *
    * --------------------------------------------------------- */
   int remap = 0;

   ret_code = get_remap(loglevel, i2cfd, 'c', &remap);

   if(ret_code) return 5 | (ret_code << 8);

   bno_ptr->axr_conf = remap;

   /* --------------------------------------------------------- *
    * Read the axis remap sign get_remap('s'), default: 0x00    *
    * --------------------------------------------------------- */
   remap = 0;

   ret_code = get_remap(loglevel, i2cfd, 's', &remap);

   if(ret_code) return 6 | (ret_code << 8);

   bno_ptr->axr_sign = remap;

   /* --------------------------------------------------------- *
    * Read 1-byte system status from register 0x39, no default  *
    * --------------------------------------------------------- */
   reg = BNO055_SYS_STAT_ADDR;

   if(write(i2cfd, &reg, 1) != 1) {
      if(loglevel) printf("Error: I2C write failure for register 0x%02X\n", reg);
      return 7;
   }

   data[0] = 0;

   if(read(i2cfd, data, 1) != 1) {
      if(loglevel) printf("Error: I2C read failure for register data 0x%02X\n", reg);
      return 8;
   }

   if(loglevel == level_verbose) printf("Debug:  System Status: [0x%02X]\n", data[0]);

   bno_ptr->sys_stat = data[0];

   /* --------------------------------------------------------- *
    * Read 1-byte Self Test Result register 0x36, 0x0F=pass     *
    * --------------------------------------------------------- */
   reg = BNO055_SELFTSTRES_ADDR;
   if(write(i2cfd, &reg, 1) != 1) {
      if(loglevel) printf("Error: I2C write failure for register 0x%02X\n", reg);
      return 8;
   }

   data[0] = 0;

   if(read(i2cfd, data, 1) != 1) {
      if(loglevel) printf("Error: I2C read failure for register data 0x%02X\n", reg);
      return 10;
   }

   if(loglevel == level_verbose) printf("Debug: Self-Test Mode: [0x%02X] 4bit [0x%02X]\n", data[0], data[0] & 0x0F);

   bno_ptr->selftest = data[0] & 0x0F; // only get the lowest 4 bits

   /* --------------------------------------------------------- *
    * Read 1-byte System Error from register 0x3A, 0=OK         *
    * --------------------------------------------------------- */
   reg = BNO055_SYS_ERR_ADDR;

   if(write(i2cfd, &reg, 1) != 1) {
      if(loglevel) printf("Error: I2C write failure for register 0x%02X\n", reg);
      return 11;
   }

   data[0] = 0;

   if(read(i2cfd, data, 1) != 1) {
      if(loglevel) printf("Error: I2C read failure for register data 0x%02X\n", reg);
      return 12;
   }

   if(loglevel == level_verbose) printf("Debug: Internal Error: [0x%02X]\n", data[0]);

   bno_ptr->sys_err = data[0];

   /* --------------------------------------------------------- *
    * Read 1-byte Unit definition from register 0x3B, 0=OK      *
    * --------------------------------------------------------- */
   reg = BNO055_UNIT_SEL_ADDR;
   if(write(i2cfd, &reg, 1) != 1) {
      if(loglevel) printf("Error: I2C write failure for register 0x%02X\n", reg);
      return 13;
   }

   data[0] = 0;

   if(read(i2cfd, data, 1) != 1) {
      if(loglevel) printf("Error: I2C read failure for register data 0x%02X\n", reg);
      return 14;
   }

   if(loglevel == level_verbose) printf("Debug: UnitDefinition: [0x%02X]\n", data[0]);

   bno_ptr->unitsel = data[0];

   /* --------------------------------------------------------- *
    * Extract the temperature unit from the unit selection data *
    * --------------------------------------------------------- */
   char t_unit;
   if((data[0] >> 4) & 0x01) t_unit = 'F';
   else  t_unit = 'C';

   /* --------------------------------------------------------- *
    * Read sensor temperature from register 0x34, no default    *
    * --------------------------------------------------------- */
   reg = BNO055_TEMP_ADDR;
   if(write(i2cfd, &reg, 1) != 1) {
      if(loglevel) printf("Error: I2C write failure for register 0x%02X\n", reg);
      return 15;
   }

   data[0] = 0;

   if(read(i2cfd, data, 1) != 1) {
      if(loglevel) printf("Error: I2C read failure for register data 0x%02X\n", reg);
      return 16;
   }

   if(loglevel == level_verbose) printf("Debug:    Temperature: [0x%02X] [%d°%c]\n", data[0], data[0], t_unit);

   bno_ptr->temp_val = data[0];

   return 0;
}

/* ------------------------------------------------------------ *
 *  get_acc() - read accelerometer data into the global struct  *
 * ------------------------------------------------------------ */
int get_acc(loglevel_t loglevel, int i2cfd, struct bnoacc *bnod_ptr) {
   char reg = BNO055_ACC_DATA_X_LSB_ADDR;

   if(write(i2cfd, &reg, 1) != 1) {
      if(loglevel) printf("Error: I2C write failure for register 0x%02X\n", reg);
      return 1;
   }

   char data[6] = {0};

   if(read(i2cfd, data, 6) != 6) {
      if(loglevel) printf("Error: I2C read failure for register data 0x%02X\n", reg);
      return 2;
   }

   int16_t buf = ((int16_t)data[1] << 8) | data[0];

   if(loglevel == level_verbose) printf("Debug: Accelerometer Data X: LSB [0x%02X] MSB [0x%02X] INT16 [%d]\n", data[0], data[1],buf);

   bnod_ptr->adata_x = (double) buf;
   buf = ((int16_t)data[3] << 8) | data[2];

   if(loglevel == level_verbose) printf("Debug: Accelerometer Data Y: LSB [0x%02X] MSB [0x%02X] INT16 [%d]\n", data[2], data[3],buf);

   bnod_ptr->adata_y = (double) buf;
   buf = ((int16_t)data[5] << 8) | data[4];

   if(loglevel == level_verbose) printf("Debug: Accelerometer Data Z: LSB [0x%02X] MSB [0x%02X] INT16 [%d]\n", data[4], data[5],buf);

   bnod_ptr->adata_z = (double) buf;

   return 0;
}

/* ------------------------------------------------------------ *
 *  get_mag() - read magnetometer data into the global struct   *
 *  Convert magnetometer data in microTesla. 1 microTesla = 16  *
 * ------------------------------------------------------------ */
int get_mag(loglevel_t loglevel, int i2cfd, struct bnomag *bnod_ptr) {
   char reg = BNO055_MAG_DATA_X_LSB_ADDR;

   if(write(i2cfd, &reg, 1) != 1) {
      if(loglevel) printf("Error: I2C write failure for register 0x%02X\n", reg);
      return 1;
   }

   char data[6] = {0};

   if(read(i2cfd, data, 6) != 6) {
      if(loglevel) printf("Error: I2C read failure for register data 0x%02X\n", reg);
      return 2;
   }

   int16_t buf = ((int16_t)data[1] << 8) | data[0]; 

   if(loglevel == level_verbose) printf("Debug: Magnetometer Data X: LSB [0x%02X] MSB [0x%02X] INT16 [%d]\n", data[0], data[1],buf);

   bnod_ptr->mdata_x = (double) buf / 1.6;
   buf = ((int16_t)data[3] << 8) | data[2];

   if(loglevel == level_verbose) printf("Debug: Magnetometer Data Y: LSB [0x%02X] MSB [0x%02X] INT16 [%d]\n", data[2], data[3],buf);

   bnod_ptr->mdata_y = (double) buf / 1.6;
   buf = ((int16_t)data[5] << 8) | data[4];

   if(loglevel == level_verbose) printf("Debug: Magnetometer Data Z: LSB [0x%02X] MSB [0x%02X] INT16 [%d]\n", data[4], data[5],buf);

   bnod_ptr->mdata_z = (double) buf / 1.6;

   return 0;
}

/* ------------------------------------------------------------ *
 *  get_gyr() - read gyroscope data into the global struct      *
 * ------------------------------------------------------------ */
int get_gyr(loglevel_t loglevel, int i2cfd, struct bnogyr *bnod_ptr) {
   char reg = BNO055_GYRO_DATA_X_LSB_ADDR;

   if(write(i2cfd, &reg, 1) != 1) {
      if(loglevel) printf("Error: I2C write failure for register 0x%02X\n", reg);
      return 1;
   }

   char data[6] = {0};

   if(read(i2cfd, data, 6) != 6) {
      if(loglevel) printf("Error: I2C read failure for register data 0x%02X\n", reg);
      return 2;
   }

   int16_t buf = ((int16_t)data[1] << 8) | data[0];

   if(loglevel == level_verbose) printf("Debug: Gyroscope Data X: LSB [0x%02X] MSB [0x%02X] INT16 [%d]\n", data[0], data[1],buf);

   bnod_ptr->gdata_x = (double) buf / 16.0;
   buf = ((int16_t)data[3] << 8) | data[2];

   if(loglevel == level_verbose) printf("Debug: Gyrosscope Data Y: LSB [0x%02X] MSB [0x%02X] INT16 [%d]\n", data[2], data[3],buf);

   bnod_ptr->gdata_y = (double) buf / 16.0;
   buf = ((int16_t)data[5] << 8) | data[4];

   if(loglevel == level_verbose) printf("Debug: Gyroscope Data Z: LSB [0x%02X] MSB [0x%02X] INT16 [%d]\n", data[4], data[5],buf);

   bnod_ptr->gdata_z = (double) buf / 16.0;

   return 0;
}

/* ------------------------------------------------------------ *
 *  get_eul() - read Euler orientation into the global struct   *
 * ------------------------------------------------------------ */
int get_eul(loglevel_t loglevel, int i2cfd, struct bnoeul *bnod_ptr) {
   char reg = BNO055_EULER_H_LSB_ADDR;

   if(write(i2cfd, &reg, 1) != 1) {
      if(loglevel) printf("Error: I2C write failure for register 0x%02X\n", reg);
      return 1;
   }

   if(loglevel == level_verbose) printf("Debug: I2C read 6 bytes starting at register 0x%02X\n", reg);

   unsigned char data[6] = {0};

   if(read(i2cfd, data, 6) != 6) {
      if(loglevel) printf("Error: I2C read failure for register data 0x%02X\n", reg);
      return 1;
   }

   int16_t buf = ((int16_t)data[1] << 8) | data[0]; 

   if(loglevel == level_verbose) printf("Debug: Euler Orientation H: LSB [0x%02X] MSB [0x%02X] INT16 [%d]\n", data[0], data[1],buf);

   bnod_ptr->eul_head = (double) buf / 16.0;
   buf = ((int16_t)data[3] << 8) | data[2]; 

   if(loglevel == level_verbose) printf("Debug: Euler Orientation R: LSB [0x%02X] MSB [0x%02X] INT16 [%d]\n", data[2], data[3],buf);

   bnod_ptr->eul_roll = (double) buf / 16.0;
   buf = ((int16_t)data[5] << 8) | data[4]; 

   if(loglevel == level_verbose) printf("Debug: Euler Orientation P: LSB [0x%02X] MSB [0x%02X] INT16 [%d]\n", data[4], data[5],buf);

   bnod_ptr->eul_pitc = (double) buf / 16.0;

   return 0;
}

/* ------------------------------------------------------------ *
 *  get_qua() - read Quaternation data into the global struct   *
 * ------------------------------------------------------------ */
int get_qua(loglevel_t loglevel, int i2cfd, struct bnoqua *bnod_ptr) {
   char reg = BNO055_QUATERNION_DATA_W_LSB_ADDR;

   if(write(i2cfd, &reg, 1) != 1) {
      if(loglevel) printf("Error: I2C write failure for register 0x%02X\n", reg);
      return 1;
   }

   unsigned char data[8] = {0};

   if(loglevel == level_verbose) printf("Debug: I2C read 8 bytes starting at register 0x%02X\n", reg);

   if(read(i2cfd, data, 8) != 8) {
      if(loglevel) printf("Error: I2C read failure for register data 0x%02X\n", reg);
      return 2;
   }

   int16_t buf = ((int16_t)data[1] << 8) | data[0];

   if(loglevel == level_verbose) printf("Debug: Quaternation W: LSB [0x%02X] MSB [0x%02X] INT16 [%d]\n", data[0], data[1], buf);

   bnod_ptr->quater_w = (double) buf / 16384.0;
   buf = ((int16_t)data[3] << 8) | data[2];

   if(loglevel == level_verbose) printf("Debug: Quaternation X: LSB [0x%02X] MSB [0x%02X] INT16 [%d]\n", data[2], data[3], buf);

   bnod_ptr->quater_x = (double) buf / 16384.0;
   buf = ((int16_t)data[5] << 8) | data[4];

   if(loglevel == level_verbose) printf("Debug: Quaternation Y: LSB [0x%02X] MSB [0x%02X] INT16 [%d]\n", data[4], data[5], buf);

   bnod_ptr->quater_y = (double) buf / 16384.0;
   buf = ((int16_t)data[7] << 8) | data[6];

   if(loglevel == level_verbose) printf("Debug: Quaternation Z: LSB [0x%02X] MSB [0x%02X] INT16 [%d]\n", data[6], data[7], buf);

   bnod_ptr->quater_z = (double) buf / 16384.0;

   return 0;
}

/* ------------------------------------------------------------ *
 *  get_gra() - read gravity vector into the global struct      *
 * ------------------------------------------------------------ */
int get_gra(loglevel_t loglevel, int i2cfd, struct bnogra *bnod_ptr) {
   /* --------------------------------------------------------- *
    * Get the unit conversion: 1 m/s2 = 100 LSB, 1 mg = 1 LSB   *
    * --------------------------------------------------------- */
   char reg = BNO055_UNIT_SEL_ADDR;

   if(write(i2cfd, &reg, 1) != 1) {
      if(loglevel) printf("Error: I2C write failure for register 0x%02X\n", reg);
      return 1;
   }

   char unit_sel;

   if(read(i2cfd, &unit_sel, 1) != 1) {
      if(loglevel) printf("Error: I2C read failure for register data 0x%02X\n", reg);
      return 2;
   }

   double ufact = ((unit_sel >> 0) & 0x01) ? 1.0 : 100.0;

   /* --------------------------------------------------------- *
    * Get the gravity vector data                               *
    * --------------------------------------------------------- */
   reg = BNO055_GRAVITY_DATA_X_LSB_ADDR;

   if(write(i2cfd, &reg, 1) != 1) {
      if(loglevel) printf("Error: I2C write failure for register 0x%02X\n", reg);
      return 3;
   }

   if(loglevel == level_verbose) printf("Debug: I2C read 6 bytes starting at register 0x%02X\n", reg);

   unsigned char data[6] = {0};

   if(read(i2cfd, data, 6) != 6) {
      if(loglevel) printf("Error: I2C read failure for register data 0x%02X\n", reg);
      return 4;
   }

   int16_t buf = ((int16_t)data[1] << 8) | data[0];

   if(loglevel == level_verbose) printf("Debug: Gravity Vector H: LSB [0x%02X] MSB [0x%02X] INT16 [%d]\n", data[0], data[1],buf);

   bnod_ptr->gravityx = (double) buf / ufact;
   buf = ((int16_t)data[3] << 8) | data[2];

   if(loglevel == level_verbose) printf("Debug: Gravity Vector M: LSB [0x%02X] MSB [0x%02X] INT16 [%d]\n", data[2], data[3],buf);

   bnod_ptr->gravityy = (double) buf / ufact;
   buf = ((int16_t)data[5] << 8) | data[4];

   if(loglevel == level_verbose) printf("Debug: Gravity Vector P: LSB [0x%02X] MSB [0x%02X] INT16 [%d]\n", data[4], data[5],buf);

   bnod_ptr->gravityz = (double) buf / ufact;

   return 0;
}

/* ------------------------------------------------------------ *
 *  get_lin() - read linear acceleration into the global struct *
 * ------------------------------------------------------------ */
int get_lin(loglevel_t loglevel, int i2cfd, struct bnolin *bnod_ptr) {
   /* --------------------------------------------------------- *
    * Get the unit conversion: 1 m/s2 = 100 LSB, 1 mg = 1 LSB   *
    * --------------------------------------------------------- */
   char reg = BNO055_UNIT_SEL_ADDR;

   if(write(i2cfd, &reg, 1) != 1) {
      if(loglevel) printf("Error: I2C write failure for register 0x%02X\n", reg);
      return 1;
   }

   char unit_sel;

   if(read(i2cfd, &unit_sel, 1) != 1) {
      if(loglevel) printf("Error: I2C read failure for register data 0x%02X\n", reg);
      return 2;
   }

   double ufact = ((unit_sel >> 0) & 0x01) ? 1.0 : 100.0;

   /* --------------------------------------------------------- *
    * Get the linear acceleration data                          *
    * --------------------------------------------------------- */
   reg = BNO055_LIN_ACC_DATA_X_LSB_ADDR;

   if(write(i2cfd, &reg, 1) != 1) {
      if(loglevel) printf("Error: I2C write failure for register 0x%02X\n", reg);
      return 3;
   }

   if(loglevel == level_verbose) printf("Debug: I2C read 6 bytes starting at register 0x%02X\n", reg);

   unsigned char data[6] = {0};

   if(read(i2cfd, data, 6) != 6) {
      if(loglevel) printf("Error: I2C read failure for register data 0x%02X\n", reg);
      return 4;
   }

   int16_t buf = ((int16_t)data[1] << 8) | data[0];

   if(loglevel == level_verbose) printf("Debug: Linear Acceleration H: LSB [0x%02X] MSB [0x%02X] INT16 [%d]\n", data[0], data[1],buf);

   bnod_ptr->linacc_x = (double) buf / ufact;
   buf = ((int16_t)data[3] << 8) | data[2];

   if(loglevel == level_verbose) printf("Debug: Linear Acceleration M: LSB [0x%02X] MSB [0x%02X] INT16 [%d]\n", data[2], data[3],buf);

   bnod_ptr->linacc_y = (double) buf / ufact;
   buf = ((int16_t)data[5] << 8) | data[4];

   if(loglevel == level_verbose) printf("Debug: Linear Acceleration P: LSB [0x%02X] MSB [0x%02X] INT16 [%d]\n", data[4], data[5],buf);

   bnod_ptr->linacc_z = (double) buf / ufact;

   return 0;
}

/* ------------------------------------------------------------ *
 * get_clksrc() - return setting for internal/external clock    *
 * ------------------------------------------------------------ */
int get_clksrc(loglevel_t loglevel, int i2cfd, int *status) {
   char reg = BNO055_SYS_TRIGGER_ADDR;

   if(write(i2cfd, &reg, 1) != 1) {
      if(loglevel) printf("Error: I2C write failure for register 0x%02X\n", reg);

      set_page0(loglevel, i2cfd);

      return 1;
   }

   char data = 0;

   if(read(i2cfd, &data, 1) != 1) {
      if(loglevel) printf("Error: I2C read failure for register data 0x%02X\n", reg);

      set_page0(loglevel, i2cfd);

      return 2;
   }

   if(loglevel == level_verbose) printf("Debug: CLK_SEL bit-7 in register %d: [%d]\n", reg, (data & 0b10000000) >> 7);

   *status = (data & 0b10000000) >> 7; // system calibration status

   return 0;
}

/* ------------------------------------------------------------ *
 * get_sstat() returns the sensor sys status from register 0x39 *
 * ------------------------------------------------------------ */
int get_sstat(loglevel_t loglevel, int i2cfd, int *sstat) {
   int reg = BNO055_SYS_STAT_ADDR;

   if(write(i2cfd, &reg, 1) != 1) {
      if(loglevel) printf("Error: I2C write failure for register 0x%02X\n", reg);
      return 1;
   }

   unsigned int data = 0;

   if(read(i2cfd, &data, 1) != 1) {
      if(loglevel) printf("Error: I2C read failure for register data 0x%02X\n", reg);
      return 2;
   }

   if(loglevel == level_verbose) printf("Debug:  System Status: [0x%02X]\n", data);

   *sstat = data;

   return 0;
}

/* --------------------------------------------------------------- *
 * bno_dump() dumps the register map data.                         *
 * --------------------------------------------------------------- */
int bno_dump(loglevel_t loglevel, int i2cfd) {
   if(loglevel > level_error) {
       printf("------------------------------------------------------\n\
BNO055 page-0:\n\
------------------------------------------------------\n\
 reg    0  1  2  3  4  5  6  7  8  9  A  B  C  D  E  F\n\
------------------------------------------------------\n");
   }

   for(int count = 0; count < 8; ++count) {
      char reg = count;

      if(write(i2cfd, &reg, 1) != 1) {
         if(loglevel) printf("Error: I2C write failure for register 0x%02X\n", reg);
         return 1;
      }

      char data[16] = {0};

      if(read(i2cfd, &data, 16) != 16) {
         if(loglevel) printf("Error: I2C read failure for register 0x%02X\n", reg);
         return 2;
      }

      if(loglevel > level_error) {
         printf("[0x%02X] %02X %02X %02X %02X %02X %02X %02X %02X  %02X %02X %02X %02X %02X %02X %02X %02X\n",
            (reg*16), data[0], data[1], data[2], data[3], data[4], data[5], data[6], data[7],
            data[8], data[9], data[10], data[11], data[12], data[13], data[14], data[15]);
      }
   }

   set_page1(loglevel, i2cfd);
   usleep(50 * 1000);

   if(loglevel > level_error) {
      printf("------------------------------------------------------\n\
BNO055 page-1:\n\
------------------------------------------------------\n\
 reg    0  1  2  3  4  5  6  7  8  9  A  B  C  D  E  F\n\
------------------------------------------------------\n");
   }
   
   for(int count = 0; count < 8; ++count) {
      char reg = count;

      if(write(i2cfd, &reg, 1) != 1) {
         if(loglevel) printf("Error: I2C write failure for register 0x%02X\n", reg);
         return 3;
      }

      char data[16] = {0};

      if(read(i2cfd, &data, 16) != 16) {
         if(loglevel) printf("Error: I2C read failure for register 0x%02X\n", reg);
         return 4;
      }

      if(loglevel > level_error) {
         printf("[0x%02X] %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X\n",
            (reg*16), data[0], data[1], data[2], data[3], data[4], data[5], data[6], data[7],
            data[8], data[9], data[10], data[11], data[12], data[13], data[14], data[15]);
      }
   }

   set_page0(loglevel, i2cfd);
   usleep(50 * 1000);
   
   return 0;
}

/* --------------------------------------------------------------- *
 * bno_reset() resets the sensor. It will come up in CONFIG mode.  *
 * --------------------------------------------------------------- */
int bno_reset(loglevel_t loglevel, int i2cfd) {
   char data[2];

   data[0] = BNO055_SYS_TRIGGER_ADDR;
   data[1] = 0x20;

   if(write(i2cfd, data, 2) != 2) {
      if(loglevel) printf("Error: I2C write failure for register 0x%02X\n", data[0]);
      
      return 1;
   }

   if(loglevel == level_verbose) printf("Debug: BNO055 Sensor Reset complete\n");
   
   /* ------------------------------------------------------------ *
    * After a reset, the sensor needs at leat 650ms to boot up.    *
    * ------------------------------------------------------------ */
   usleep(650 * 1000);
   exit(0);
}

/* ------------------------------------------------------------ *
 * load_cal() load previously saved calibration data from file  *
 * ------------------------------------------------------------ */
int load_cal(loglevel_t loglevel, int i2cfd, char *file) {
   /* -------------------------------------------------------- *
    *  Open the calibration data file for reading.             *
    * -------------------------------------------------------- */
   FILE *calib = fopen(file, "r");

   if(!calib) {
      if(loglevel) printf("Error: Can't open %s for reading.\n", file);
      return 1;
   }

   if(loglevel == level_verbose) printf("Debug: Load from file: [%s]\n", file);

   /* -------------------------------------------------------- *
    * Read 34 bytes from file into data[], starting at data[1] *
    * -------------------------------------------------------- */
   char data[CALIB_BYTECOUNT+1] = {0};
   //data[0] = ACC_OFFSET_X_LSB_ADDR;
   data[0] = BNO055_SIC_MATRIX_0_LSB_ADDR;
   int inbytes = fread(&data[1], 1, CALIB_BYTECOUNT, calib);
   fclose(calib);

   if(inbytes != CALIB_BYTECOUNT) {
      if(loglevel) printf("Error: %d/%d bytes read to file.\n", inbytes, CALIB_BYTECOUNT);
      return 2;
   }

   if(loglevel == level_verbose) {
      printf("Debug: Calibrationset:");

      for(int i = 0; i < CALIB_BYTECOUNT+1; ++i) printf(" %02X", data[i]);

      printf("\n");
   }

   /* -------------------------------------------------------- *
    * Write 34 bytes from file into sensor registers from 0x43 *
    * We need to switch in and out of CONFIG mode if needed... *
    * -------------------------------------------------------- */
   opmode_t oldmode;
   int ret_code = get_mode(loglevel, i2cfd, &oldmode);
   
   if(ret_code) return 3 | (ret_code << 8);

   ret_code = set_mode(loglevel, i2cfd, config);

   if(ret_code) return 4 | (ret_code << 8);

   usleep(50 * 1000);

   if(write(i2cfd, data, (CALIB_BYTECOUNT+1)) != (CALIB_BYTECOUNT+1)) {
      if(loglevel) printf("Error: I2C write failure for register 0x%02X\n", data[0]);
      return 5;
   }

   /* -------------------------------------------------------- *
    * To verify, we read 34 bytes from 0x43 & compare to input *
    * -------------------------------------------------------- */
   //char reg = ACC_OFFSET_X_LSB_ADDR;
   char reg = BNO055_SIC_MATRIX_0_LSB_ADDR;
   if(write(i2cfd, &reg, 1) != 1) {
      if(loglevel) printf("Error: I2C write failure for register 0x%02X\n", reg);
      return 6;
   }

   char newdata[CALIB_BYTECOUNT] = {0};
   if(read(i2cfd, newdata, CALIB_BYTECOUNT) != CALIB_BYTECOUNT) {
      if(loglevel) printf("Error: I2C calibration data read from 0x%02X\n", reg);
      return 7;
   }

   if(loglevel == level_verbose) printf("Debug: Registerupdate:");

   for(int i = 0; i < CALIB_BYTECOUNT; ++i) {
      if(data[i+1] != newdata[i]) {
         if(loglevel) printf("\nError: Calibration load failure %02X register 0x%02X\n", newdata[i], reg+i);
         //exit(-1);
      }

      if(loglevel == level_verbose) printf(" %02X", newdata[i]);
   }

   if(loglevel == level_verbose) printf("\n");

   ret_code = set_mode(loglevel, i2cfd, oldmode);

   if(ret_code) return 8 | (ret_code << 8);

   /* -------------------------------------------------------- *
    * 650 ms delay are only needed if -l and -t are both used  *
    * to let the fusion code process the new calibration data  *
    * -------------------------------------------------------- */
   usleep(650 * 1000);

   return 0;
}

/* ------------------------------------------------------------ *
 * save_cal() - writes calibration data to file for reuse       *
 * ------------------------------------------------------------ */
int save_cal(loglevel_t loglevel, int i2cfd, char *file) {
   /* --------------------------------------------------------- *
    * Read 34 bytes calibration data from registers 0x43~66,    *
    * plus 4 reg 0x67~6A with accelerometer/magnetometer radius *
    * switch to CONFIG, data is only visible in non-fusion mode *
    * --------------------------------------------------------- */
   opmode_t oldmode;
   int ret_code = get_mode(loglevel, i2cfd, &oldmode);

   if(ret_code) return 1 | (ret_code << 8);

   ret_code = set_mode(loglevel, i2cfd, config);

   if(ret_code) return 2 | (ret_code << 8);

   // int i = 0;
   //char reg = ACC_OFFSET_X_LSB_ADDR;
   char reg = BNO055_SIC_MATRIX_0_LSB_ADDR;
   if(write(i2cfd, &reg, 1) != 1) {
      if(loglevel) printf("Error: I2C write failure for register 0x%02X\n", reg);
      return 3;
   }

   if(loglevel == level_verbose) printf("Debug: I2C read %d bytes starting at register 0x%02X\n",
                           CALIB_BYTECOUNT, reg);

   char data[CALIB_BYTECOUNT] = {0};

   if(read(i2cfd, data, CALIB_BYTECOUNT) != CALIB_BYTECOUNT) {
      if(loglevel) printf("Error: I2C calibration data read from 0x%02X\n", reg);
      return 4;
   }

   if(loglevel == level_verbose) {
      printf("Debug: Calibrationset:");

      for(int i = 0; i < CALIB_BYTECOUNT; ++i) printf(" %02X", data[i]);

      printf("\n");
   }

   /* -------------------------------------------------------- *
    *  Open the calibration data file for writing.             *
    * -------------------------------------------------------- */
   FILE *calib = fopen(file, "w");

   if(!calib) {
      if(loglevel) printf("Error: Can't open %s for writing.\n", file);
      return 5;
   }

   if(loglevel == level_verbose) printf("Debug:  Write to file: [%s]\n", file);

   /* -------------------------------------------------------- *
    * write the bytes in data[] out                            *
    * -------------------------------------------------------- */
   int outbytes = fwrite(data, 1, CALIB_BYTECOUNT, calib);

   fclose(calib);

   if(loglevel == level_verbose) printf("Debug:  Bytes to file: [%d]\n", outbytes);

   if(outbytes != CALIB_BYTECOUNT) {
      if(loglevel) printf("Error: %d/%d bytes written to file.\n", outbytes, CALIB_BYTECOUNT);
      return 6;
   }

   ret_code = set_mode(loglevel, i2cfd, oldmode);

   return ret_code ? (7 | (ret_code << 8)) : 0;
}

/* ------------------------------------------------------------ *
 * get_acc_conf() read accelerometer config into global struct  *
 * Requires switching register page 0->1 and back after reading *
 * ------------------------------------------------------------ */
int get_acc_conf(loglevel_t loglevel, int i2cfd, struct bnoaconf *bnoc_ptr) {
   set_page1(loglevel, i2cfd);

   char reg = BNO055_ACC_CONFIG_ADDR;

   if(write(i2cfd, &reg, 1) != 1) {
      if(loglevel) printf("Error: I2C write failure for register 0x%02X\n", reg);

      set_page0(loglevel, i2cfd);

      return 1;
   }

   char data;

   if(read(i2cfd, &data, 1) != 1) {
      if(loglevel) printf("Error: I2C read failure for register data 0x%02X\n", reg);
      
      set_page0(loglevel, i2cfd);

      return 2;
   }

   bnoc_ptr->range   = (data & 0b00000011) >> 2; // accel range
   bnoc_ptr->bandwth = (data & 0b00011100) >> 4; // accel bandwidth
   bnoc_ptr->pwrmode = (data & 0b11100000) >> 6; // accel power mode


   if(loglevel == level_verbose) {
      printf("Debug:       accelerometer range: [%d]\n\
Debug:   accelerometer bandwidth: [%d]\n\
Debug:  accelerometer power mode: [%d]\n",
         bnoc_ptr->pwrmode, bnoc_ptr->bandwth, bnoc_ptr->pwrmode);
   }

   reg = BNO055_ACC_SLEEP_CONFIG_ADDR;

   if(write(i2cfd, &reg, 1) != 1) {
      if(loglevel) printf("Error: I2C write failure for register 0x%02X\n", reg);
      
      set_page0(loglevel, i2cfd);

      return 3;
   }

   data = 0;

   if(read(i2cfd, &data, 1) != 1) {
      if(loglevel) printf("Error: I2C read failure for register data 0x%02X\n", reg);
      
      set_page0(loglevel, i2cfd);

      return 4;
   }

   bnoc_ptr->slpmode = (data & 0b00000011) >> 2; // accel sleep mode
   bnoc_ptr->slpdur = (data & 0b00011100) >> 4; // accel sleep duration

   if(loglevel == level_verbose) {
      printf("Debug:  accelerometer sleep mode: [%d]\n\
Debug:   accelerometer sleep dur: [%d]\n",
         bnoc_ptr->slpmode, bnoc_ptr->slpdur);
   }

   set_page0(loglevel, i2cfd);

   return 0;
}
