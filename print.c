/* ------------------------------------------------------------ *
 * file:        i2c_bno055.c                                    *
 * purpose:     Extract sensor data from Bosch BNO055 modules.  *
 *              Functions for I2C bus communication, get and    *
 *              set sensor register data. Ths file belongs to   *
 *              the pi-bno055 package. Functions are called     *
 *              from getbno055.c, globals are in getbno055.h.   *
 *                                                              *
 * Requires:	I2C development packages i2c-tools libi2c-dev   *
 *                                                              *
 * author:      07/14/2018 Frank4DD                             *
 * ------------------------------------------------------------ */
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <fcntl.h>
#include "getbno055.h"
#include "libbno055.h"

/* ------------------------------------------------------------ *
 * print_unit() - Extract the SI unit config from register 0x3B *
 * ------------------------------------------------------------ */
void print_unit(int unit_sel) {
   // bit-0
   printf("Acceleration Unit  = ");
   if((unit_sel >> 0) & 0x01) printf("mg\n");
   else printf("m/s2\n");

   // bit-1
   printf("    Gyroscope Unit = ");
   if((unit_sel >> 1) & 0x01) printf("rps\n");
   else printf("dps\n");

   // bit-2
   printf("        Euler Unit = ");
   if((unit_sel >> 2) & 0x01) printf("Radians\n");
   else printf("Degrees\n");

   // bit-3: unused
   // bit-4
   printf("  Temperature Unit = ");
   if((unit_sel >> 4) & 0x01) printf("Fahrenheit\n");
   else printf("Celsius\n");

   // bit-5: unused
   // bit-6: unused
   // bit-7
   printf("  Orientation Mode = ");
   if((unit_sel >> 3) & 0x01) printf("Android\n");
   else printf("Windows\n");
}

/* ------------------------------------------------------------ *
 * print_mode() - prints sensor operational mode string from    *
 * sensor operational mode numeric value.                       *
 * ------------------------------------------------------------ */
int print_mode(int mode) {
   if(mode < 0 || mode > 12) return(-1);
   
   switch(mode) {
      case 0x00:
         printf("CONFIG\n");
         break;
      case 0x01:
         printf("ACCONLY\n");
         break;
      case 0x02:
         printf("MAGONLY\n");
         break;
      case 0x03:
         printf("GYRONLY\n");
         break;
      case 0x04:
         printf("ACCMAG\n");
         break;
      case 0x05:
         printf("ACCGYRO\n");
         break;
      case 0x06:
         printf("MAGGYRO\n");
         break;
      case 0x07:
         printf("AMG\n");
         break;
      case 0x08:
         printf("IMU\n");
         break;
      case 0x09:
         printf("COMPASS\n");
         break;
      case 0x0A:
         printf("M4G\n");
         break;
      case 0x0B:
         printf("NDOF_FMC_OFF\n");
         break;
      case 0x0C:
         printf("NDOF_FMC\n");
         break;
   }
   return(0);
}

/* ------------------------------------------------------------ *
 * print_power() - prints the sensor power mode string from the *
 * sensors power mode numeric value.                            *
 * ------------------------------------------------------------ */
int print_power(int mode) {
   if(mode < 0 || mode > 2) return(-1);

   switch(mode) {
      case 0x00:
         printf("NORMAL\n");
         break;
      case 0x01:
         printf("LOW\n");
         break;
      case 0x02:
         printf("SUSPEND\n");
         break;
   }
   return(0);
}

/* ------------------------------------------------------------ *
 * print_sstat() - prints the sensor system status string from  *
 * the numeric value located in the sys_stat register 0x39      *
 * ------------------------------------------------------------ */
int print_sstat(int stat_code) {
   if(stat_code < 0 || stat_code > 6) return(-1);

   switch(stat_code) {
      case 0x00:
         printf("Idle\n");
         break;
      case 0x01:
         printf("System Error\n");
         break;
      case 0x02:
         printf("Initializing Peripherals\n");
         break;
      case 0x03:
         printf("System Initalization\n");
         break;
      case 0x04:
         printf("Executing Self-Test\n");
         break;
      case 0x05:
         printf("Sensor running with fusion algorithm\n");
         break;
      case 0x06:
         printf("Sensor running without fusion algorithm\n");
         break;
   }
   return(0);
}

/* ------------------------------------------------------------ *
 * print_remap_conf() - prints the sensor axis configuration.   *
 * the numeric values are located in register 0x41. Valid modes *
 * are: 
 * 0x24 (default), 0x21, 
 * ------------------------------------------------------------ */
int print_remap_conf(int mode) {
   if(mode != 0x24 && mode != 0x18 && mode != 0x09 && mode != 0x36) return(-1);

   switch(mode) {
      case 0x24:     // 0 1 | 1 0 | 0 0
         printf("X==X Y==Y Z==Z (ENU)\n");
         break;
      case 0x18:     // 0 1 | 0 0 | 1 0 
         printf("X<>Y Y<>X Z==Z (NEU)\n");
         break;
      case 0x09:     // 0 0 | 1 0 | 0 1
         printf("X<>Z Y==Y Z<>X (UNE)\n");
         break;
      case 0x36:     // 1 0 | 0 1 | 0 0
         printf("X==X Y<>Z Z<>Y (EUN)\n");
         break;
   }
   return(0);
}

/* ------------------------------------------------------------ *
 * print_remap_sign() - prints the sensor axis remapping +/-.   *
 * the numeric values are located in register 0x42.             *
 * ------------------------------------------------------------ */
int print_remap_sign(int mode) {
   if(mode < 0 || mode > 7) return(-1);

   switch(mode) {
      case 0x00:
         printf("X+ Y+ Z+\n");
         break;
      case 0x01:
         printf("X+ Y+ Z-\n");
         break;
      case 0x02:
         printf("X+ Y- Z+\n");
         break;
      case 0x03:
         printf("X+ Y- Z-\n");
         break;
      case 0x04:
         printf("X- Y+ Z+\n");
         break;
      case 0x05:
         printf("X- Y+ Z-\n");
         break;
      case 0x06:
         printf("X- Y- Z+\n");
         break;
      case 0x07:
         printf("X- Y- Z-\n");
         break;
   }
   return(0);
}

/* ------------------------------------------------------------ *
 * print_clksrc() - print setting for internal/external clock   *
 * ------------------------------------------------------------ */
void print_clksrc(loglevel_t loglevel) {
   int src = 0;
   int ret_code = get_clksrc(loglevel, i2cfd, &src);

   if(src == 0) printf("Internal Clock (default)\n");
   else if(src == 1) printf("External Clock\n");
   else if(ret_code || src == -1) printf("Clock Reading error\n");
}

/* ----------------------------------------------------------- *
 *  print_acc_conf() - print accelerometer configuration       *
 * ----------------------------------------------------------- */
void print_acc_conf(struct bnoaconf *bnoc_ptr) {
   printf("Accelerometer  Power = ");
   switch(bnoc_ptr->pwrmode) {
      case 0:
         printf("NORMAL\n");
         break;
      case 1:
         printf("SUSPEND\n");
         break;
      case 2:
         printf("LOW POWER1\n");
         break;
      case 3:
         printf("STANDBY\n");
         break;
      case 4:
         printf("LOW POWER2\n");
         break;
      case 5:
         printf("DEEP SUSPEND\n");
         break;
   }
   printf("Accelerometer Bwidth = ");
   switch(bnoc_ptr->bandwth) {
     case 0:
         printf("7.81Hz\n");
         break;
      case 1:
         printf("15.63Hz\n");
         break;
      case 2:
         printf("31.25Hz\n");
         break;
      case 3:
         printf("62.5Hz\n");
         break;
      case 4:
         printf("125Hz\n");
         break;
      case 5:
         printf("250Hz\n");
         break;
      case 6:
         printf("500Hz\n");
         break;
      case 7:
         printf("1KHz\n");
         break;
   }
   printf("Accelerometer GRange = ");
   switch(bnoc_ptr->range) {
      case 0:
         printf("2G\n");
         break;
      case 1:
         printf("4G\n");
         break;
      case 2:
         printf("8G\n");
         break;
      case 3:
         printf("16G\n");
         break;
   }
   printf("Accelerometer  Sleep = ");
   switch(bnoc_ptr->slpmode) {
      case 0:
         printf("event-driven, ");
         break;
      case 1:
         printf("equidistant sampling, ");
         break;
   }
   if(bnoc_ptr->slpdur < 6) printf("0.5ms\n");
   else switch(bnoc_ptr->slpdur) {
      case 6:
         printf("1ms\n");
         break;
      case 7:
         printf("2ms\n");
         break;
      case 8:
         printf("4ms\n");
         break;
      case 9:
         printf("6ms\n");
         break;
      case 10:
         printf("10ms\n");
         break;
      case 11:
         printf("25ms\n");
         break;
      case 12:
         printf("50ms\n");
         break;
      case 13:
         printf("100ms\n");
         break;
      case 14:
         printf("500ms\n");
         break;
      case 15:
         printf("1s\n");
         break;
   }
}
