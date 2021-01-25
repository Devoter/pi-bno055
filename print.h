#ifndef PRINT_H
#define PRINT_H

/* ------------------------------------------------------------ *
 * external function prototypes for printing the data           *
 * ------------------------------------------------------------ */
extern void print_clksrc();               // print clock source setting
extern int print_mode(int);               // print ops mode string
extern void print_unit(int);              // print SI unit configuration
extern int print_power(int);              // print power mode string
extern int print_sstat(int);              // print system status string
extern int print_remap_conf(int);         // print axis configuration
extern int print_remap_sign(int);         // print the axis remap +/-
extern void print_acc_conf();             // print accelerometer config
extern void print_mag_conf();             // print magnetometer config
extern void print_gyr_conf();             // print gyroscope config

#endif // PRINT_H
