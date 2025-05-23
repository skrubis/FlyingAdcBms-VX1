#ifndef HWDEFS_H_INCLUDED
#define HWDEFS_H_INCLUDED


//Common for any config

#define RCC_CLOCK_SETUP()  rcc_clock_setup_pll(&rcc_hsi_configs[RCC_CLOCK_HSI_64MHZ])

//Address of parameter block in flash
#define FLASH_PAGE_SIZE 1024
#define PARAM_BLKSIZE FLASH_PAGE_SIZE
#define PARAM_BLKNUM  1   //last block of 1k
#define CAN1_BLKNUM   2

enum HwRev { HW_UNKNOWN, HW_1X, HW_20, HW_21, HW_22, HW_23 };

extern HwRev hwRev;

#endif // HWDEFS_H_INCLUDED
