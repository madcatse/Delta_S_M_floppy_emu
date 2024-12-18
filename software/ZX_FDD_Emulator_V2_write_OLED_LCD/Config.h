#ifndef FDDCONFIG_H
#define FDDCONFIG_H

#include <avr/pgmspace.h>
#include <util/atomic.h>
#include <util/delay.h>
#include <avr/eeprom.h>

#define USE_ENCODER 0       /// не менять, управление энкодером и так работает кнопками

#define MAX_CYL 82          /// maximal cylinder supported by FDD. More cylinders - more memory used.
#define MAX_DIR_LEVEL 5     /// maximal subfolders support 15 MAX. The higher level the more memory used

//============================НАСТРОЙКА ДИСПЛЕЕВ========================================================
#define addrOLED   0x3C     // настройка адреса OLED дисплея
#define addrLCD    0x25     // настройка адреса LCD дисплея (основные 0x27, 0x3F)

//какой экран использовать, LCD или OLED (не включайте OLED и LCD2004 вместе!)------------------------------
#define OLED       0        // 1 - использование OLED дисплея, 0 - LCD дисплей
#define ROTATE     0        // разворот OLED на 180 градусов: 0 - не разворачивать, 1 - развернуть на 180
#define OLED128x32 0        // какой OLED дисплей использовать: 0 - 128х64, 1 - 128х32
#define LCD2004    1        // какой LCD дисплей использовать: 0 - 1602, 1 - 2004, 

// Настройка количества выводимых строк и стартовой позиции в зависимости от экрана LCD ----------------
#if (LCD2004 == 1)
 #define MAX_DISP_COUNT 4    // количество отображаемых строк на экране LCD
 #define START_DISP_POS 0    // стартовая позиция для LCD
#else
 #define MAX_DISP_COUNT 2    // количество отображаемых строк на экране LCD
 #define START_DISP_POS 0    // стартовая позиция для LCD
#endif

// Настройка количества выводимых строк и стартовой позиции в зависимости от экрана OLED----------------
#if (OLED == 1 && OLED128x32 == 1)
 #define MAX_DISP_COUNT 4    // количество отображаемых файлов на экране 128x32
 #define START_DISP_POS 0    // стартовая позиция для файлов 128x32
#else
 #if (OLED == 1 && LCD2004 == 0)
  #define MAX_DISP_COUNT 6    // количество отображаемых файлов на экране 128x64
  #define START_DISP_POS 2    // стартовая позиция для файлов 128x64
 #endif
#endif
//============================КОНЕЦ НАСТРОЙКИ ДИСПЛЕЕВ===================================================

/// Floppy pinout configuration -----------------------------------------------------------------------------

#define SIDE_PIN  PIND  // PORT pin at which side pin is located
#define SIDE_SEL  PD0   // pin 0 (RXD), SIDE SELECT                      (INPUT)
#define READ_DATA PD1   // pin 1 (TXD), READ_DATA                        (OUTPUT) /// defined in USART
#define WP        PD2   // pin 2 (D2),  WRITE PROTECT                    (OUTPUT)
#define TRK00     PD3   // pin 3 (D3),  TRACK 00                         (OUTPUT)
#define STEP      PD4   // pin 4 (D4),  STEP                             (INPUT)
#define DIR_SEL   PD5   // pin 5 (D5),  DIRECTION SELECT                 (INPUT)
#define MOTOR_ON  PD6   // pin 6 (D6),  MOTOR ON                         (INPUT)
#define DRIVE_SEL PD7   // pin 7 (D7),  DRIVE A/B SELECT using jumper    (INPUT)
#define INDEX     PB1   // pin 9 (D9),  INDEX                            (OUTPUT)
#define WG        PC0   // pin A0 ,     WRITE GATE                       (INPUT)

/// Encoder pinout configuration ----------------------------------------------------------------------------

#define ENC_A     PC2
#define ENC_B     PC3
#define BTN       PC1

/// SD Card pinout configuration -----------------------------------------------------------------------------

/* SD card attached to SPI bus as follows: MOSI - pin 11, MISO - pin 12, CLK(SCK) - pin 13, CS - pin 10 */
#define SPI_DDR   DDRB
#define SPI_PORT  PORTB
#define SPI_CS    PB2   // pin 10
#define SPI_MOSI  PB3   // pin 11
#define SPI_MISO  PB4   // pin 12
#define SPI_SCK   PB5   // pin 13


/// ==========================================================================================================
/// DON'T CHANGE NEXT LINES
/// ==========================================================================================================

#define swap(value) asm("swap %0" : "=r" (value) : "0" (value)) 

/// Constant arrays ------------------------------------------------------------------------------------------

const uint16_t Crc16Table[256] PROGMEM = {
    0x0000, 0x1021, 0x2042, 0x3063, 0x4084, 0x50A5, 0x60C6, 0x70E7, 0x8108, 0x9129, 0xA14A, 0xB16B, 0xC18C, 0xD1AD, 0xE1CE, 0xF1EF,
    0x1231, 0x0210, 0x3273, 0x2252, 0x52B5, 0x4294, 0x72F7, 0x62D6, 0x9339, 0x8318, 0xB37B, 0xA35A, 0xD3BD, 0xC39C, 0xF3FF, 0xE3DE,
    0x2462, 0x3443, 0x0420, 0x1401, 0x64E6, 0x74C7, 0x44A4, 0x5485, 0xA56A, 0xB54B, 0x8528, 0x9509, 0xE5EE, 0xF5CF, 0xC5AC, 0xD58D,
    0x3653, 0x2672, 0x1611, 0x0630, 0x76D7, 0x66F6, 0x5695, 0x46B4, 0xB75B, 0xA77A, 0x9719, 0x8738, 0xF7DF, 0xE7FE, 0xD79D, 0xC7BC,
    0x48C4, 0x58E5, 0x6886, 0x78A7, 0x0840, 0x1861, 0x2802, 0x3823, 0xC9CC, 0xD9ED, 0xE98E, 0xF9AF, 0x8948, 0x9969, 0xA90A, 0xB92B,
    0x5AF5, 0x4AD4, 0x7AB7, 0x6A96, 0x1A71, 0x0A50, 0x3A33, 0x2A12, 0xDBFD, 0xCBDC, 0xFBBF, 0xEB9E, 0x9B79, 0x8B58, 0xBB3B, 0xAB1A,
    0x6CA6, 0x7C87, 0x4CE4, 0x5CC5, 0x2C22, 0x3C03, 0x0C60, 0x1C41, 0xEDAE, 0xFD8F, 0xCDEC, 0xDDCD, 0xAD2A, 0xBD0B, 0x8D68, 0x9D49,
    0x7E97, 0x6EB6, 0x5ED5, 0x4EF4, 0x3E13, 0x2E32, 0x1E51, 0x0E70, 0xFF9F, 0xEFBE, 0xDFDD, 0xCFFC, 0xBF1B, 0xAF3A, 0x9F59, 0x8F78,
    0x9188, 0x81A9, 0xB1CA, 0xA1EB, 0xD10C, 0xC12D, 0xF14E, 0xE16F, 0x1080, 0x00A1, 0x30C2, 0x20E3, 0x5004, 0x4025, 0x7046, 0x6067,
    0x83B9, 0x9398, 0xA3FB, 0xB3DA, 0xC33D, 0xD31C, 0xE37F, 0xF35E, 0x02B1, 0x1290, 0x22F3, 0x32D2, 0x4235, 0x5214, 0x6277, 0x7256,
    0xB5EA, 0xA5CB, 0x95A8, 0x8589, 0xF56E, 0xE54F, 0xD52C, 0xC50D, 0x34E2, 0x24C3, 0x14A0, 0x0481, 0x7466, 0x6447, 0x5424, 0x4405,
    0xA7DB, 0xB7FA, 0x8799, 0x97B8, 0xE75F, 0xF77E, 0xC71D, 0xD73C, 0x26D3, 0x36F2, 0x0691, 0x16B0, 0x6657, 0x7676, 0x4615, 0x5634,
    0xD94C, 0xC96D, 0xF90E, 0xE92F, 0x99C8, 0x89E9, 0xB98A, 0xA9AB, 0x5844, 0x4865, 0x7806, 0x6827, 0x18C0, 0x08E1, 0x3882, 0x28A3,
    0xCB7D, 0xDB5C, 0xEB3F, 0xFB1E, 0x8BF9, 0x9BD8, 0xABBB, 0xBB9A, 0x4A75, 0x5A54, 0x6A37, 0x7A16, 0x0AF1, 0x1AD0, 0x2AB3, 0x3A92,
    0xFD2E, 0xED0F, 0xDD6C, 0xCD4D, 0xBDAA, 0xAD8B, 0x9DE8, 0x8DC9, 0x7C26, 0x6C07, 0x5C64, 0x4C45, 0x3CA2, 0x2C83, 0x1CE0, 0x0CC1,
    0xEF1F, 0xFF3E, 0xCF5D, 0xDF7C, 0xAF9B, 0xBFBA, 0x8FD9, 0x9FF8, 0x6E17, 0x7E36, 0x4E55, 0x5E74, 0x2E93, 0x3EB2, 0x0ED1, 0x1EF0
};

/// PIN Definitions for Arduion IDE 1.0 support --------------------------------------------------------------
/*
#if !defined(PB0)
#define  PB0 0
#endif

#if !defined(PB1)
#define PB1 1
#endif

#if !defined(PB2)
#define PB2 2
#endif

#if !defined(PB3)
#define PB3 3
#endif

#if !defined(PB4)
#define PB4 4
#endif

#if !defined(PB5)
#define PB5 5
#endif

#if !defined(PB6)
#define PB6 6
#endif

#if !defined(PB7)
#define PB7 7
#endif


#if !defined(PC0)
#define PC0 0
#endif

#if !defined(PC1)
#define PC1 1
#endif

#if !defined(PC2)
#define PC2 2
#endif

#if !defined(PC3)
#define PC3 3
#endif

#if !defined(PC4)
#define PC4 4
#endif

#if !defined(PC5)
#define PC5 5
#endif

#if !defined(PC6)
#define PC6 6
#endif

#if !defined(PC7)
#define PC7 7
#endif


#if !defined(PD0)
#define PD0 0
#endif

#if !defined(PD1)
#define PD1 1
#endif

#if !defined(PD2)
#define PD2 2
#endif

#if !defined(PD3)
#define PD3 3
#endif

#if !defined(PD4)
#define PD4 4
#endif

#if !defined(PD5)
#define PD5 5
#endif

#if !defined(PD6)
#define PD6 6
#endif

#if !defined(PD7)
#define PD7 7
#endif
*/

#endif /* FDDCONFIG_H */
