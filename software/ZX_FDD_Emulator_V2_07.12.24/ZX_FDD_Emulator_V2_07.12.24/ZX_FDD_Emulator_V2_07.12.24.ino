/*
# ZX_FDD_Emulator V2.0

Эмулятор дисковода от EvgenRu, ссылка на форум:
https://zx-pk.ru/threads/26328-planiruyu-sdelat-fdd-emulyator-na-atmega8.html

Ссылка на гитхаб:
https://github.com/EvgeniyRU/ZX_FDD_Emulator

Сделана  большая доработка эмулятора в плане произвольного расширения
количества  выводимых на дисплей строк файлов,  а так же долгожданная 
функция  записи!  Автор  этих доработок  Evgeny Ugryumov. 

В Config.h можно настроить вывод на 4 вида дисплеев:
> OLED 128x64
> OLED 128x32
> LCD 1602
> LCD 2004
Так же добавлена возможность разворота изображения на 180 градусов
для OLED дисплеев.
Автор "прикрученных" дисплеев - Rahost

Отдельное спасибо Alex Rauch за бета тестирование и помощи в поимке
глюков, Сергею MadCat за поддержку.

Статья о сборке эмулятора:
https://rahost-studio.ru/2024/03/fdd-zx-spectrum/
Дополнение:
https://rahost-studio.ru/2024/03/zx-fdd-2/
Возможные обновления данного кода будут в Телеграме:
https://t.me/RahostStudio


RahostStudio - 08.06.2024
*/

#include "Config.h"
#include "SDCardModule.h"
#include "Fat32Module.h"
//#include "pff.h" //// ?? Что то от Евгения Угрюмова )) 
#include "Wire.h"
#include "ACROBOTIC_SSD1306.h"
#include "LCDModule.h"

uint8_t eeprom_file;
int8_t read_sector_(uint8_t * , uint8_t) ;
void send_byte(uint8_t sector_byte) ;
/// EMULATOR START -------------------------------------------------------------------------------------------------
/// Global variables
uint8_t sector_order[]{0,8,1,9,2,10,3,11,4,12,5,13,6,14,7,15} ;
uint8_t  max_cylinder, cylinder , A1_mark = 0;
volatile uint8_t cylinder_changed ;
uint8_t sector_data[512]; // sector data
uint32_t clust_table[MAX_CYL], sector_table[16]; // Cluster table, Cluster table for sectors in cylinder
union { uint16_t val; struct { byte low; byte high; } bytes; } CRC_H;
char dirs[MAX_DIR_LEVEL][10];
char *path;
FATFS fat;

///////
/// Interrupts enable/disable functions
///////////////////////////////////////////

#ifdef ARDUINO_ARCH_LGT 
#define START_CAPTURE    TCCR1B = _BV(CS11) ;  // start timer with /8 prescaler
#define LENGTH_PULSE  8
#else
#define START_CAPTURE    TCCR1B = _BV(CS10) ;  // start timer with /8 prescaler
#define LENGTH_PULSE  32
#endif

void inline USART_enable()
{
 //   cli();
    /* Set MSPI mode of operation and SPI data mode 0. */
 //   UCSR0C = _BV(UMSEL01) | _BV(UMSEL00);    
    UCSR0B |= _BV(TXEN0);
 //   sei();
}
void inline USART_disable()
{
 //   cli();
 //   UCSR0C &= ~(_BV(UMSEL01) | _BV(UMSEL00));
    UCSR0B &= ~_BV(TXEN0);
//    sei();
}
void inline PCINT1_enable() { PCIFR  |= _BV(PCIE1); PCICR  |= _BV(PCIE1); }
void inline PCINT1_disable() { PCICR &= ~_BV(PCIE1); }
void inline PCINT2_enable() { PCIFR  |= _BV(PCIE2); PCICR  |= _BV(PCIE2); }
void inline PCINT2_disable() { PCICR &= ~_BV(PCIE2); }

#if (USE_ENCODER == 1)  // if encoder selected in config
///////
/// ENCODER interrupt
volatile int8_t encoder_val = 0; // this is important!
uint8_t prev_pc = 0;
///////////////////////////////////////////
ISR(PCINT1_vect)
{
    uint8_t pc_val = PINC & (_BV(ENC_A) | _BV(ENC_B)), A=0, B=0;
    if(prev_pc == (_BV(ENC_A) | _BV(ENC_B)) && pc_val != 0)
    {
        for(uint8_t i = 0; i < 50; i++)
        {
            if(PINC & _BV(ENC_A)) A++;
            if(PINC & _BV(ENC_B)) B++;
        }
        if(A > 48 && B < 2) encoder_val++; else if(B > 48 && A < 2) encoder_val--;
    }
    prev_pc = pc_val;
}
#endif    // end - if encoder selected in config

///////
/// STEP pin interrupt
///////////////////////////////////////////
ISR(PCINT2_vect)
{
    if((PIND & _BV(DRIVE_SEL)) == 0){
        if(!(PIND & (1<<STEP))){
 //           UCSR0B &= ~_BV(TXEN0);
            if(PIND & _BV(DIR_SEL)) {
                if(cylinder != 0) cylinder--;
                if(cylinder == 0) DDRD |= _BV(TRK00);
            }
            else{
 //           if(cylinder + 1 < max_cylinder) cylinder++;
                cylinder++ ;
                if(cylinder == max_cylinder)cylinder-- ;
                DDRD &= ~_BV(TRK00); // Set TRK00 - LOW or HIGH
            }
            cylinder_changed = 1;
        }
   }

}

///////
/// Print files on display and file pointer
////////////////////////////////////////////////////////////////
FILINFO disp_files[MAX_DISP_COUNT], fnfo;
DIR dir, first_dir;
uint8_t  f_index ;

void print_files(uint8_t index)
{
    #if (OLED == 0)
    LCD_print_char(0,index + START_DISP_POS,0);
    //LCD_print_char(0,index,0); // original cod EvgenRU
    for(uint8_t i = 0; i < f_index; i++)
    {
        if((disp_files[i].fattrib & AM_DIR) != 0) LCD_print_char(1,i+ START_DISP_POS,1); // display folder icon
        LCD_print(2,i + START_DISP_POS,disp_files[i].fname); // display file name
        //LCD_print(2,i,disp_files[i].fname); // display file name // original cod EvgenRU
    }
   #else 
    oled.setTextXY(index + START_DISP_POS,1);              // Set cursor position
    oled.putChar(62);
    for(uint8_t i = 0; i <  f_index; i++) {
        oled.setTextXY(i+ START_DISP_POS ,2);              // Set cursor position
        if((disp_files[i].fattrib & AM_DIR) != 0)    oled.putChar(47);
        else oled.putChar(0);
               
        oled.setTextXY(i+ START_DISP_POS ,3);
        oled.putString("            ");
        oled.setTextXY(i + START_DISP_POS ,3);              // Set cursor position
        oled.putString(disp_files[i].fname);
    }
   #endif 
}

///////
/// f_array_ind - LCD display line number
/// dire - direction 0 - forward, 1 - backward
/// READ DIRECTORY ENTRY (1 file name) and put it to array (disp_files) for print on LCD
/////////////////////////////////////////////////////

int8_t readdir0()
{
    for(;;){
        if(pf_dirnext(&dir) != FR_OK) { return -2;}
        if(pf_readdir(&dir, &fnfo, 0) != FR_OK) return -1;   // read directory entry
        
        if(fnfo.fname[0] != 0 && ( (  strcasestr(fnfo.fname,".trd") && (fnfo.fattrib & AM_DIR) == 0) || (fnfo.fattrib & AM_DIR) != 0) ){
   //     if(fnfo.fname[0] != 0) {
            if(fnfo.fname[0] != '.' || fnfo.fname[1] == '.'){
                memcpy(&disp_files[0],&fnfo,sizeof(fnfo));
                return 0;
            }
        }
    }
    return -3;
}

int8_t  shift_pos(uint8_t direct)
{
     uint8_t ret ;
     for(;;){
        if(direct){
            if(!memcmp(&dir,&first_dir,sizeof(dir))){return -2;}
            if(pf_dirprev(&dir) != FR_OK) { return -2;}
        }
        else{
            ret = pf_dirnext(&dir);
            if(ret != FR_OK) {  return -2;}
        }
        if(pf_readdir(&dir, &fnfo, direct) != FR_OK) {  return -1;}
        if(fnfo.fname[0] != 0 && ( (  strcasestr(fnfo.fname,".trd") && (fnfo.fattrib & AM_DIR) == 0) || (fnfo.fattrib & AM_DIR) != 0) ){ 
            if(fnfo.fname[0] != '.' || fnfo.fname[1] == '.'){
               return 0 ; 
            }
        }   
    }
    return -1 ;
}

int8_t readdir(uint8_t f_array_ind, uint8_t dire,uint8_t shift)
{
     uint8_t ret ;
     for(;;){
        ret = shift_pos(dire) ;
        
        if(ret ==0){ 
     //       if( dire && !memcmp(&disp_files[0],&disp_files[1],sizeof(fnfo)-13) && !strncmp(disp_files[0].fname,disp_files[1].fname,12) )  return 0;
            if( !memcmp(&disp_files[f_array_ind],&fnfo,sizeof(fnfo)) )  continue;
            if(f_array_ind == 0 && dire == 1 && shift == 1) {
                memmove( &disp_files[1] , &disp_files[0] ,  sizeof(fnfo)*(MAX_DISP_COUNT-1) ) ;  
            }        
            if(f_array_ind == (MAX_DISP_COUNT-1) && dire == 0 && shift == 1) {
                memmove( &disp_files[0] , &disp_files[1] , sizeof(fnfo)*(MAX_DISP_COUNT-1) ) ;  
            }    
            memcpy(&disp_files[f_array_ind],&fnfo,sizeof(fnfo));
            return 0;
        }
        else {
            break;}   
    }
    return -3;
}
void usart_init()
{
     // Setup USART in MasterSPI mode 1000000bps
 //   UBRR0H = 0x00;
 //   UBRR0L =0x0F ; //0x07; // 1000 kbps for 16MHz external oscillator
    UBRR0 = F_CPU/(1000000L*2) - 1  ;  // 1000 kbps for 16MHz external oscillator
    UCSR0A = 0x00;
    UCSR0C = _BV(UMSEL01) | _BV(UMSEL00); 
    UCSR0B = 0x00; // disabled

}

void port_init()
{
    PCMSK2 |= _BV(PCINT20); // SET PCINT2 interrupt on PD4 (STEP pin)
#if (USE_ENCODER == 1)
    PCMSK1 |= _BV(PCINT10) | _BV(PCINT11); // SET PCINT1 (PC2, PC3) Encoder
#endif
    // INIT pins and ports
    PORTB |= _BV(0) ;
    PORTD |= _BV(STEP) | _BV(MOTOR_ON) | _BV(DRIVE_SEL) | _BV(DIR_SEL) | _BV(SIDE_SEL); // set pull-up
    PORTC |= _BV(ENC_A) | _BV(ENC_B) | _BV(BTN) | _BV(WG); // set pull-up
    DDRB &= ~_BV(INDEX); // SET INDEX HIGH
    DDRB &= ~_BV(0); 
 //   DDRB |= _BV(6);
    DDRD &= ~(_BV(WP) | _BV(TRK00)); // Set WP,TRK00 as input
 }

void spi_init()
{
    // Init SPI for SD Card 
    // (Если я правильно понял, то скорость SPI устанавливается параметрами 
    // _BV(SPR0) и _BV(SPR1), а так же _BV(SPI2X) как удвоение).
    //
    // Таблица:
    // SPI2X    SPR0    SPR1    SCK Frequency
    //   1        0       0      f_osc/2
    //   0        0       0      f_osc/4
    //   1        1       0      f_osc/8
    //   0        1       0      f_osc/16
    //   1        0       1      f_osc/32
    //   0        0       1      f_osc/64
    //   1        1       1      f_osc/64
    //   0        1       1      f_osc/128    
    // 
    // 0 - параметр не используется, 1 - параметр используется
    // Пример:
    // SPCR = _BV(MSTR) | _BV(SPE) | _BV(SPR0) | _BV(SPR1); // Master mode, SPI enable, clock rate f_osc/128, LSB first

    SPI_DDR |= _BV(SPI_MOSI) | _BV(SPI_SCK) | _BV(SPI_CS); //set output mode for MOSI, SCK, CS(SS)
    //SPCR = _BV(MSTR) | _BV(SPE) | _BV(SPR0);   // Master mode, SPI enable, clock rate f_osc/4, LSB first
    SPCR = _BV(MSTR) | _BV(SPE);
    SPSR |= _BV(SPI2X);           // set double speed
}

void clock_init(){
#ifdef ARDUINO_ARCH_LGT   // не совсем понятно, тут должно быть правило для платы LGT8F, но это для platformio, а для arduinoIDE нужно что то другое
    CLKPR |= _BV(WCE) ;
    CLKPR = 0 ;
    asm("nop");
#endif    // для arduinoIDE используется эта часть
    ACSR = _BV(ACD) ;
} 

///
/// MAIN Routine
///////////////////////////////////////////
int main()
{   
    uint8_t  disp_index, btn_cnt, dir_level, pind; 
    uint8_t read_only  = 0;   
    clock_init();  
    TCCR1A = 0 ;
    TCCR1C = 0 ;
   /// INIT emulator --------------------------------------------------
    usart_init();
    port_init();
    spi_init() ;
    Wire.begin();  
    sei();   // ENABLE GLOBAL INTERRUPTS

#if (OLED == 1)
    // иницилизация дисплея oled SSD1306
    // ---------------------------- ----------------------------------------
    oled.init();                      // Initialze SSD1306 OLED display
    #if (OLED128x32 == 1)             //переключалка для 128х32
      oled.sendCommand(0xA8);         //SETMULTIPLEX
      oled.sendCommand(0x1F);         //128x32
      oled.sendCommand(0xDA);         //SETCOMPINS?
      oled.sendCommand(0x02);         //128x32
    #endif  
      
    #if (ROTATE == 1)                 // правило для поворота в config.h
    oled.sendCommand(0xA0);
    oled.sendCommand(0xC0);           // переворот экрана на 180
    #endif
    
    //oled.setBrightness(0x0);          // уменьшение яркости
    oled.clearDisplay();              // Clear screen
    //------------------------------------------------------------------------------
#else
      LCD_init();    //иницилизация LCD
#endif


    path = (char*)(sector_data + 32); // use as temporary buffer for path generation
    fat.buf = sector_data;

    /// ---------------------------------------------------------------
    while(1) { // MAIN LOOP START
        /// MAIN LOOP USED FOR SELECT and INIT SD CARD and other

     MOUNT:
        PCINT1_disable();

         oled.clearDisplay();
         LCD_clear();

     NO_FILES:

        memset(disp_files,0,sizeof(fnfo)*MAX_DISP_COUNT);
        pf_mount(0);
        while(pf_mount(&fat) != FR_OK){
         #if (OLED == 1)
        // oled.clearDisplay();
         #if (OLED128x32 == 1)
         oled.setTextXY(1,2);
         oled.putString(F("--No Card--"));
         #else
         oled.setTextXY(0,2);
         oled.putString(F("--No Card--"));
         oled.setTextXY(1,0);
         oled.putString(F("----------------"));
         oled.setTextXY(2,0);
         oled.putString(F("The original project of EvgenRU."));
         oled.setTextXY(4,0);
         oled.putString(F("Edited by:"));
         oled.setTextXY(5,0);
         oled.putString(F(".Evgeny Ugryumov"));
         oled.setTextXY(6,0);
         oled.putString(F(".RahostStudio"));
         oled.setTextXY(7,12);
         oled.putString(F("2024"));
         #endif
        #else
         //LCD_clear();
         #if (LCD2004 == 1)
          LCD_print(4,0,(F("--No Card--")));
          LCD_print(1,3,(F("t.me/RahostStudio")));
         #else 
          LCD_print(0,0,(F("NO CARD INSERTED"))); 
         #endif
       #endif     
        };

    #if (FAST_START == 1)        
      #if (OLED == 1)
        oled.clearDisplay();
      #else
        LCD_clear();
      #endif
      _delay_ms(100);
    #else
    logo(); //в конце кода void logo()
    #endif  

        uint32_t serial = card_read_serial();

        DESELECT(); // set SD card inactive
        dir_level = 0;   
          uint8_t eeprom_file = 0;
        eeprom_read_block((void*)path,(const void*)4,224); // read saved block with trd filename from eeprom
        dir_level = 0;    
        if(path[0] != 0){
            eeprom_file = 1;
            uint32_t serial2;
            eeprom_read_block((void*)&serial2,(const void*)0,4); // read saved card serial number from eeprom
            if(serial2 != serial) // compare saved serial number with current card serial number
            {
                eeprom_write_block((const void*)&serial, (void*)0, 4); // if not equal write sd card serial to eeprom
                eeprom_write_byte((uint8_t*)4, 0); // write zero value to eeprom for reset saved filename on next loop
                goto NO_FILES;
            }
            goto OPEN_FILE; // if serials equal jump to open file (file name read from eeprom)
        }
 
        /// SELECT TRD IMAGE HERE ----------------------------------------------------------------------------
        path[0] = '/';
        path[1] = 0 ;
        
DIRECTORY_LIST:
        LCD_clear();
        pf_opendir(&dir,path);
      
        disp_index = 0, f_index = 0;
        memset(disp_files,0,sizeof(fnfo)*MAX_DISP_COUNT);
      
        if(readdir0() == 0) {
            memcpy(&first_dir,&dir,sizeof(dir));
            f_index++;
        }
        for(uint8_t i =1 ;i < MAX_DISP_COUNT; i++){
            if(readdir(i,0,0) != 0) break ;
            f_index++;
            disp_index++ ;
        }      
        if(!f_index){
            if(!dir_level){
              #if (OLED == 1)
              oled.clearDisplay();
               #if (OLED128x32 == 1)
                 oled.setTextXY(1,2);
               #else
                 oled.setTextXY(3,1);
               #endif
                oled.putString(F("-- no files --"));
              #else
                LCD_clear();
                #if (LCD2004 == 1)
                 LCD_print(3,1,(F("-- No files --")));
                #else 
                  LCD_print(F("NO FILES")); 
                #endif
              #endif
                _delay_ms(3000);
                goto NO_FILES;
            }
            else{
                memset(disp_files,0,sizeof(fnfo)*MAX_DISP_COUNT);
                //memset(disp_files,0,sizeof(fnfo)*2); // original cod EvgenRU
                f_index = 1;
                disp_index = 0;
                disp_files[disp_index].fname[0] = '.';
                disp_files[disp_index].fname[1] = '.';
                disp_files[disp_index].fattrib |= AM_DIR;
            }
        }
        while(disp_index >0){
            disp_index-- ;
            shift_pos(1);
        }      
FILE_LIST:
    #if (OLED == 1)
        oled.clearDisplay();
        #if (OLED128x32 == 0)
        oled.setTextXY(0,0);              // Set cursor position
        oled.putString("ZX Disk Emul 2.0");
        oled.setTextXY(1,0);              // Set cursor position
        oled.putString("----------------");
        #endif
        print_files(disp_index);
     #else
        LCD_clear();
        print_files(disp_index);   
     #endif 

    // Buttons processing -----------------------------------------
        while(PINC & _BV(BTN)){
              while((PINC & _BV(ENC_A)) && (PINC & _BV(ENC_B))) {
                  if(!(PINC & _BV(BTN))) break;
              }
              if( serial != card_read_serial() ) goto MOUNT;
              
              if(! (PINC & _BV(ENC_A) )) { // button A pushed
              PRESS_A_AGAIN:
                if(f_index > 1){
                    if(disp_index < (f_index-1)) { // only move pointer
                        #if (OLED == 1)
                        oled.setTextXY(disp_index + START_DISP_POS ,1);
                        oled.putChar(0);
                        #else
                        //LCD_print_char(0,disp_index + START_DISP_POS ,0); //код Евгения Угрюмова
                        //LCD_print_char(0,1,0); // original cod EvgenRU
                        LCD_print_char(0,disp_index +1,0); // код Rahost'a
                        #endif  
                        disp_index++;
                        shift_pos(0);
                        #if (OLED == 1)  
                        oled.setTextXY(disp_index + START_DISP_POS ,1);
                        oled.putChar(62);
                        #else
                        //LCD_print_char(0,disp_index + START_DISP_POS ,32); //код Евгения Угрюмова
                        //LCD_print_char(0,0,32); // original cod EvgenRU
                        LCD_print_char(0,disp_index -1,32); // код Rahost'a
                        #endif
                    }
                    else { // load next entry
                        int8_t res = readdir(disp_index,0,1);
                        if(res == 0) {
                          #if (OLED == 0)
                          LCD_clear();
                          #endif
                          print_files(disp_index);
                        }
                        else if(res == -1) goto MOUNT;
                    }
                  }
                  // wait button released
                  uint8_t wait = 0;
                  while(! (PINC & _BV(ENC_A)) ) {
                    _delay_ms(100);
                    if(++wait == 3) goto PRESS_A_AGAIN;
                }
              }
  
              if(! (PINC & _BV(ENC_B)) ){ // button B pushed
                PRESS_B_AGAIN:
                  if(f_index > 1) {
                      if(disp_index > 0){ // only move pointer
                        #if (OLED == 1)
                        oled.setTextXY(disp_index + START_DISP_POS , 1);
                        oled.putChar(0);
                        #else
                        //LCD_print_char(0,disp_index + START_DISP_POS,0); //код Евгения Угрюмова
                        //LCD_print_char(0,0,0);  // original cod EvgenRU
                        LCD_print_char(0,disp_index -1,0); //код Rahost'a
                        #endif
                        disp_index--;   
                        shift_pos(1);     
                        #if (OLED == 1)         
                        oled.setTextXY(disp_index + START_DISP_POS, 1);
                        oled.putChar(62);
                        #else
                        //LCD_print_char(0,disp_index + START_DISP_POS,32); //код Евгения Угрюмова
                        //LCD_print_char(0,1,32); // original cod EvgenRU
                        LCD_print_char(0,disp_index +1,32); //код Rahost'a
                        #endif
                      }
                      else { // load previous entry
                        if(fnfo.fname[disp_index] != '.') // дебаг против двойного нажатия при выходе из папки (от Aleksandr Gofman 6.12.24)
                        {
                      int8_t res = readdir(disp_index,1,1);
                        if(res == 0) {
                          #if (OLED == 0)
                          LCD_clear();
                          #endif
                          print_files(disp_index);
                        }
                        else if(res == -1) goto MOUNT;  
                        }
                      }
                  }
                  // wait button released
                  uint8_t wait = 0;
                  while(! (PINC & _BV(ENC_B)) ){
                    _delay_ms(100);
                    if(++wait == 3) goto PRESS_B_AGAIN;
                  }
              }
        }

        btn_cnt = 0;
        while(!(PINC & _BV(BTN))){
            // wait button is released
            btn_cnt++;
            _delay_ms(100);
        }
        _delay_ms(300);
        
        pind = 0;
       
        // if directory selected oled
            if( disp_files[disp_index].fattrib & AM_DIR ){
            if(!memcmp(disp_files[disp_index].fname,"..",2)){
                if(dir_level > 0) dir_level--;
            }
            else 
            if(disp_files[disp_index].fname[0] != '.'){
                if(dir_level < MAX_DIR_LEVEL){
                    memcpy(&dirs[dir_level],disp_files[disp_index].fname,13);
                    dir_level++ ;
                } 
            }
            if(dir_level){          
                for(uint8_t i = 0; i < dir_level; i++){
                    path[pind++]='/' ;
                    memcpy(&path[pind],&dirs[i],strlen(dirs[i]));
                    pind += strlen(dirs[i]);
                }
                path[pind] = 0; 
            }
            else{
                path[0] = '/';
                path[1] = 0 ;    
            }

            PCINT1_disable();
            goto DIRECTORY_LIST;
        }
        
        PCINT1_disable();

        /// /END SELECT TRD IMAGE ------------------------------------------------------------------------------

        //////////////////////////////////////////////////////////////////////////////////
        // MOUNT TRD IMAGE and init Track Cluster table
        // disp_files[disp_index].fname contain short name (8.3) of selected TRD image
        //////////////////////////////////////////////////////////////////////////////////
        // -----------------------------------------------------------------------------------------------------
        if(dir_level){  
            for(uint8_t i = 0; i < dir_level; i++){
                memcpy(&path[pind],&dirs[i],strlen(dirs[i]));
                pind += strlen(dirs[i]);
                path[pind++]='/';
            }
        }    
        memcpy(path + pind, disp_files[disp_index].fname,strlen(disp_files[disp_index].fname));
        path[pind + strlen(disp_files[disp_index].fname)]=0;
        read_only = disp_files[disp_index].fattrib & AM_RDO ;

OPEN_FILE:        

        if(pf_open(path) != FR_OK){ // if unable to open file, usually if SD card is removed
            if(eeprom_file == 1) {
                DESELECT();
                eeprom_write_byte((uint8_t*)4, 0);
            }
            goto MOUNT;
        }
        oled.clearDisplay();
        oled.putString(F(">")); // вывод символа ">" перед именем выбранного файла
        oled.putString(0);      // вывод пробела после ">"  
       
        if(eeprom_file == 0){   // вывод имени файла на экран
           #if (OLED == 1)
           oled.putString(disp_files[disp_index].fname);
           #else
           LCD_clear();
           LCD_print_char(0);
           LCD_print_char(32);
           LCD_print(disp_files[disp_index].fname);
           #endif
            if(btn_cnt > 20){
               DESELECT();
               eeprom_write_block((const void*)&serial,(void*)0, 4);
               eeprom_write_block((const void*)path, (void*)4, strlen(path)+1);
            }
        }
        else{
            uint8_t ptr = strlen(path);
            while(path[ptr] != '/') ptr--;
            #if (OLED == 1)
            oled.putString((char*)&path[ptr+1]);
            #else
            LCD_print((char*)&path[ptr+1]);
            #endif
        }
            //---------------------------------------------------------------------------------------------------------------------------------------------------------------

#if (OLED == 1)
        #if (OLED128x32 == 1)
        oled.setTextXY(1,1);
        oled.putString("______________");
        oled.setTextXY(3,1);
        #else
        oled.setTextXY(6,1);
        #endif
        oled.putString(F("cyl:00  head:0") );
#else
 #if (LCD2004 == 1)
 LCD_print(0,3, F("CYL: 00  HEAD: 0") );
 #else
 LCD_print(0,1, F("CYL: 00  HEAD: 0") );
 #endif
#endif 

        max_cylinder = fat.fsize / 8192 + ((fat.fsize % 8192) ? 1 : 0); // calculate maximal cylinder
        if( max_cylinder > MAX_CYL ) max_cylinder = MAX_CYL; // if TRD image size too big set limit to MAX_CYL

        /// FAST create cluster table for cylinders ---------------------------------------------------------------------------------------
        uint32_t cur_fat = fat.org_clust, cur_fat_sector = cur_fat/128;
        if(card_readp(sector_data, fat.fatbase + cur_fat_sector, 0, 512) != RES_OK) goto MOUNT;
        clust_table[0] = cur_fat;
        for(uint16_t i = 1; i < max_cylinder*16; i++)
        { /// 16 SD sectors per cylinder
            if( i % fat.csize == 0) // cluster boundary
            {
                if( (cur_fat / 128) != cur_fat_sector )
                {
                     cur_fat_sector = cur_fat / 128;
                     card_readp(sector_data, fat.fatbase + cur_fat_sector, 0, 512) ;
                }
                cur_fat = (uint32_t)(*(uint32_t*)(sector_data + (((uint16_t)cur_fat << 2) & 0x1FF)));
            }
            if(i % 16 == 0) clust_table[i/16] = cur_fat;
        } // --------------------------------------------------------------------------------------------------------------------------------

        /// Emulator loop --------------
        uint8_t s_cylinder = 255;
        cylinder = 0;
        cylinder_changed = 0;
        uint8_t sector_length = 1 ;
        uint8_t sector_size  = 2;
        uint8_t max_sector_count = 16 ;

        _delay_ms(1500);

        if(read_only ){DDRD |= _BV(WP);}
        else {DDRD &= ~_BV(WP) ;}
        PCINT2_enable(); // ENABLE INDERRUPT (STEP pin)

        while (1){ /// DRIVE SELECT LOOP
            DDRB &= ~_BV(INDEX);
            DDRD &= ~_BV(TRK00);

LCD_light_off();        

            while ( ( PIND & (_BV(MOTOR_ON) | _BV(DRIVE_SEL)) ) != 0 ){  // wait drive select && motor_on
                if(!(PINC & _BV(BTN))){ // if button pressed                    
                    while(!(PINC & _BV(BTN))); // wait button is released
                    DESELECT();

LCD_light_on();

                    if(eeprom_file == 1){ // if filename from eeprom, reset eeprom data
                        eeprom_write_byte((uint8_t*)4, 0);
                        goto MOUNT;
                    }
                    PCINT2_disable(); // ENABLE INDERRUPT (STEP pin)
                    goto FILE_LIST;
                }
            }
           //oled.putString(F("")); //при комментировании компиляция не идет! стало норм
LCD_light_on();

            /// DEVICE ENABLED =========================================================================================================================|
            if(cylinder == 0) DDRD |= _BV(TRK00);
            else DDRD &= ~_BV(TRK00);
 
   //         PCINT2_enable(); // ENABLE INDERRUPT (STEP pin)
            uint8_t read_error = 0;            
            uint32_t sector_adress ;
            uint8_t real_sector = 0 ;
            do { // READ DATA LOOP (send data from FDD to FDD controller)
            //=================================================================================================================================]
             uint8_t tmpc;
              for(volatile uint16_t tmpcn = 0; tmpcn < 1000; tmpcn++) tmpc++; // wait for cylinder change detect
                if(s_cylinder != cylinder){ // if cylinder is changed we need to calculate sector table for current cylinder (for fast sectors read)
                    while( cylinder_changed ){ // wait while cylinder changing
                        ATOMIC_BLOCK(ATOMIC_FORCEON)cylinder_changed = 0;
                        for(volatile uint16_t tmpcn = 0; tmpcn < 10000; tmpcn++) tmpc++;
                    }
                    while(!(PIND & (1<<STEP)));
                    ATOMIC_BLOCK(ATOMIC_FORCEON)cylinder_changed = 0; 
                    s_cylinder = cylinder;
                    // FAST create cluster table for 32 cylinder sectors (sector_table) --------------
                    cur_fat = clust_table[s_cylinder];
                    cur_fat_sector = cur_fat / 128;
                    sector_table[0]  = cur_fat; // sector 1 offset on SD card
                    // process cluster chain
                    if(card_readp(sector_data, fat.fatbase + cur_fat_sector, 0, 512) != RES_OK) { read_error = 1;  break; }
                    for(uint8_t i = 1; i < 16; i++){ // TRD sector 256 bytes, SD card sector 512 bytes, so 32 TRD sectors = 16 sectors on SD
                        if((i % fat.csize == 0)){ // if cluster is changed
                            if( (cur_fat / 128) != cur_fat_sector ){
                                cur_fat_sector = cur_fat / 128;
                                card_readp(sector_data, fat.fatbase + cur_fat_sector, 0, 512); // read data_block with current cluster number
                            }
                            cur_fat = (uint32_t)(*(uint32_t*)(sector_data + ( ( (uint16_t)cur_fat << 2) & 0x1FF ) ));
                        }
                        sector_table[i] = cur_fat;  // 2 TRD sectors in same cluster
                    }
             //       while(!(PIND & (1<<STEP)));
             //       }

                }

                #if (OLED128x32 == 1)
                oled.setTextXY(3,5);
                #else
                oled.setTextXY(6,5);
                #endif
                oled.putNumber(cylinder / 10);
                oled.putNumber(cylinder % 10);
                 #if (LCD2004 == 1)
                 LCD_print(5,3,cylinder / 10); //для 2004 дисплея
                 LCD_print(cylinder % 10);
                 #else
                 LCD_print(5,1,cylinder / 10);
                 LCD_print(cylinder % 10);
                 #endif
                
                
                do { 
                    uint8_t side = ((~SIDE_PIN) & _BV(SIDE_SEL)) >> SIDE_SEL; // side detect
                    if(cylinder_changed || (PIND & _BV(MOTOR_ON))) break; // Stop sending track if cylinder is changed or FDD is disabled
                     // READ SECTOR DATA from SD card
                    // fat.database - start cluster of FAT32 filesystem
                    // fat.csize    - cluster size
                    USART_enable();
                    uint8_t sector = sector_order[real_sector];
                    uint8_t sector_fl = sector%2 ;
       //             if (sector_fl == 0)
                    {
                        sector_adress = fat.database + (sector_table[side*8 + sector/2] - 2) * fat.csize + ((s_cylinder*2 + side)*8 + sector/2) % fat.csize ;
                        if(card_readp(sector_data,sector_adress , 0,512) != RES_OK) { read_error = 1; break; }
                    }
                    if(cylinder_changed || (PIND & _BV(MOTOR_ON))) break; // Stop sending track if cylinder is changed or FDD is disabled

if(!LCD_check_light()) LCD_light_on(); // Enable LCD Light if FDD is active

                    // NOW WE'RE READY TO SEND SECTOR ===============================================================================>
                    if(!sector){ // if sector = 0
                        // print CYLINDER, HEAD INFO or track number on LCD
                   
                       #if (OLED == 1)
                        #if (OLED128x32 == 1)
                        oled.setTextXY(3,14);
                        #else
                        oled.setTextXY(6,14);
                        #endif
                        oled.putNumber(side);
                       #else 
                        #if (LCD2004 == 1)
                        LCD_print(15,3,side); //для 2004 дисплея
                        #else
                        LCD_print(15,1,side);
                        #endif
                        //
                       #endif 

                        // Send TRACK GAP4A --------------------------------------------------
                        USART_enable();
                        for(uint8_t cnt = 0; cnt < 10; cnt++) send_byte(0x4E);
                        DDRB |= _BV(INDEX); // SET INDEX LOW if sector = 0 (Start Index pulse at start of the track)
                    }
                    // Send sector Address Field + start data field --------------------------
                    uint8_t temp ;
                    uint8_t *temp_ptr ;
                    temp_ptr = &sector_data[sector_fl*256];

                    for(uint8_t cnt = 0; cnt < 12; cnt++){  send_byte(0); }
                    A1_mark = 1 ; 
                    CRC_H.val = 0xFFFF;
                    send_byte(0xA1);
                    send_byte(0xA1);
                    send_byte(0xA1);
                    A1_mark = 0; 
                    send_byte(0xFE);
                    send_byte(s_cylinder);
                    send_byte(side);
                    send_byte(sector+1);
                    send_byte(sector_length);
                    uint8_t tmp = CRC_H.bytes.low ;
                    send_byte(CRC_H.bytes.high);
                    send_byte(tmp) ;
                    for(uint8_t cnt = 0; cnt < 22; cnt++){  send_byte(0x4E); if((PINC & _BV(WG)) == 0) break ;}
                    for(uint8_t cnt = 0; cnt < 12; cnt++){  send_byte(0); if((PINC & _BV(WG)) == 0) break ;}
                    if((PINC & _BV(WG)) == 0){
            //           PORTB|= _BV(6) ;
                        USART_disable(); // DISABLE USART INDERRUPT after sending track
                        START_CAPTURE ;
                        
                        if(!sector) DDRB &= ~_BV(INDEX);
                        int8_t ret=  read_sector_(temp_ptr,sector_size) ;
                        TCCR1B = 0  ;  // stop timer 
                        USART_enable(); // ENABLE USART INDERRUPT after recieve sector
                        if(ret == 0 ){
                            if((ret  = card_writep(0, sector_adress ) ) == 0){
                                if((ret = card_writep(sector_data , 512) ) == 0){
                                    if((ret = card_writep(0 , 0) ) != 0){
                                        oled.setTextXY(2,12);
                                        oled.putNumber(ret);
                                    }; 
                                }else{   
                                    oled.setTextXY(2,7);
                                    oled.putNumber(ret);
                                }
                            }else{   
                               oled.setTextXY(2,2);
                               oled.putNumber(ret);
                            };
                        };
                    }else{            
                        A1_mark = 1 ; 
                        CRC_H.val = 0xFFFF;
                        send_byte(0xA1);
                        send_byte(0xA1);
                        send_byte(0xA1);
                        A1_mark = 0; 
                        send_byte(0xFB);
                        // ----------------------------------------------------------------------
                        if(!sector) DDRB &= ~_BV(INDEX); // SET INDEX HIGH if sector = 0 (End Index pulse at start of the track)
                        if(cylinder_changed || (PIND & (_BV(MOTOR_ON) | _BV(DRIVE_SEL)))) break; // Stop sending track if cylinder is changed or FDD is not active
                        // Send sector data (256 bytes) -----------------------------------------
                        uint8_t cnt1 = sector_size ;
                        do{
                            for(uint8_t cnt = 0;cnt < 128 ;cnt++){
                                temp = *temp_ptr++ ;
                                send_byte(temp );
                            }
                            cnt1-- ;
                        }while(cnt1 ) ;
                        //Send sector data CRC -------------------------------------------------
                        tmp = CRC_H.bytes.low ;
                        send_byte(CRC_H.bytes.high);
                        send_byte(tmp) ;
                       // Send sector GAP ------------------------------------------------------
                        for(uint8_t cnt = 0; cnt < 54; cnt++){  send_byte(0x4E);  if(cylinder_changed || (PIND & (_BV(MOTOR_ON) | _BV(DRIVE_SEL)))) break;  }    
                    }       // ----------------------------------------------------------------------
                        if(cylinder_changed || (PIND & _BV(MOTOR_ON))) break; // Stop sending track if cylinder is changed or FDD is disabled
                        // END SEND SECTOR ==============================================================================================>
                        real_sector++ ;
                        if(real_sector == max_sector_count) real_sector = 0 ;
                        sector = sector_order[real_sector] ;
                } while( 1 )  ;
                if(read_error) break;
            } while(  (PIND & ( _BV(MOTOR_ON) | _BV(DRIVE_SEL) )) == 0 ); // READ DATA SEND LOOP END
            //=================================================================================================================================]
            USART_disable(); // DISABLE USART INDERRUPT after sending track
      
             DDRB &= ~_BV(INDEX); // SET INDEX HIGH
  //          DDRD &= ~(_BV(WP) | _BV(TRK00)); // Set WP,TRK00 as input
              //============================================================================================================]
            //oled.setTextXY(3,0);
            //oled.putString(F(" "));

#if (OLED == 0)
LCD_light_off();
#endif
    
            DESELECT(); // disconnect SD Card
            if(read_error) break;
            /// DEVICE DISABLED ========================================================================================================================|
        } /// DRIVE SELECT LOOP END
        PCINT2_disable(); // DISABLE PCINT INDERRUPT (STEP pin)
    } // MAIN LOOP END
} // END MAIN


int8_t read_sector_(uint8_t *buf, uint8_t sector_size){
    int16_t size = (uint16_t)sector_size * (128) ;
    uint8_t ret ;
#define PULSE 8
    asm volatile (
        // define READPULSE macro (wait for pulse)
        // macro arguments: 
        //         length: none => just wait for pulse, don't check         ( 9 cycles)
        //                 1    => wait for pulse and jump if NOT short  (12/14 cycles)
        //                 2    => wait for pulse and jump if NOT medium (14/16 cycles)
        //                 3    => wait for pulse and jump if NOT long   (12/14 cycles)
        //         dst:    label to jump to if DIFFERENT pulse found
        // 
        //           r18 contains time of previous pulse
        // on exit:  r18 is updated to the time of this pulse
        //           r22 contains the pulse length in timer ticks (=processor cycles)     
        // CLOBBERS: r19
                    ".macro READPULSE length=0,dst=undefined\n"
                    "  1:    sbic   PINC , 0\n"
                    "        rjmp   rddone\n "
                    "        sbis   TIFR1, ICF1\n"     // (1/2) skip next instruction if timer input capture seen
                    "        rjmp   1b\n"           // (2)   wait more 
         //           "        sbi     PORTB , 6 \n    "
                    "        lds     r19, ICR1L\n"     // (2)   get time of input capture (ICR1L, lower 8 bits only)
                    "        sbi     TIFR1, ICF1\n"    // (2)   clear input capture flag
                    "        mov     r22, r19\n"      // (1)   calculate time since previous capture...
                    "        sub     r22, r18\n"      // (1)   ...into r22
                    "        mov     r18, r19\n"      // (1)   set r18 to time of current capture
                    ".if \\length == 1\n"           //       waiting for short pule?
                    "        cpi      r22, MED_PULSE\n"      // (1)   compare r22 to min medium pulse
                    "        brlo   2f\n"            // (1/2) skip jump if less
                    "        rjmp   \\dst\n"          // (3)   not the expected pulse => jump to dst
                    " 2:      \n "
                    "   .else \n"
                    "   .if \\length == 2\n"         // waiting for medium pulse?
                    "        cpi      r22 , MED_PULSE\n"      // (1)   min medium pulse < r22? => carry set if so
                    "        brsh    3f\n"           // (1/2) skip next instruction if carry is clear      
                    "        rjmp   \\dst\n"          // (3)   not the expected pulse => jump to dst
                    "  3:      cpi      r22, LONG_PULSE\n"      // (1)   r22 < min long pulse? => carry set if so
                    "        brlo   4f\n"            // (1/2) skip jump if greater
                    "        rjmp   \\dst\n"          // (3)   not the expected pulse => jump to dst
                    "  4:    \n"
                    "   .else\n"
                    "   .if \\length == 3\n" 
                    "        cpi      r22, LONG_PULSE\n"      // (1)   min long pulse < r22?
                    "        brsh   5f\n"            // (1/2) skip jump if greater
                    "        rjmp   \\dst\n"          // (3)   not the expected pulse => jump to dst
                    "  5: \n"
                    "   .endif\n"
                    " .endif\n"
                    " .endif\n"
                    ".endm\n"
                            // define STOREBIT macro for storing or verifying data bit 
                            // storing  data  : 5/14 cycles for "1", 4/13 cycles for "0"
                            ".macro STOREBIT data:req,done:req\n"
                            "        lsl     r20\n"           // (1)   shift received data
                            ".if \\data != 0\n"
                            "        ori     r20, 1\n"        // (1)   store "1" bit
                            ".endif\n"
                            "        dec     r21\n"           // (1)   decrement bit counter
                            "        brne    6f\n"          // (1/2) skip if bit counter >0
                //            "        cbi     PORTB , 6 \n"
                            "        sbrc    r16 , 0\n"       // РЅРµ РїРёС€РµРј РїРµСЂРІС‹Р№ Р±Р°Р№С‚
                            "        st      Z+, r20\n"       // (2)   store received data byte
                            "        ser     r16   \n "
                            "        ldi     r21, 8\n"        // (1)   re-initialize bit counter
                            "        subi    r26, 1\n"        // (1)   subtract one from byte counter
                            "        sbci    r27, 0\n"        // (1) 
                            "        brmi    \\done\n"        // (1/2) done if byte counter<0
                            "  6:\n"
                            ".endm\n"
                            // prepare for reading SYNC
          //                  " .align 8 \n"
                            "        .equ TIFR1,    0x16\n"  // timer 1 flag register
                            "        .equ ICF1,     5\n"     // input capture flag
                            "        .equ ICR1L,    0x86\n"  // timer 1 input capture register (low byte)
                            "        .equ ICR1H,    0x87\n"  // timer 1 input capture register (low byte)
                            "        .equ PULSE ,  %[pu] \n"    // 
                            "        .equ MED_PULSE , PULSE * 5 / 2\n"
                            "        .equ LONG_PULSE , PULSE * 7 / 2  \n"
                            "        .equ PINC    , 0x06 \n"
                            "        .equ PORTB   , 0x05 \n"
                            // expect remaining part of first sync mark (..00010010001001)
                //            "        cbi     PORTB , 6 \n       "
                            "ws:                    \n"
                            "        READPULSE   1,ws\n"     // (12)  expect short pulse (01)
                            "ws0:                      \n"
                            "        READPULSE   3,ws0\n"     // (12)  expect long pulse (0001)
                            "        READPULSE   2,ws0\n"     // (14)  expect medium pulse (001)
                            "        READPULSE   3,ws0\n"     // (12)  expect long pulse (0001)
                            "        READPULSE   2,ws0\n"     // (14)  expect medium pulse (001)
                //            "        cbi     PORTB , 6 \n         "
                             // expect second sync mark (0100010010001001)
                            "        READPULSE   1,ws0\n"     // (12)  expect short pulse (01)
                            "        READPULSE   3,ws0\n"     // (12)  expect long pulse (0001)
                            "        READPULSE   2,ws0\n"     // (14)  expect medium pulse (001)
                            "        READPULSE   3,ws0\n"     // (12)  expect long pulse (0001)
                            "        READPULSE   2,ws0\n"     // (14)  expect medium pulse (001)
                //         "        cbi     PORTB , 6 \n         "
                            // expect third sync mark (0100010010001001)
                            "        READPULSE   1,ws0\n"     // (12)  expect short pulse (01)
                            "        READPULSE   3,ws0\n"     // (12)  expect long pulse (0001)
                            "        READPULSE   2,ws0\n"     // (14)  expect medium pulse (001)
                            "        READPULSE   3,ws0\n"     // (12)  expect long pulse (0001)
                            "        READPULSE   2,ws0\n"     // (14)  expect medium pulse (001)
                //            "        cbi     PORTB , 6 \n         "
                            // found SYNC => prepare for reading data
                            "        ldi     r21, 8\n"        // (1)   initialize bit counter (8 bits per byte)
                            "        clr     r16\n"
                            "        clr     %[ret_val]\n"
                            // odd section (previous data bit was "1", no unprocessed MFM bit)
                            // shortest path: 19 cycles, longest path: 34 cycles
                            // (longest path only happens when finishing a byte, about every 5-6 pulses)
                            "rdo:    READPULSE\n"             // (9)   wait for pulse
                            "        cpi      r22, MED_PULSE\n"      // (1)   pulse length >= min medium pulse?
                            "        brlo    rdos\n"          // (1/2) jump if not
                            "        cpi      r22, LONG_PULSE\n"      // (1)   pulse length >= min long pulse?
                            "        brlo    rdom\n"          // (1/2) jump if not

                            // long pulse (0001) => read "01", still odd
                            "        STOREBIT 0,rddone\n"      // (4/13) store "0" bit
                            "        STOREBIT 1,rddone\n"      // (5/14) store "1" bit
                            "        rjmp    rdo\n"            // (2)    back to start (still odd)

                            // jump target for relative conditional jumps in STOREBIT macro
                            "rddone:  rjmp    rdend\n"      

                            // medium pulse (001) => read "0", now even
                            "rdom:      STOREBIT 0,rddone\n"      // (4/13) store "0" bit
                            "        rjmp    rde      \n"            // (2)   back to start (now even)

                            // short pulse (01) => read "1", still odd
                            "rdos:   STOREBIT 1,rddone\n"      // (5/14) store "1" bit
                            "        rjmp    rdo\n"            // (2)    back to start (still odd)

                            // even section (previous data bit was "0", previous MFM "1" bit not yet processed)
                            // shortest path: 19 cycles, longest path: 31 cycles
                            "rde:    READPULSE\n"             // (9)   wait for pulse
                            "        cpi      r22, MED_PULSE\n"      // (1)   pulse length >= min medium pulse?
                            "        brlo    rdes\n"          // (1/2) jump if not  

                            // either medium pulse (1001) or long pulse (10001) => read "01"
                            // (a long pulse should never occur in this section but it may just be a 
                            //  slightly too long medium pulse so count it as medium)
                            "        STOREBIT 0,rdend\n"      // (4/13) store "0" bit
                            "        STOREBIT 1,rdend\n"      // (5/14) store "1" bit
                            "        rjmp    rdo\n"           // (2)    back to start (now odd)

                            // short pulse (101) => read "0"
                            "rdes:   STOREBIT 0,rdend\n"      // (5/14) store "0" bit
                            "        rjmp    rde\n"           // (2)    back to start (still even)
                            "rdend:\n"
                 //           "        cbi PORTB , 6 \n"

                            :  [ret_val] "=r"(ret)            // outputs
                            : "x"(size), "z"(buf) , [pu] "M"(LENGTH_PULSE)  // inputs  (x=r26/r27, z=r30/r31)
                            :  "r16" , "r18", "r19", "r20", "r21", "r22");  // clobbers
   
    return ret ;    
}

void send_byte(uint8_t sector_byte)
{
    /// inverted, very small MFM table for fast converting
    static uint8_t MFM_tab[8] = { 0x77,0x7D,0xDF,0xDD,0xF7,0xFD,0xDF,0xDD };
    static uint8_t  prev_byte = 0  ;
    uint8_t temp1,temp2, temp3 , tmp ;
     
     tmp = sector_byte ;
     swap(tmp) ;
   
     temp1 =  MFM_tab[tmp & 0x07];
     tmp = (tmp /4) & 0x03 ; 
     if((prev_byte & 1)&& !(sector_byte & 0x80)) tmp |= 0x04;
     loop_until_bit_is_set(UCSR0A,UDRE0); // wait USART buffer is ready for the next byte
     if( (PIND & (_BV(MOTOR_ON) | _BV(DRIVE_SEL)))){USART_disable() ; return ;} ;
     UDR0 = MFM_tab[tmp];
     prev_byte = sector_byte;
     
     temp2 = A1_mark ? 0x7F : MFM_tab[ (sector_byte / 4 ) & 0x07];
     temp3 = MFM_tab[ sector_byte & 0x07];
     
    loop_until_bit_is_set(UCSR0A,UDRE0); // wait USART buffer is ready for the next byte
    if( (PIND & (_BV(MOTOR_ON) | _BV(DRIVE_SEL)))){USART_disable() ; return ;} ;
    UDR0 = temp1;
    CRC_H.val = (CRC_H.bytes.low << 8) ^ pgm_read_word_near(Crc16Table + (CRC_H.bytes.high ^ sector_byte));
    loop_until_bit_is_set(UCSR0A,UDRE0); // wait USART buffer is ready for the next byte
    if( (PIND & (_BV(MOTOR_ON) | _BV(DRIVE_SEL)))){USART_disable() ; return ;} ;
    UDR0 = temp2;
    
    loop_until_bit_is_set(UCSR0A,UDRE0); // wait USART buffer is ready for the next byte
    UDR0 = temp3;
   
}

void send_byte_(uint8_t sector_byte)
{
    /// inverted, very small MFM table for fast converting
    static uint8_t MFM_tab[8] = { 0x77,0x7D,0xDF,0xDD,0xF7,0xFD,0xDF,0xDD };
    static uint8_t  prev_byte = 0  ;

    uint8_t tmp = sector_byte >> 6; // get first MFM byte from table (first 4 bits)
    
    if((prev_byte & 1) && !(sector_byte & 0x80)) tmp |= 0x04; // check previous last bit and correct first clock bit of a new byte
    
    loop_until_bit_is_set(UCSR0A,UDRE0); // wait USART buffer is ready for the next byte
 //   if( (PIND & (_BV(MOTOR_ON) | _BV(DRIVE_SEL)))){USART_disable() ; return ;} ;
    UDR0 = MFM_tab[tmp];

    loop_until_bit_is_set(UCSR0A,UDRE0); // wait USART buffer is ready for the next byte
 //   if( (PIND & (_BV(MOTOR_ON) | _BV(DRIVE_SEL)))){USART_disable() ; return ;} ;
    UDR0 = MFM_tab[(sector_byte >> 4) & 0x07]; // get first MFM byte from table (second 4 bits)

    loop_until_bit_is_set(UCSR0A,UDRE0); // wait USART buffer is ready for the next byte
  //  if( (PIND & (_BV(MOTOR_ON) | _BV(DRIVE_SEL)))){USART_disable() ; return ;} ;
    UDR0 = A1_mark ? 0x7F : MFM_tab[(sector_byte >> 2)& 0x07]; // get second MFM byte from table (first 4 bits)

    prev_byte = sector_byte;
    CRC_H.val = (CRC_H.bytes.low << 8) ^ pgm_read_word_near(Crc16Table + (CRC_H.bytes.high ^ sector_byte));
    loop_until_bit_is_set(UCSR0A,UDRE0); // wait USART buffer is ready for the next byte
 //  if( (PIND & (_BV(MOTOR_ON) | _BV(DRIVE_SEL)))){USART_disable() ; return ;} ;
    UDR0 = MFM_tab[sector_byte & 0x07]; // get second MFM byte from table (second 4 bits)

}
void logo()  //блок для красоты запуска
{
  #if (OLED == 1)
        oled.clearDisplay();
        #if (OLED128x32 == 1)
        oled.setTextXY(1,1);
        oled.putString(F("ZX FDD Emul 2.0"));
_delay_ms(100);
// Подпись RahostStudio внизу, с эффектом
       for (byte i = 0; i < 3; i++){ 
        oled.setTextXY(3,2);
        oled.putString(F("R"));
        _delay_ms(20);
        oled.setTextXY(3,3);
        oled.putString(F("a"));
        _delay_ms(20);
        oled.setTextXY(3,4);
        oled.putString(F("h"));
        _delay_ms(20);
        oled.setTextXY(3,5);
        oled.putString(F("o"));
        _delay_ms(20);
        oled.setTextXY(3,6);
        oled.putString(F("s"));
        _delay_ms(20);
        oled.setTextXY(3,7);
        oled.putString(F("t"));
        _delay_ms(20);
        oled.setTextXY(3,8);
        oled.putString(F("S"));
        _delay_ms(20);
        oled.setTextXY(3,9);
        oled.putString(F("t"));
        _delay_ms(20);
        oled.setTextXY(3,10);
        oled.putString(F("u"));
        _delay_ms(20);
        oled.setTextXY(3,11);
        oled.putString(F("d"));
        _delay_ms(20);
        oled.setTextXY(3,12);
        oled.putString(F("i"));
        _delay_ms(20);
        oled.setTextXY(3,13);
        oled.putString(F("o"));
        _delay_ms(50);
for (byte z = 2; z < 14; z++){ 
 oled.setTextXY(3,z);
 oled.putString(F(" "));   
 _delay_ms(20);    
    }
  }
        _delay_ms(200);
        oled.setTextXY(1,1);
        oled.putString(F("  "));
        _delay_ms(200);
        oled.putString(F("    "));
        _delay_ms(200);
        oled.putString(F("     "));
        _delay_ms(200);
        oled.putString(F("    "));
        _delay_ms(200);

        #else
        _delay_ms(200);
// Вывод полос слева и справа для 128х64
int a = 15;
int b = 0;
for (a, b; a > 0, b < 16; a--, b++) { 

oled.setTextXY(2,a);
oled.putString(F("-"));
oled.setTextXY(4,b);
oled.putString(F("-"));
_delay_ms(30);
}
// Надпись
        _delay_ms(200);
        oled.setTextXY(3,0);
        oled.putString(F("ZX FDD Emul  2.0"));
        
        _delay_ms(300);
// Подпись RahostStudio внизу, с эффектом
       for (byte i = 0; i < 3; i++){ 
        oled.setTextXY(7,2);
        oled.putString(F("R"));
        _delay_ms(20);
        oled.setTextXY(7,3);
        oled.putString(F("a"));
        _delay_ms(20);
        oled.setTextXY(7,4);
        oled.putString(F("h"));
        _delay_ms(20);
        oled.setTextXY(7,5);
        oled.putString(F("o"));
        _delay_ms(20);
        oled.setTextXY(7,6);
        oled.putString(F("s"));
        _delay_ms(20);
        oled.setTextXY(7,7);
        oled.putString(F("t"));
        _delay_ms(20);
        oled.setTextXY(7,8);
        oled.putString(F("S"));
        _delay_ms(20);
        oled.setTextXY(7,9);
        oled.putString(F("t"));
        _delay_ms(20);
        oled.setTextXY(7,10);
        oled.putString(F("u"));
        _delay_ms(20);
        oled.setTextXY(7,11);
        oled.putString(F("d"));
        _delay_ms(20);
        oled.setTextXY(7,12);
        oled.putString(F("i"));
        _delay_ms(20);
        oled.setTextXY(7,13);
        oled.putString(F("o"));
        _delay_ms(50);
for (byte z = 2; z < 14; z++){ 
 oled.setTextXY(7,z);
 oled.putString(F(" "));   
 _delay_ms(20);    
}
_delay_ms(50);

       if (i == 3) break;  // сколько раз повторить
               }
// угасание надписей
        _delay_ms(200);
        oled.setTextXY(2,0);
        oled.putString(F("                ")); //полоса верхняя
        oled.setTextXY(4,0);
        oled.putString(F("                ")); //полоса нижняя
        _delay_ms(500);
        oled.setTextXY(3,0);
        oled.putString(F("  ")); //надпись
        _delay_ms(150);
        oled.putString(F("     "));
        _delay_ms(150);
        oled.putString(F("     "));
        _delay_ms(150);
        oled.putString(F("    "));

        _delay_ms(200);
        #endif
        
      #else  //эффекты для ЛСД
        LCD_clear();
        #if (LCD2004 == 1)
        LCD_print(2,1,(F("ZX FDD Emul  2.0")));

_delay_ms(50);
// Вывод полос слева и справа
int a = 19;
int b = 0;
for (a, b; a > 0, b < 20; a--, b++) { 

LCD_print(a,0,(F("-")));
LCD_print(b,2,(F("-")));
_delay_ms(50);
}

LCD_print(0,3,(F("for t.me/zx_delta_s")));
_delay_ms(1500);
LCD_print(0,0,(F("                    ")));
LCD_print(0,2,(F("                    ")));
_delay_ms(500);
LCD_print(0,1,(F("                    ")));

        _delay_ms(500);
        #else
        LCD_print(0,0,(F("ZX")));
        _delay_ms(200);
        LCD_print(F(" FDD"));
        _delay_ms(200);
        LCD_print(F(" Emul"));
        _delay_ms(200);
        LCD_print(F("  2.0"));
        _delay_ms(1000);
        LCD_print(0,0,(F("  ")));
        _delay_ms(200);
        LCD_print(F("    "));
        _delay_ms(200);
        LCD_print(F("     "));
        _delay_ms(200);
        LCD_print(F("     "));
        _delay_ms(100);
        #endif
      #endif
      _delay_ms(100);
}

// end of code. 
// EvgenRU && Evgeniy Ugryumov && rahostStudio && more users )). 
// last update 07.12.2024 