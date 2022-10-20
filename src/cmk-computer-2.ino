/****************************************************************\
 ================================================================

                           8-bit computer
                      
                                 by
                     
                          Code Monkey King

 ================================================================
\****************************************************************/

// libraries
#include <EEPROM.h>
#include <avr/pgmspace.h> 
#include <LiquidCrystal.h>
#include <Keypad.h>

/****************************************************************\    
================================================================
        CIRCUIT (LCD shield 1602A D1 ROBOT (DF ROBOT))
================================================================
            LCD RS pin to arduino digital pin 8
            LCD Enable pin to arduino digital pin 9
            LCD Backlight LED to arduino pin 10
            LCD D4 pin to arduino digital pin 4
            LCD D5 pin to arduino digital pin 5
            LCD D6 pin to arduino digital pin 6
            LCD D7 pin to arduino digital pin 7
            LCD A0  to arduino A0 pin (LCD shield keypad) 
            LCD VIN to arduino VIN pin
            LCD GND to arduino GND pin
            LCD 5V to arduino 5V pin
            LCD RST to arduino RST pin
            KEYPAD row 1 to arduino digital pin 12
            KEYPAD row 2 to arduino digital pin 11
            KEYPAD row 3 to arduino digital pin 3
            KEYPAD row 4 to arduino digital pin 2
            KEYPAD col 1 to arduino analog pin 4
            KEYPAD col 2 to arduino analog pin 3
            KEYPAD col 3 to arduino analog pin 2
            KEYPAD col 4 to arduino analog pin 1
            
================================================================
\****************************************************************/

// uncomment to rotate keypad CCW, enable LCD shield buttons
//#define CMK_HARDWARE

// messages
const char MESSAGE_ZERO[] PROGMEM = {"0"};
const char MESSAGE_REGISTER_A_DEBUG[] PROGMEM = {"Register A: 0x"};
const char MESSAGE_REGISTER_B_DEBUG[] PROGMEM = {"Register B: 0x"};
const char MESSAGE_REGISTER_PC_DEBUG[] PROGMEM = {"Program Counter: 0x"};
const char MESSAGE_REGISTER_SP_DEBUG[] PROGMEM = {"Stack pointer: 0x"};
const char MESSAGE_REGISTER_ZF_DEBUG[] PROGMEM = {"Zero Flag: 0x"};
const char MESSAGE_UNKNOWN_OPCODE[] PROGMEM = {"Unknown opcode:"};
const char MESSAGE_QUESTION_MARK[] PROGMEM = {"? "};
const char MESSAGE_CMK[] PROGMEM = {"Code Monkey King"};
const char MESSAGE_NEW[] PROGMEM = {"NEW  "};
const char MESSAGE_LOAD[] PROGMEM = {"LOAD "};
const char MESSAGE_SAVE[] PROGMEM = {"SAVE "};
const char MESSAGE_CLEAR[] PROGMEM = {"CLEAR"};
const char MESSAGE_RUN[] PROGMEM = {"RUN  "};
const char MESSAGE_VIEW[] PROGMEM = {"VIEW: "};
const char MESSAGE_WAITING[] PROGMEM = {" Waiting for"};
const char MESSAGE_INCOMING[] PROGMEM = {"incoming data..."};
const char MESSAGE_LOADING[] PROGMEM = {"Loading..."};
const char MESSAGE_DONE[] PROGMEM = {"  done"};
const char MESSAGE_DONE_LONG[] PROGMEM = {"   done"};
const char MESSAGE_SAVING[] PROGMEM = {"Saving..."};

// LCD pins
#define RS 8    // LCD reset pin
#define En 9    // LCD enable pin
#define D4 4    // LCD data pin 4
#define D5 5    // LCD data pin 5
#define D6 6    // LCD data pin 6
#define D7 7    // LCD data pin 7

// init LCD
LiquidCrystal lcd(RS, En, D4, D5, D6, D7);

// keypad size
const byte num_rows= 4;
const byte num_cols= 4;

#ifdef CMK_HARDWARE
// map keypad keys
uint8_t keymap[num_rows][num_cols] = {
  {'A', 'B', 'C', 'D'},
  {'3', '6', '9', 'F'},
  {'2', '5', '8', '0'},
  {'1', '4', '7', 'E'}
};
#else
// map keypad keys
uint8_t keymap[num_rows][num_cols] = {
  { '1', '2', '3', 'A'},
  { '4', '5', '6', 'B'},
  { '7', '8', '9', 'C'},
  { 'E', '0', 'F', 'D'}
};
#endif

// map keypad rows and columns
byte row_pins[num_rows] = {12, 11, 3, 2};     // Rows 0 to 3
byte col_pins[num_cols] = {A4, A3, A2, A1};  // Columns 0 to 3


// init kepad
Keypad keypad = Keypad(makeKeymap(keymap), row_pins, col_pins, num_rows, num_cols);

// define opcodes
#define NOP 0x00
#define LDI 0x01
#define LDA 0x02
#define TAB 0x03
#define ADD 0x04
#define SUB 0x05
#define STA 0x06
#define RCH 0x07
#define LPC 0x08
#define INC 0x09
#define DCR 0x0a
#define CMP 0x0b
#define JMP 0x0c
#define DBG 0x0d
#define IN  0x0e
#define OUT 0x0f
#define BIT 0x10
#define AND 0x11
#define OR  0x12
#define XOR 0x13
#define NOT 0x14
#define SHL 0x15
#define SHR 0x16
#define CLS 0x17
#define SDL 0x18
#define SDR 0x19
#define CRS 0x1a
#define NCR 0x1b
#define UDG 0x1c
#define SPR 0x1d
#define POS 0x1e
#define DLY 0x1f
#define RND 0x20
#define PSH 0x21
#define POP 0x22
#define SBR 0x23
#define RET 0x24
#define NUM 0x25
#define INM 0x26
#define DCM 0x27
#define SER 0x28

// define commands
#define CLEAR 0xfffa
#define NEW   0xfffb
#define VIEW  0xfffc
#define LOAD  0xfffd
#define SAVE  0xfffe
#define RUN   0xffff

// define memory size
#define RAM_SIZE 1024
#define ROM_SIZE 2048

// RAM array
uint8_t RAM[RAM_SIZE];

// ROM array
const uint8_t ROM[ROM_SIZE] PROGMEM = {
  // ROM
  0x08, 0x00, 0x00
  //0x1b, 0x01, 0x00, 0x03, 0x1e, 0x26, 0x00, 0x00, 0x08, 0x04, 0x2b, 0x02, 0x04, 0x1c, 0x0b, 0x00, 0x0c, 0x04, 0x17, 0x0f, 0x09, 0x08, 0x04, 0x0a, 0x25, 0x11, 0x11, 0x08, 0x04, 0x2b, 0x20, 0x41, 0x44, 0x44, 0x52, 0x45, 0x53, 0x53, 0x20, 0x20, 0x44, 0x41, 0x54, 0x41, 0x00, 0x08, 0x04, 0x2b
};

// CPU registers
uint8_t  register_A = 0;
uint8_t  register_B = 0;
uint16_t  program_counter = 0x0400;
uint16_t stack_pointer = 0x3ff;
bool zero_flag = 0;


/****************************************************************\
 ================================================================

                              MEMORY

 ================================================================
\****************************************************************/

// clear RAM
void reset_memory() {
  for (uint16_t i = 0; i < RAM_SIZE; i++)
    RAM[i] = 0;
}

// read byte from RAM
/*uint8_t read_byte() {
  uint8_t value;
  if (program_counter >= 0x0000 && program_counter <= 0x03FF)
    value = RAM[program_counter];
  if (program_counter >= 0x0400 && program_counter <= 0x0BFF)
    value = pgm_read_byte_near(ROM + (program_counter - 0x0400));
  program_counter++;
  return value;
}*/

uint8_t read_byte(uint16_t addr) {
  uint8_t value;
  if (addr >= 0x0000 && addr <= 0x03FF)
    value = RAM[addr];
  if (addr >= 0x0400 && addr <= 0x0BFF)
    value = pgm_read_byte_near(ROM + (addr - 0x0400));
  program_counter++;
  
  
  /*if (addr != program_counter) {
    Serial.print(addr, HEX);
    Serial.print(": ");
    Serial.print(value, HEX);
    Serial.print("\n");
  }*/ //getch();
  
  
  return value;
}


// read word from RAM
uint16_t read_word() { 
  uint8_t MSB = read_byte(program_counter);
  uint8_t LSB = read_byte(program_counter);
  uint16_t value = MSB;
  value <<= 8;
  value |= LSB;
  return value;
}

// encode address
uint16_t encode_word() {
  uint16_t addr = 0;
  for (int i = 12; i >= 0; i -= 4) {  
    char input = getch();
    uint8_t hex = ascii_to_hex(input);
    if (i) addr |= hex << i;
    else addr |= hex;
    lcd.print(hex, HEX);
    
  }
  delay(200);
  return addr;
}

// encode byte
uint8_t encode_byte() {
  uint8_t value = 0;
  for (int i = 4; i >= 0; i -= 4) {  
    char input = getch();
    uint8_t hex = ascii_to_hex(input);
    if (i) value |= hex << i;
    else value |= hex;
    lcd.print(hex, HEX);
  } return value;
}

// prints 8-bit data in hex with leading zeroes
void print_byte(uint8_t  byte) {
  if (byte<0x10) print_message_lcd(MESSAGE_ZERO);
  lcd.print(byte, HEX);
  lcd.print(' ');
}

// prints 16-bit data in hex with leading zeroes
void print_word(uint16_t  word) {
  uint8_t  MSB = byte(word >> 8);
  uint8_t  LSB = byte(word);
  if (MSB<0x10) { print_message_lcd(MESSAGE_ZERO); } lcd.print(MSB,HEX);
  if (LSB<0x10) { print_message_lcd(MESSAGE_ZERO); } lcd.print(LSB,HEX);
}

// dump memory
void memory_dump(uint16_t addr) {
  lcd.clear();
  print_word(addr); lcd.print(':');
  for (uint16_t i = addr; i < addr + 4; i++) {
    if (i >= 0x0000 && i <= 0x03FF) print_byte(RAM[i]);
    if (i >= 0x0400 && i <= 0x0BFF) print_byte(pgm_read_byte_near(ROM + (i - 0x0400)));
  }
  lcd.setCursor(0, 1);
}

// prints 8-bit data in hex with leading zeroes
void print_hex(uint8_t data)
{  
  if (data < 0x10) print_message_serial(MESSAGE_ZERO); 
  Serial.print(data, HEX);
}

// print predefined message from PROGMEM to LCD
void print_message_lcd(const char *message) {
  // read back a char
  for (byte i = 0; i < strlen_P(message); i++) {
    char character = pgm_read_byte_near(message + i);
    lcd.print(character);
  }
}

// print predefined message from PROGMEM to serial port
void print_message_serial(const char *message) {
  // read back a char
  for (byte i = 0; i < strlen_P(message); i++) {
    char character = pgm_read_byte_near(message + i);
    Serial.print(character);
  }
}


/****************************************************************\
 ================================================================

                               CPU

 ================================================================
\****************************************************************/

// clear CPU registers
void reset_cpu() {
  register_A = 0;
  register_B = 0;
  program_counter = 0x0400; // points to ROM start
  stack_pointer = 0x3ff;
  zero_flag = 0;
}

// execute instruction
void execute() {
  while (true) { //Serial.print(program_counter, HEX);
    // read next opcode
    uint8_t opcode = read_byte(program_counter);
    
    
      
      /*Serial.print(": 0x");
      Serial.print(opcode, HEX);
      Serial.print("\n");
      getch();*/
    
    // execute instruction
    switch (opcode) {
      case NOP: program_counter = 0x0400; return;
      case LDI: zero_flag = ((register_A = read_byte(program_counter)) == 0); break;
      
      
        
      case LDA:
        //Serial.print("Word before:\n"); Serial.print(program_counter, HEX);
        
        zero_flag = ((register_A = read_byte((read_word() + register_B))) == 0);
        program_counter--;
        //Serial.print("Word after:\n"); Serial.print(program_counter, HEX);
        
        break;
      
      
      case TAB: zero_flag = ((register_B = register_A) == 0); break;
      case ADD: zero_flag = ((register_A += read_byte(program_counter)) == 0); break;
      case SUB: zero_flag = ((register_A -= read_byte(program_counter)) == 0); break;
      case STA: RAM[read_word() + register_B] = register_A; break;
      case LPC: program_counter = read_word(); break;
      case INC: zero_flag = (++register_B == 0); break;
      case DCR: zero_flag = (--register_B == 0); break;
      case CMP: zero_flag = ((register_A - read_byte(program_counter)) == 0); break;
      case JMP: if (zero_flag) program_counter = read_word(); else read_word(); break;
      case RCH: zero_flag = (register_A = keypad.getKey()) == 0; break;
      case IN: while ((register_A = keypad.getKey()) == NO_KEY); break;
      case OUT: lcd.print(char(register_A)); break;
      case SER: Serial.print(char(register_A)); break;
      case BIT: zero_flag = ((register_A & read_byte(program_counter)) == 0); break;
      case AND: zero_flag = ((register_A &= read_byte(program_counter)) == 0); break;
      case OR: zero_flag = ((register_A |= read_byte(program_counter)) == 0); break;
      case XOR: zero_flag = ((register_A ^= read_byte(program_counter)) == 0); break;
      case NOT: zero_flag = ((register_A = ~read_byte(program_counter)) == 0); break;
      case SHL: zero_flag = ((register_A <<= read_byte(program_counter)) == 0); break;
      case SHR: zero_flag = ((register_A >>= read_byte(program_counter)) == 0); break;
      case CLS: lcd.clear(); break;
      case SDL: lcd.scrollDisplayLeft(); break;
      case SDR: lcd.scrollDisplayRight(); break;
      case CRS: lcd.blink(); break;
      case NCR: lcd.noBlink(); break;
      case POS: lcd.setCursor(register_A, register_B); break;
      case DLY: delay(read_byte(program_counter)); break;
      case RND: zero_flag = (register_A = random(read_byte(program_counter))); break;
      case NUM:
        //Serial.print(read_word(), HEX);
        //Serial.print(read_byte(read_word()), HEX);
        //lcd.print(read_byte(read_word()));
        lcd.print(RAM[read_word()]); break;

        //Serial.print(program_counter, HEX);
        break; // TODO: fix address
      case INM: zero_flag = (++RAM[read_word()] == 0); break;
      case DCM: zero_flag = (--RAM[read_word()] == 0); break;
      case PSH:
        RAM[stack_pointer--] = register_A;
        RAM[stack_pointer--] = register_B;
        break;
      case POP:
        register_B = RAM[++stack_pointer];
        register_A = RAM[++stack_pointer];
        break;
      case SBR:
        RAM[stack_pointer--] = (uint8_t)(program_counter & 0x00ff) + 2;
        RAM[stack_pointer--] = (uint8_t)(program_counter >> 4);
        program_counter = read_word();
        break;
      case RET:
        program_counter = 0;
        program_counter <<= RAM[++stack_pointer];
        program_counter |= RAM[++stack_pointer];
        break;
      case UDG: {
          lcd.createChar(register_A, RAM + register_B);
          lcd.begin(16, 2);
          break;
        }
      case SPR: lcd.write(byte(read_byte(program_counter))); break;
      case DBG:
        lcd.setCursor(0, 0);
        lcd.print("A  B  PC SP ZF");
        lcd.setCursor(0, 1);
        print_byte(register_A);
        print_byte(register_B);
        print_byte(program_counter);
        print_byte(stack_pointer);
        print_byte(zero_flag);
        break;
      default:
        lcd.clear();
        print_message_lcd(MESSAGE_UNKNOWN_OPCODE);
        lcd.setCursor(0, 1);
        print_byte(opcode);
        print_message_lcd(MESSAGE_QUESTION_MARK);
        return;
    }
  }
}


/****************************************************************\
 ================================================================

                               MAIN

 ================================================================
\****************************************************************/

// get user keypress
char getch() {
  char key;
  while ((key = keypad.getKey()) == NO_KEY) {
    // use LCD Shield buttons as commands shortcuts
    #ifdef CMK_HARDWARE
      int shield_input;
      shield_input = analogRead (0);
      if (shield_input < 60) { delay(300); command_save(); }          // button right
      else if (shield_input < 200) { delay(300); command_view(); }    // button up
      else if (shield_input < 400) { delay(300); command_clear(); }   // button down
      else if (shield_input < 600) { delay(300); command_load(); }    // button left
      else if (shield_input < 800) { delay(300); command_run(); }     // button select
    #endif
  }

  return key;
}

// convert ASCII character to HEX number
uint8_t ascii_to_hex(char ascii) {
  return ascii <= '9' ? (ascii - '0') : (ascii - 'A' + 10);
}

// reset computer
void init_computer() {
  lcd.clear();
  //print_message_lcd(MESSAGE_CMK);
  lcd.setCursor(0, 0);
  reset_cpu();
  reset_memory();
}

// run program
void command_run() {
  lcd.clear();
  //print_message_lcd(MESSAGE_RUN);
  lcd.setCursor(3, 2);
  delay(300);
  lcd.clear();
  execute();
}

// view memory dump
void command_view() {
  lcd.clear();
  print_message_lcd(MESSAGE_VIEW);
  memory_dump(encode_word());
}

// load program
void command_load() {
  lcd.clear();
  print_message_lcd(MESSAGE_LOAD);
  delay(300);
  lcd.clear();
  print_message_lcd(MESSAGE_WAITING);
  lcd.setCursor(0, 1);
  print_message_lcd(MESSAGE_INCOMING);
  while (Serial.available() == 0);
  lcd.clear();
  print_message_lcd(MESSAGE_LOADING);
  Serial.readBytes(RAM, RAM_SIZE);
  
  // ascii to bytes       
  for (int i = 0; i < RAM_SIZE; i++) {
    RAM[i] = ascii_to_hex(RAM[i]);
    if ((i % 2) == 0) RAM[i] <<= 4;
    else {
      RAM[i - 1] |= RAM[i];
      RAM[i] = 0;
    }
  }
  
  // group bytes
  for (int i = 0; i < RAM_SIZE; i++) {
    if ((i % 2) == 0) {
      if (i) {
        RAM[i - ((int)(i / 2))] = RAM[i];
        RAM[i] = 0;
      }
    }
  }

  // verify transfered bytes
  //Serial.println("Your program bytes loaded:");
  //for (int i = 0; i < RAM_SIZE; i++) print_hex(RAM[i]);

  print_message_lcd(MESSAGE_DONE);
  lcd.setCursor(0, 1);
}

// save program
void command_save() {
  lcd.clear();
  print_message_lcd(MESSAGE_SAVE);
  delay(300);
  lcd.clear();
  print_message_lcd(MESSAGE_SAVING);
  
  for (int i = 0; i < RAM_SIZE; i++) {
    if ( RAM[i] < 0x10) print_message_serial(MESSAGE_ZERO);
    Serial.print(RAM[i], HEX);
  }
  
  print_message_lcd(MESSAGE_DONE_LONG);
  lcd.setCursor(0, 1);
}

// clear screen
void command_clear() {
  lcd.clear();
  print_message_lcd(MESSAGE_CLEAR);
  delay(300);
  lcd.clear();
}

// software reset
void command_new() {
  lcd.clear();
  print_message_lcd(MESSAGE_NEW);
  lcd.setCursor(3, 2);
  delay(300);
  init_computer();
}

// arduino setup
void setup() {
  // init serial port for debugging
  Serial.begin(9600);
  
  // init LCD
  lcd.begin(16, 2);
  lcd.noAutoscroll();
  lcd.blink();
  
  // reset computer  
  init_computer();
  
RAM[0] =  0x08 ;
RAM[1] =  0x00 ;
RAM[2] =  0x72 ;
RAM[3] =  0x00 ;
RAM[4] =  0x20 ;
RAM[5] =  0x41 ;
RAM[6] =  0x52 ;
RAM[7] =  0x45 ;
RAM[8] =  0x20 ;
RAM[9] =  0x59 ;
RAM[10] =  0x4f ;
RAM[11] =  0x55 ;
RAM[12] =  0x20 ;
RAM[13] =  0x52 ;
RAM[14] =  0x45 ;
RAM[15] =  0x41 ;
RAM[16] =  0x44 ;
RAM[17] =  0x59 ;
RAM[18] =  0x3f ;
RAM[19] =  0x20 ;
RAM[20] =  0xfe ;
RAM[21] =  0x47 ;
RAM[22] =  0x41 ;
RAM[23] =  0x4d ;
RAM[24] =  0x45 ;
RAM[25] =  0x20 ;
RAM[26] =  0x4f ;
RAM[27] =  0x56 ;
RAM[28] =  0x45 ;
RAM[29] =  0x52 ;
RAM[30] =  0x21 ;
RAM[31] =  0xfe ;
RAM[32] =  0x20 ;
RAM[33] =  0x00 ;
RAM[34] =  0x02 ;
RAM[35] =  0x20 ;
RAM[36] =  0x59 ;
RAM[37] =  0x4f ;
RAM[38] =  0x55 ;
RAM[39] =  0x20 ;
RAM[40] =  0x57 ;
RAM[41] =  0x49 ;
RAM[42] =  0x4e ;
RAM[43] =  0x21 ;
RAM[44] =  0x20 ;
RAM[45] =  0x02 ;
RAM[46] =  0x00 ;
RAM[47] =  0x20 ;
RAM[48] =  0xfe ;
RAM[49] =  0x20 ;
RAM[50] =  0x20 ;
RAM[51] =  0x20 ;
RAM[52] =  0x20 ;
RAM[53] =  0x20 ;
RAM[54] =  0x20 ;
RAM[55] =  0x20 ;
RAM[56] =  0x53 ;
RAM[57] =  0x63 ;
RAM[58] =  0x6f ;
RAM[59] =  0x72 ;
RAM[60] =  0x65 ;
RAM[61] =  0x3a ;
RAM[62] =  0x20 ;
RAM[63] =  0x20 ;
RAM[64] =  0x20 ;
RAM[65] =  0x20 ;
RAM[66] =  0x00 ;
RAM[67] =  0x20 ;
RAM[68] =  0x20 ;
RAM[69] =  0x20 ;
RAM[70] =  0x20 ;
RAM[71] =  0x20 ;
RAM[72] =  0x20 ;
RAM[73] =  0x20 ;
RAM[74] =  0x20 ;
RAM[75] =  0x20 ;
RAM[76] =  0x20 ;
RAM[77] =  0x20 ;
RAM[78] =  0x20 ;
RAM[79] =  0x20 ;
RAM[80] =  0x20 ;
RAM[81] =  0xfe ;
RAM[82] =  0x07 ;
RAM[83] =  0x05 ;
RAM[84] =  0x07 ;
RAM[85] =  0x16 ;
RAM[86] =  0x1f ;
RAM[87] =  0x1e ;
RAM[88] =  0x0e ;
RAM[89] =  0x04 ;
RAM[90] =  0x07 ;
RAM[91] =  0x05 ;
RAM[92] =  0x07 ;
RAM[93] =  0x16 ;
RAM[94] =  0x1f ;
RAM[95] =  0x1e ;
RAM[96] =  0x0e ;
RAM[97] =  0x02 ;
RAM[98] =  0x00 ;
RAM[99] =  0x04 ;
RAM[100] =  0x05 ;
RAM[101] =  0x15 ;
RAM[102] =  0x16 ;
RAM[103] =  0x0c ;
RAM[104] =  0x04 ;
RAM[105] =  0x04 ;
RAM[106] =  0x00 ;
RAM[107] =  0x00 ;
RAM[108] =  0x04 ;
RAM[109] =  0x05 ;
RAM[110] =  0x15 ;
RAM[111] =  0x16 ;
RAM[112] =  0x0c ;
RAM[113] =  0x04 ;
RAM[114] =  0x01 ;
RAM[115] =  0x52 ;
RAM[116] =  0x03 ;
RAM[117] =  0x01 ;
RAM[118] =  0x00 ;
RAM[119] =  0x1c ;
RAM[120] =  0x01 ;
RAM[121] =  0x5a ;
RAM[122] =  0x03 ;
RAM[123] =  0x01 ;
RAM[124] =  0x01 ;
RAM[125] =  0x1c ;
RAM[126] =  0x01 ;
RAM[127] =  0x62 ;
RAM[128] =  0x03 ;
RAM[129] =  0x01 ;
RAM[130] =  0x02 ;
RAM[131] =  0x1c ;
RAM[132] =  0x01 ;
RAM[133] =  0x6a ;
RAM[134] =  0x03 ;
RAM[135] =  0x01 ;
RAM[136] =  0x04 ;
RAM[137] =  0x1c ;
RAM[138] =  0x01 ;
RAM[139] =  0x00 ;
RAM[140] =  0x03 ;
RAM[141] =  0x17 ;
RAM[142] =  0x02 ;
RAM[143] =  0x00 ;
RAM[144] =  0x04 ;
RAM[145] =  0x0b ;
RAM[146] =  0xfe ;
RAM[147] =  0x0c ;
RAM[148] =  0x00 ;
RAM[149] =  0x9d ;
RAM[150] =  0x1f ;
RAM[151] =  0x50 ;
RAM[152] =  0x0f ;
RAM[153] =  0x09 ;
RAM[154] =  0x08 ;
RAM[155] =  0x00 ;
RAM[156] =  0x8e ;
RAM[157] =  0x07 ;
RAM[158] =  0x0b ;
RAM[159] =  0x30 ;
RAM[160] =  0x0c ;
RAM[161] =  0x00 ;
RAM[162] =  0xa6 ;
RAM[163] =  0x08 ;
RAM[164] =  0x00 ;
RAM[165] =  0x9d ;
RAM[166] =  0x17 ;
RAM[167] =  0x07 ;
RAM[168] =  0x0b ;
RAM[169] =  0x30 ;
RAM[170] =  0x0c ;
RAM[171] =  0x00 ;
RAM[172] =  0xb3 ;
RAM[173] =  0x23 ;
RAM[174] =  0x00 ;
RAM[175] =  0xed ;
RAM[176] =  0x08 ;
RAM[177] =  0x00 ;
RAM[178] =  0xa7 ;
RAM[179] =  0x01 ;
RAM[180] =  0x00 ;
RAM[181] =  0x03 ;
RAM[182] =  0x01 ;
RAM[183] =  0x00 ;
RAM[184] =  0x1e ;
RAM[185] =  0x01 ;
RAM[186] =  0x01 ;
RAM[187] =  0x03 ;
RAM[188] =  0x02 ;
RAM[189] =  0x00 ;
RAM[190] =  0x41 ;
RAM[191] =  0x06 ;
RAM[192] =  0x00 ;
RAM[193] =  0x31 ;
RAM[194] =  0x01 ;
RAM[195] =  0x20 ;
RAM[196] =  0x06 ;
RAM[197] =  0x00 ;
RAM[198] =  0x41 ;
RAM[199] =  0x23 ;
RAM[200] =  0x00 ;
RAM[201] =  0xed ;
RAM[202] =  0x23 ;
RAM[203] =  0x00 ;
RAM[204] =  0xed ;
RAM[205] =  0x23 ;
RAM[206] =  0x00 ;
RAM[207] =  0xed ;
RAM[208] =  0x01 ;
RAM[209] =  0x00 ;
RAM[210] =  0x03 ;
RAM[211] =  0x01 ;
RAM[212] =  0x00 ;
RAM[213] =  0x1e ;
RAM[214] =  0x01 ;
RAM[215] =  0x01 ;
RAM[216] =  0x03 ;
RAM[217] =  0x02 ;
RAM[218] =  0x00 ;
RAM[219] =  0x31 ;
RAM[220] =  0x06 ;
RAM[221] =  0x00 ;
RAM[222] =  0x41 ;
RAM[223] =  0x01 ;
RAM[224] =  0x20 ;
RAM[225] =  0x06 ;
RAM[226] =  0x00 ;
RAM[227] =  0x31 ;
RAM[228] =  0x01 ;
RAM[229] =  0x20 ;
RAM[230] =  0x0a ;
RAM[231] =  0x06 ;
RAM[232] =  0x00 ;
RAM[233] =  0x41 ;
RAM[234] =  0x08 ;
RAM[235] =  0x00 ;
RAM[236] =  0xa7 ;
RAM[237] =  0x1f ;
RAM[238] =  0xff ;
RAM[239] =  0x23 ;
RAM[240] =  0x01 ;
RAM[241] =  0x3e ;
RAM[242] =  0x01 ;
RAM[243] =  0x00 ;
RAM[244] =  0x03 ;
RAM[245] =  0x0b ;
RAM[246] =  0x3a ;
RAM[247] =  0x0c ;
RAM[248] =  0x01 ;
RAM[249] =  0x17 ;
RAM[250] =  0x02 ;
RAM[251] =  0x00 ;
RAM[252] =  0x31 ;
RAM[253] =  0x0c ;
RAM[254] =  0x01 ;
RAM[255] =  0x0f ;
RAM[256] =  0x0b ;
RAM[257] =  0x01 ;
RAM[258] =  0x0c ;
RAM[259] =  0x01 ;
RAM[260] =  0x0f ;
RAM[261] =  0x0b ;
RAM[262] =  0xfe ;
RAM[263] =  0x0c ;
RAM[264] =  0x01 ;
RAM[265] =  0x25 ;
RAM[266] =  0x0f ;
RAM[267] =  0x09 ;
RAM[268] =  0x08 ;
RAM[269] =  0x00 ;
RAM[270] =  0xf5 ;
RAM[271] =  0x13 ;
RAM[272] =  0x01 ;
RAM[273] =  0x06 ;
RAM[274] =  0x00 ;
RAM[275] =  0x31 ;
RAM[276] =  0x08 ;
RAM[277] =  0x01 ;
RAM[278] =  0x05 ;
RAM[279] =  0x21 ;
RAM[280] =  0x01 ;
RAM[281] =  0x01 ;
RAM[282] =  0x03 ;
RAM[283] =  0x01 ;
RAM[284] =  0x00 ;
RAM[285] =  0x1e ;
RAM[286] =  0x22 ;
RAM[287] =  0x09 ;
RAM[288] =  0x09 ;
RAM[289] =  0x09 ;
RAM[290] =  0x08 ;
RAM[291] =  0x00 ;
RAM[292] =  0xfa ;
RAM[293] =  0x26 ;
RAM[294] =  0x00 ;
RAM[295] =  0x03 ;
RAM[296] =  0x21 ;
RAM[297] =  0x01 ;
RAM[298] =  0x00 ;
RAM[299] =  0x03 ;
RAM[300] =  0x01 ;
RAM[301] =  0x0d ;
RAM[302] =  0x1e ;
RAM[303] =  0x25 ;
RAM[304] =  0x00 ;
RAM[305] =  0x03 ;
RAM[306] =  0x01 ;
RAM[307] =  0x20 ;
RAM[308] =  0x0f ;
RAM[309] =  0x0f ;
RAM[310] =  0x02 ;
RAM[311] =  0x00 ;
RAM[312] =  0x03 ;
RAM[313] =  0x0c ;
RAM[314] =  0x01 ;
RAM[315] =  0xcb ;
RAM[316] =  0x22 ;
RAM[317] =  0x24 ;
RAM[318] =  0x01 ;
RAM[319] =  0x00 ;
RAM[320] =  0x03 ;
RAM[321] =  0x20 ;
RAM[322] =  0x20 ;
RAM[323] =  0x0b ;
RAM[324] =  0x02 ;
RAM[325] =  0x0c ;
RAM[326] =  0x01 ;
RAM[327] =  0x4f ;
RAM[328] =  0x0b ;
RAM[329] =  0x04 ;
RAM[330] =  0x0c ;
RAM[331] =  0x01 ;
RAM[332] =  0x4f ;
RAM[333] =  0x01 ;
RAM[334] =  0x20 ;
RAM[335] =  0x06 ;
RAM[336] =  0x00 ;
RAM[337] =  0x50 ;
RAM[338] =  0x01 ;
RAM[339] =  0x00 ;
RAM[340] =  0x03 ;
RAM[341] =  0x09 ;
RAM[342] =  0x02 ;
RAM[343] =  0x00 ;
RAM[344] =  0x41 ;
RAM[345] =  0x0b ;
RAM[346] =  0x02 ;
RAM[347] =  0x0c ;
RAM[348] =  0x01 ;
RAM[349] =  0x6b ;
RAM[350] =  0x0b ;
RAM[351] =  0x04 ;
RAM[352] =  0x0c ;
RAM[353] =  0x01 ;
RAM[354] =  0x6b ;
RAM[355] =  0x0b ;
RAM[356] =  0xfe ;
RAM[357] =  0x0c ;
RAM[358] =  0x01 ;
RAM[359] =  0x8c ;
RAM[360] =  0x08 ;
RAM[361] =  0x01 ;
RAM[362] =  0x55 ;
RAM[363] =  0x21 ;
RAM[364] =  0x0a ;
RAM[365] =  0x21 ;
RAM[366] =  0x02 ;
RAM[367] =  0x00 ;
RAM[368] =  0x41 ;
RAM[369] =  0x0b ;
RAM[370] =  0x00 ;
RAM[371] =  0x0c ;
RAM[372] =  0x01 ;
RAM[373] =  0x8d ;
RAM[374] =  0x0b ;
RAM[375] =  0x01 ;
RAM[376] =  0x0c ;
RAM[377] =  0x01 ;
RAM[378] =  0x8d ;
RAM[379] =  0x22 ;
RAM[380] =  0x06 ;
RAM[381] =  0x00 ;
RAM[382] =  0x41 ;
RAM[383] =  0x09 ;
RAM[384] =  0x09 ;
RAM[385] =  0x02 ;
RAM[386] =  0x00 ;
RAM[387] =  0x41 ;
RAM[388] =  0x0a ;
RAM[389] =  0x06 ;
RAM[390] =  0x00 ;
RAM[391] =  0x41 ;
RAM[392] =  0x22 ;
RAM[393] =  0x08 ;
RAM[394] =  0x01 ;
RAM[395] =  0x63 ;
RAM[396] =  0x24 ;
RAM[397] =  0x22 ;
RAM[398] =  0x22 ;
RAM[399] =  0x22 ;
RAM[400] =  0x22 ;
RAM[401] =  0x01 ;
RAM[402] =  0x02 ;
RAM[403] =  0x03 ;
RAM[404] =  0x01 ;
RAM[405] =  0x20 ;
RAM[406] =  0x21 ;
RAM[407] =  0x02 ;
RAM[408] =  0x00 ;
RAM[409] =  0x41 ;
RAM[410] =  0x0b ;
RAM[411] =  0xfe ;
RAM[412] =  0x0c ;
RAM[413] =  0x01 ;
RAM[414] =  0xa7 ;
RAM[415] =  0x22 ;
RAM[416] =  0x06 ;
RAM[417] =  0x00 ;
RAM[418] =  0x41 ;
RAM[419] =  0x09 ;
RAM[420] =  0x08 ;
RAM[421] =  0x01 ;
RAM[422] =  0x96 ;
RAM[423] =  0x22 ;
RAM[424] =  0x01 ;
RAM[425] =  0x01 ;
RAM[426] =  0x03 ;
RAM[427] =  0x01 ;
RAM[428] =  0x03 ;
RAM[429] =  0x1e ;
RAM[430] =  0x27 ;
RAM[431] =  0x00 ;
RAM[432] =  0x03 ;
RAM[433] =  0x0c ;
RAM[434] =  0x01 ;
RAM[435] =  0xb7 ;
RAM[436] =  0x08 ;
RAM[437] =  0x01 ;
RAM[438] =  0xae ;
RAM[439] =  0x01 ;
RAM[440] =  0x00 ;
RAM[441] =  0x03 ;
RAM[442] =  0x02 ;
RAM[443] =  0x00 ;
RAM[444] =  0x15 ;
RAM[445] =  0x0b ;
RAM[446] =  0xfe ;
RAM[447] =  0x0c ;
RAM[448] =  0x01 ;
RAM[449] =  0xc9 ;
RAM[450] =  0x1f ;
RAM[451] =  0x50 ;
RAM[452] =  0x0f ;
RAM[453] =  0x09 ;
RAM[454] =  0x08 ;
RAM[455] =  0x01 ;
RAM[456] =  0xba ;
RAM[457] =  0x1a ;
RAM[458] =  0x00 ;
RAM[459] =  0x22 ;
RAM[460] =  0x22 ;
RAM[461] =  0x01 ;
RAM[462] =  0x01 ;
RAM[463] =  0x03 ;
RAM[464] =  0x01 ;
RAM[465] =  0x00 ;
RAM[466] =  0x1e ;
RAM[467] =  0x01 ;
RAM[468] =  0x00 ;
RAM[469] =  0x03 ;
RAM[470] =  0x02 ;
RAM[471] =  0x00 ;
RAM[472] =  0x20 ;
RAM[473] =  0x0b ;
RAM[474] =  0xfe ;
RAM[475] =  0x0c ;
RAM[476] =  0x01 ;
RAM[477] =  0xe5 ;
RAM[478] =  0x1f ;
RAM[479] =  0x50 ;
RAM[480] =  0x0f ;
RAM[481] =  0x09 ;
RAM[482] =  0x08 ;
RAM[483] =  0x01 ;
RAM[484] =  0xd6 ;
RAM[485] =  0x1f ;
RAM[486] =  0xff ;
RAM[487] =  0x1f ;
RAM[488] =  0xff ;
RAM[489] =  0x1f ;
RAM[490] =  0xff ;
RAM[491] =  0x1f ;
RAM[492] =  0xff ;
RAM[493] =  0x08 ;
RAM[494] =  0x01 ;
RAM[495] =  0x91 ;
}

// arduino loop
void loop() {
  execute();
  
  /*while (true) {
    // get user command/address
    uint16_t addr = encode_word();
    lcd.print(':');

    // handle user commands
    switch (addr) {
      case CLEAR: command_clear(); break;      // clear LCD
      case NEW: command_new(); break;          // software reset
      case VIEW: command_view(); break;        // view memory dump
      case LOAD: command_load(); break;        // load program via serial port
      case SAVE: command_save(); break;        // save program via serial port
      case RUN: command_run(); break;
      
      // enter bytes to memory
      default:
        for (int i = addr; i < addr + 4; i++) {
          RAM[i] = encode_byte();
          lcd.print(' ');
        }
        
        delay(300);
        memory_dump(addr);
        break;
    }
  }*/
}
 