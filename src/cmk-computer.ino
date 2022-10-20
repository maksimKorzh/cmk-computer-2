/****************************************************************\
 ================================================================

                           8-bit computer
                      
                                 by
                     
                          Code Monkey King

 ================================================================
\****************************************************************/

// libraries
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
const char MESSAGE_CAPTIONS[] PROGMEM = {"ADDRESS DATA ASM"};
const char MESSAGE_ZERO[] PROGMEM = {"0"};

/*const char MESSAGE_REGISTER_A_DEBUG[] PROGMEM = {"Register A: 0x"};
const char MESSAGE_REGISTER_B_DEBUG[] PROGMEM = {"Register B: 0x"};
const char MESSAGE_REGISTER_PC_DEBUG[] PROGMEM = {"Program Counter: 0x"};
const char MESSAGE_REGISTER_SP_DEBUG[] PROGMEM = {"Stack pointer: 0x"};
const char MESSAGE_REGISTER_ZF_DEBUG[] PROGMEM = {"Zero Flag: 0x"};*/

const char MESSAGE_DEBUG[] PROGMEM = {" A  B  PC SP ZF"};
const char MESSAGE_UNKNOWN_OPCODE[] PROGMEM = {"Unknown opcode:"};
const char MESSAGE_QUESTION_MARK[] PROGMEM = {"? "};
const char MESSAGE_INTRO_1[] PROGMEM = {" 8-bit computer "};
const char MESSAGE_INTRO_2[] PROGMEM = {"Code Monkey King"};
const char MESSAGE_NEW[] PROGMEM = {"NEW  "};
const char MESSAGE_LOAD[] PROGMEM = {"LOAD FROM SERIAL"};
const char MESSAGE_SAVE[] PROGMEM = {" SAVE TO SERIAL"};
const char MESSAGE_CLEAR[] PROGMEM = {"CLEAR"};
const char MESSAGE_RUN[] PROGMEM = {" RUNS AT 0x"};
const char MESSAGE_VIEW[] PROGMEM = {"VIEW: "};
const char MESSAGE_WAITING[] PROGMEM = {" Waiting for"};
const char MESSAGE_INCOMING[] PROGMEM = {"incoming data..."};
const char MESSAGE_LOADING[] PROGMEM = {"Loading..."};
const char MESSAGE_DONE[] PROGMEM = {"  done"};
const char MESSAGE_DONE_LONG[] PROGMEM = {"   done"};
const char MESSAGE_SAVING[] PROGMEM = {"Saving..."};

// opcodes
const char *OPCODES[] = {
  "NOP", "LDI", "LDA", "TAB", "ADD", "SUB", "STA", "RCH",
  "LPC", "INC", "DCR", "CMP", "JMP", "DBG", "IN ", "OUT",
  "BIT", "AND", "OR ", "XOR", "NOT", "SHL", "SHR", "CLS",
  "SDL", "SDR", "CRS", "NCR", "UDG", "SPR", "POS", "DLY",
  "RND", "PSH", "POP", "SBR", "RET", "NUM", "INM", "DCM", "SER"
};

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

// define RAM size
#define MEMORY_SIZE 1024

// RAM array
uint8_t  memory[MEMORY_SIZE];

// CPU registers
uint8_t  register_A = 0;
uint8_t  register_B = 0;
uint16_t  program_counter = 0;
uint16_t stack_pointer = 0x3ff;
bool zero_flag = 0;

// OS variables
uint16_t current_addr = 0;
bool mode = 0; // ADDR = 0; DATA = 1;
bool command_keys = 1; // use keypad for commands/digits

/****************************************************************\
 ================================================================

                              MEMORY

 ================================================================
\****************************************************************/

// clear RAM
void reset_memory() {
  for (uint16_t i = 0; i < MEMORY_SIZE; i++)
    memory[i] = 0;
}

// read byte from memory
uint8_t read_byte() {
  uint8_t value = memory[program_counter];
  program_counter++;
  return value;
}

// read word from memory
uint16_t read_word() {
  uint8_t MSB = read_byte();
  uint8_t LSB = read_byte();
  uint16_t value = MSB;
  value <<= 8;
  value |= LSB;
  return value;
}

// encode address
uint16_t encode_word() {
  lcd.setCursor(4, 1);
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
  for (uint16_t i = addr; i < addr + 4; i++) print_byte(memory[i]);
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
  program_counter = 0;
  stack_pointer = 0x3ff;
  zero_flag = 0;
}

// execute instruction
bool execute() { //Serial.print(program_counter, HEX); Serial.print("\n");
  // read next opcode
  uint8_t opcode = read_byte();    
  
  // execute instruction
  switch (opcode) {
    case NOP: return 0;
    case LDI: zero_flag = ((register_A = read_byte()) == 0); break;
    case LDA: zero_flag = ((register_A = memory[(read_word() + register_B)]) == 0); break;
    case TAB: zero_flag = ((register_B = register_A) == 0); break;
    case ADD: zero_flag = ((register_A += read_byte()) == 0); break;
    case SUB: zero_flag = ((register_A -= read_byte()) == 0); break;
    case STA: memory[read_word() + register_B] = register_A; break;
    case LPC: program_counter = read_word(); break;
    case INC: zero_flag = (++register_B == 0); break;
    case DCR: zero_flag = (--register_B == 0); break;
    case CMP: zero_flag = ((register_A - read_byte()) == 0); break;
    case JMP: if (zero_flag) program_counter = read_word(); else read_word(); break;
    case RCH:
      zero_flag = (register_A = keypad.getKey()) == 0;
      if (register_A == 'F') return 0;
      break;
    case IN: while ((register_A = keypad.getKey()) == NO_KEY); break;
    case OUT: lcd.print(char(register_A)); break;
    case SER: Serial.print(register_A); break;//Serial.print(char(register_A)); break;
    case BIT: zero_flag = ((register_A & read_byte()) == 0); break;
    case AND: zero_flag = ((register_A &= read_byte()) == 0); break;
    case OR: zero_flag = ((register_A |= read_byte()) == 0); break;
    case XOR: zero_flag = ((register_A ^= read_byte()) == 0); break;
    case NOT: zero_flag = ((register_A = ~read_byte()) == 0); break;
    case SHL: zero_flag = ((register_A <<= read_byte()) == 0); break;
    case SHR: zero_flag = ((register_A >>= read_byte()) == 0); break;
    case CLS: lcd.clear(); break;
    case SDL: lcd.scrollDisplayLeft(); break;
    case SDR: lcd.scrollDisplayRight(); break;
    case CRS: lcd.blink(); break;
    case NCR: lcd.noBlink(); break;
    case POS: lcd.setCursor(register_A, register_B); break;
    case DLY: delay(read_byte()); break;
    case RND: zero_flag = (register_A = random(read_byte())); break;
    case NUM: lcd.print(memory[read_word()]); break;
    case INM: zero_flag = (++memory[read_word()] == 0); break;
    case DCM: zero_flag = (--memory[read_word()] == 0); break;
    case PSH:
      memory[stack_pointer--] = register_A;
      memory[stack_pointer--] = register_B;
      break;
    case POP:
      register_B = memory[++stack_pointer];
      register_A = memory[++stack_pointer];
      break;
    case SBR:
      memory[stack_pointer--] = (uint8_t)(program_counter & 0x00ff) + 2;
      memory[stack_pointer--] = (uint8_t)(program_counter >> 4);
      program_counter = read_word();
      break;
    case RET:
      program_counter = 0;
      program_counter <<= memory[++stack_pointer];
      program_counter |= memory[++stack_pointer];
      break;
    case UDG:
      lcd.createChar(register_A, memory + register_B);
      lcd.begin(16, 2);
      break;
    case SPR: lcd.write(byte(read_byte())); break;
    case DBG:
      print_debug();
      break;
    default:
      lcd.clear();
      print_message_lcd(MESSAGE_UNKNOWN_OPCODE);
      lcd.setCursor(0, 1);
      print_byte(opcode);
      print_message_lcd(MESSAGE_QUESTION_MARK);
      lcd.print(" at 0x");
      print_word(program_counter-1);
      delay(2000);
      return 0;
  } return 1;
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
      if (shield_input < 60) { delay(300); command_save(); }              // button right
      else if (shield_input < 200) { delay(300); command_view(); }        // button up
      else if (shield_input < 400) { delay(300); command_clear(); }       // button down
      else if (shield_input < 600) { delay(300); command_load(); }        // button left
      else if (shield_input < 800) { delay(300); command_keys ^= 1; }     // button select
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
  print_message_lcd(MESSAGE_INTRO_1);
  lcd.setCursor(0, 1);
  print_message_lcd(MESSAGE_INTRO_2);
  delay(2000);
  lcd.clear();
  print_message_lcd(MESSAGE_CAPTIONS);
  reset_cpu();
  reset_memory();
}

// run program
void command_run(char key) {
  lcd.clear();
  print_message_lcd(MESSAGE_RUN);
  print_word(program_counter);
  lcd.setCursor(3, 2);
  delay(1000);
  lcd.clear();
  //bool keypad_mode = command_keys;
  //command_keys = 0;
  if (key == 'F') while(execute());
  else execute();
  //command_keys = keypad_mode;
  current_addr = program_counter;
  lcd.clear();
  print_message_lcd(MESSAGE_CAPTIONS);
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
  delay(1000);
  lcd.clear();
  print_message_lcd(MESSAGE_WAITING);
  lcd.setCursor(0, 1);
  print_message_lcd(MESSAGE_INCOMING);
  while (Serial.available() == 0);
  lcd.clear();
  print_message_lcd(MESSAGE_LOADING);
  Serial.readBytes(memory, MEMORY_SIZE);
  print_message_lcd(MESSAGE_DONE);
  delay(1000);
  lcd.clear();
  print_message_lcd(MESSAGE_CAPTIONS);
}

// save program
void command_save() {
  lcd.clear();
  print_message_lcd(MESSAGE_SAVE);
  delay(1000);
  lcd.clear();
  print_message_lcd(MESSAGE_SAVING);
  
  for (int i = 0; i < MEMORY_SIZE; i++) {
    if ( memory[i] < 0x10) print_message_serial(MESSAGE_ZERO);
    Serial.print(memory[i], HEX);
  }
  
  print_message_lcd(MESSAGE_DONE_LONG);
  delay(1000);
  lcd.clear();
  print_message_lcd(MESSAGE_CAPTIONS);
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

// print debug
void print_debug() {
  lcd.clear();
  print_message_lcd(MESSAGE_DEBUG);
  lcd.setCursor(1, 1);
  print_byte(register_A);
  lcd.print(" ");
  print_byte(register_B);
  lcd.print(" ");
  print_byte(program_counter);
  lcd.print(" ");
  print_byte(stack_pointer);
  lcd.print(" ");
  print_byte(zero_flag);
  getch();
  lcd.clear();
  print_message_lcd(MESSAGE_CAPTIONS);
}

// arduino setup
void setup() {
  // init serial port for debugging
  Serial.begin(9600);
  
  // init LCD
  lcd.begin(16, 2);
  lcd.noAutoscroll();
  lcd.noBlink();
  
  // reset computer  
  init_computer();
  
memory[0] =  0x08;
memory[1] =  0x00;
memory[2] =  0x72;
memory[3] =  0x00;
memory[4] =  0x20;
memory[5] =  0x41;
memory[6] =  0x52;
memory[7] =  0x45;
memory[8] =  0x20;
memory[9] =  0x59;
memory[10] =  0x4f;
memory[11] =  0x55;
memory[12] =  0x20;
memory[13] =  0x52;
memory[14] =  0x45;
memory[15] =  0x41;
memory[16] =  0x44;
memory[17] =  0x59;
memory[18] =  0x3f;
memory[19] =  0x20;
memory[20] =  0xfe;
memory[21] =  0x47;
memory[22] =  0x41;
memory[23] =  0x4d;
memory[24] =  0x45;
memory[25] =  0x20;
memory[26] =  0x4f;
memory[27] =  0x56;
memory[28] =  0x45;
memory[29] =  0x52;
memory[30] =  0x21;
memory[31] =  0xfe;
memory[32] =  0x20;
memory[33] =  0x00;
memory[34] =  0x02;
memory[35] =  0x20;
memory[36] =  0x59;
memory[37] =  0x4f;
memory[38] =  0x55;
memory[39] =  0x20;
memory[40] =  0x57;
memory[41] =  0x49;
memory[42] =  0x4e;
memory[43] =  0x21;
memory[44] =  0x20;
memory[45] =  0x02;
memory[46] =  0x00;
memory[47] =  0x20;
memory[48] =  0xfe;
memory[49] =  0x20;
memory[50] =  0x20;
memory[51] =  0x20;
memory[52] =  0x20;
memory[53] =  0x20;
memory[54] =  0x20;
memory[55] =  0x20;
memory[56] =  0x53;
memory[57] =  0x63;
memory[58] =  0x6f;
memory[59] =  0x72;
memory[60] =  0x65;
memory[61] =  0x3a;
memory[62] =  0x20;
memory[63] =  0x20;
memory[64] =  0x20;
memory[65] =  0x20;
memory[66] =  0x00;
memory[67] =  0x20;
memory[68] =  0x20;
memory[69] =  0x20;
memory[70] =  0x20;
memory[71] =  0x20;
memory[72] =  0x20;
memory[73] =  0x20;
memory[74] =  0x20;
memory[75] =  0x20;
memory[76] =  0x20;
memory[77] =  0x20;
memory[78] =  0x20;
memory[79] =  0x20;
memory[80] =  0x20;
memory[81] =  0xfe;
memory[82] =  0x07;
memory[83] =  0x05;
memory[84] =  0x07;
memory[85] =  0x16;
memory[86] =  0x1f;
memory[87] =  0x1e;
memory[88] =  0x0e;
memory[89] =  0x04;
memory[90] =  0x07;
memory[91] =  0x05;
memory[92] =  0x07;
memory[93] =  0x16;
memory[94] =  0x1f;
memory[95] =  0x1e;
memory[96] =  0x0e;
memory[97] =  0x02;
memory[98] =  0x00;
memory[99] =  0x04;
memory[100] =  0x05;
memory[101] =  0x15;
memory[102] =  0x16;
memory[103] =  0x0c;
memory[104] =  0x04;
memory[105] =  0x04;
memory[106] =  0x00;
memory[107] =  0x00;
memory[108] =  0x04;
memory[109] =  0x05;
memory[110] =  0x15;
memory[111] =  0x16;
memory[112] =  0x0c;
memory[113] =  0x04;
memory[114] =  0x01;
memory[115] =  0x52;
memory[116] =  0x03;
memory[117] =  0x01;
memory[118] =  0x00;
memory[119] =  0x1c;
memory[120] =  0x01;
memory[121] =  0x5a;
memory[122] =  0x03;
memory[123] =  0x01;
memory[124] =  0x01;
memory[125] =  0x1c;
memory[126] =  0x01;
memory[127] =  0x62;
memory[128] =  0x03;
memory[129] =  0x01;
memory[130] =  0x02;
memory[131] =  0x1c;
memory[132] =  0x01;
memory[133] =  0x6a;
memory[134] =  0x03;
memory[135] =  0x01;
memory[136] =  0x04;
memory[137] =  0x1c;
memory[138] =  0x01;
memory[139] =  0x00;
memory[140] =  0x03;
memory[141] =  0x17;
memory[142] =  0x02;
memory[143] =  0x00;
memory[144] =  0x04;
memory[145] =  0x0b;
memory[146] =  0xfe;
memory[147] =  0x0c;
memory[148] =  0x00;
memory[149] =  0x9d;
memory[150] =  0x1f;
memory[151] =  0x50;
memory[152] =  0x0f;
memory[153] =  0x09;
memory[154] =  0x08;
memory[155] =  0x00;
memory[156] =  0x8e;
memory[157] =  0x07;
memory[158] =  0x0b;
memory[159] =  0x30;
memory[160] =  0x0c;
memory[161] =  0x00;
memory[162] =  0xa6;
memory[163] =  0x08;
memory[164] =  0x00;
memory[165] =  0x9d;
memory[166] =  0x17;
memory[167] =  0x07;
memory[168] =  0x0b;
memory[169] =  0x30;
memory[170] =  0x0c;
memory[171] =  0x00;
memory[172] =  0xb3;
memory[173] =  0x23;
memory[174] =  0x00;
memory[175] =  0xed;
memory[176] =  0x08;
memory[177] =  0x00;
memory[178] =  0xa7;
memory[179] =  0x01;
memory[180] =  0x00;
memory[181] =  0x03;
memory[182] =  0x01;
memory[183] =  0x00;
memory[184] =  0x1e;
memory[185] =  0x01;
memory[186] =  0x01;
memory[187] =  0x03;
memory[188] =  0x02;
memory[189] =  0x00;
memory[190] =  0x41;
memory[191] =  0x06;
memory[192] =  0x00;
memory[193] =  0x31;
memory[194] =  0x01;
memory[195] =  0x20;
memory[196] =  0x06;
memory[197] =  0x00;
memory[198] =  0x41;
memory[199] =  0x23;
memory[200] =  0x00;
memory[201] =  0xed;
memory[202] =  0x23;
memory[203] =  0x00;
memory[204] =  0xed;
memory[205] =  0x23;
memory[206] =  0x00;
memory[207] =  0xed;
memory[208] =  0x01;
memory[209] =  0x00;
memory[210] =  0x03;
memory[211] =  0x01;
memory[212] =  0x00;
memory[213] =  0x1e;
memory[214] =  0x01;
memory[215] =  0x01;
memory[216] =  0x03;
memory[217] =  0x02;
memory[218] =  0x00;
memory[219] =  0x31;
memory[220] =  0x06;
memory[221] =  0x00;
memory[222] =  0x41;
memory[223] =  0x01;
memory[224] =  0x20;
memory[225] =  0x06;
memory[226] =  0x00;
memory[227] =  0x31;
memory[228] =  0x01;
memory[229] =  0x20;
memory[230] =  0x0a;
memory[231] =  0x06;
memory[232] =  0x00;
memory[233] =  0x41;
memory[234] =  0x08;
memory[235] =  0x00;
memory[236] =  0xa7;
memory[237] =  0x1f;
memory[238] =  0xff;
memory[239] =  0x23;
memory[240] =  0x01;
memory[241] =  0x3e;
memory[242] =  0x01;
memory[243] =  0x00;
memory[244] =  0x03;
memory[245] =  0x0b;
memory[246] =  0x3a;
memory[247] =  0x0c;
memory[248] =  0x01;
memory[249] =  0x17;
memory[250] =  0x02;
memory[251] =  0x00;
memory[252] =  0x31;
memory[253] =  0x0c;
memory[254] =  0x01;
memory[255] =  0x0f;
memory[256] =  0x0b;
memory[257] =  0x01;
memory[258] =  0x0c;
memory[259] =  0x01;
memory[260] =  0x0f;
memory[261] =  0x0b;
memory[262] =  0xfe;
memory[263] =  0x0c;
memory[264] =  0x01;
memory[265] =  0x25;
memory[266] =  0x0f;
memory[267] =  0x09;
memory[268] =  0x08;
memory[269] =  0x00;
memory[270] =  0xf5;
memory[271] =  0x13;
memory[272] =  0x01;
memory[273] =  0x06;
memory[274] =  0x00;
memory[275] =  0x31;
memory[276] =  0x08;
memory[277] =  0x01;
memory[278] =  0x05;
memory[279] =  0x21;
memory[280] =  0x01;
memory[281] =  0x01;
memory[282] =  0x03;
memory[283] =  0x01;
memory[284] =  0x00;
memory[285] =  0x1e;
memory[286] =  0x22;
memory[287] =  0x09;
memory[288] =  0x09;
memory[289] =  0x09;
memory[290] =  0x08;
memory[291] =  0x00;
memory[292] =  0xfa;
memory[293] =  0x26;
memory[294] =  0x00;
memory[295] =  0x03;
memory[296] =  0x21;
memory[297] =  0x01;
memory[298] =  0x00;
memory[299] =  0x03;
memory[300] =  0x01;
memory[301] =  0x0d;
memory[302] =  0x1e;
memory[303] =  0x25;
memory[304] =  0x00;
memory[305] =  0x03;
memory[306] =  0x01;
memory[307] =  0x20;
memory[308] =  0x0f;
memory[309] =  0x0f;
memory[310] =  0x02;
memory[311] =  0x00;
memory[312] =  0x03;
memory[313] =  0x0c;
memory[314] =  0x01;
memory[315] =  0xcb;
memory[316] =  0x22;
memory[317] =  0x24;
memory[318] =  0x01;
memory[319] =  0x00;
memory[320] =  0x03;
memory[321] =  0x20;
memory[322] =  0x20;
memory[323] =  0x0b;
memory[324] =  0x02;
memory[325] =  0x0c;
memory[326] =  0x01;
memory[327] =  0x4f;
memory[328] =  0x0b;
memory[329] =  0x04;
memory[330] =  0x0c;
memory[331] =  0x01;
memory[332] =  0x4f;
memory[333] =  0x01;
memory[334] =  0x20;
memory[335] =  0x06;
memory[336] =  0x00;
memory[337] =  0x50;
memory[338] =  0x01;
memory[339] =  0x00;
memory[340] =  0x03;
memory[341] =  0x09;
memory[342] =  0x02;
memory[343] =  0x00;
memory[344] =  0x41;
memory[345] =  0x0b;
memory[346] =  0x02;
memory[347] =  0x0c;
memory[348] =  0x01;
memory[349] =  0x6b;
memory[350] =  0x0b;
memory[351] =  0x04;
memory[352] =  0x0c;
memory[353] =  0x01;
memory[354] =  0x6b;
memory[355] =  0x0b;
memory[356] =  0xfe;
memory[357] =  0x0c;
memory[358] =  0x01;
memory[359] =  0x8c;
memory[360] =  0x08;
memory[361] =  0x01;
memory[362] =  0x55;
memory[363] =  0x21;
memory[364] =  0x0a;
memory[365] =  0x21;
memory[366] =  0x02;
memory[367] =  0x00;
memory[368] =  0x41;
memory[369] =  0x0b;
memory[370] =  0x00;
memory[371] =  0x0c;
memory[372] =  0x01;
memory[373] =  0x8d;
memory[374] =  0x0b;
memory[375] =  0x01;
memory[376] =  0x0c;
memory[377] =  0x01;
memory[378] =  0x8d;
memory[379] =  0x22;
memory[380] =  0x06;
memory[381] =  0x00;
memory[382] =  0x41;
memory[383] =  0x09;
memory[384] =  0x09;
memory[385] =  0x02;
memory[386] =  0x00;
memory[387] =  0x41;
memory[388] =  0x0a;
memory[389] =  0x06;
memory[390] =  0x00;
memory[391] =  0x41;
memory[392] =  0x22;
memory[393] =  0x08;
memory[394] =  0x01;
memory[395] =  0x63;
memory[396] =  0x24;
memory[397] =  0x22;
memory[398] =  0x22;
memory[399] =  0x22;
memory[400] =  0x22;
memory[401] =  0x01;
memory[402] =  0x02;
memory[403] =  0x03;
memory[404] =  0x01;
memory[405] =  0x20;
memory[406] =  0x21;
memory[407] =  0x02;
memory[408] =  0x00;
memory[409] =  0x41;
memory[410] =  0x0b;
memory[411] =  0xfe;
memory[412] =  0x0c;
memory[413] =  0x01;
memory[414] =  0xa7;
memory[415] =  0x22;
memory[416] =  0x06;
memory[417] =  0x00;
memory[418] =  0x41;
memory[419] =  0x09;
memory[420] =  0x08;
memory[421] =  0x01;
memory[422] =  0x96;
memory[423] =  0x22;
memory[424] =  0x01;
memory[425] =  0x01;
memory[426] =  0x03;
memory[427] =  0x01;
memory[428] =  0x03;
memory[429] =  0x1e;
memory[430] =  0x27;
memory[431] =  0x00;
memory[432] =  0x03;
memory[433] =  0x0c;
memory[434] =  0x01;
memory[435] =  0xb7;
memory[436] =  0x08;
memory[437] =  0x01;
memory[438] =  0xae;
memory[439] =  0x01;
memory[440] =  0x00;
memory[441] =  0x03;
memory[442] =  0x02;
memory[443] =  0x00;
memory[444] =  0x15;
memory[445] =  0x0b;
memory[446] =  0xfe;
memory[447] =  0x0c;
memory[448] =  0x01;
memory[449] =  0xc9;
memory[450] =  0x1f;
memory[451] =  0x50;
memory[452] =  0x0f;
memory[453] =  0x09;
memory[454] =  0x08;
memory[455] =  0x01;
memory[456] =  0xba;
memory[457] =  0x1a;
memory[458] =  0x00;
memory[459] =  0x22;
memory[460] =  0x22;
memory[461] =  0x01;
memory[462] =  0x01;
memory[463] =  0x03;
memory[464] =  0x01;
memory[465] =  0x00;
memory[466] =  0x1e;
memory[467] =  0x01;
memory[468] =  0x00;
memory[469] =  0x03;
memory[470] =  0x02;
memory[471] =  0x00;
memory[472] =  0x20;
memory[473] =  0x0b;
memory[474] =  0xfe;
memory[475] =  0x0c;
memory[476] =  0x01;
memory[477] =  0xe5;
memory[478] =  0x1f;
memory[479] =  0x50;
memory[480] =  0x0f;
memory[481] =  0x09;
memory[482] =  0x08;
memory[483] =  0x01;
memory[484] =  0xd6;
memory[485] =  0x1f;
memory[486] =  0xff;
memory[487] =  0x1f;
memory[488] =  0xff;
memory[489] =  0x1f;
memory[490] =  0xff;
memory[491] =  0x1f;
memory[492] =  0xff;
memory[493] =  0x08;
memory[494] =  0x01;
memory[495] =  0x91;
}

// arduino loop
void loop() {
  lcd.setCursor(0, 1);
  lcd.print(" ");
  lcd.print("0x");
  print_word(current_addr);
  mode ? lcd.print(">") : lcd.print("<");
  if (current_addr < MEMORY_SIZE) {
    lcd.print("0x");
    uint8_t value = memory[current_addr];
    print_byte(value);
    lcd.print(" ");
    if (value <= 0x28) lcd.print(OPCODES[value]);
    else lcd.print("BYT");
  } else lcd.print("---- ---");
  
  char key = getch();
  if (command_keys) {
    switch(key) {
      case 'A': command_load(); break;
      case 'B': command_save(); break;
      case '2': mode ? memory[current_addr]++ : current_addr++; break;
      case '8': mode ? memory[current_addr]-- : current_addr--; break;
      case 'F': // execute
      case 'E': // single step
        if (current_addr < MEMORY_SIZE) {
          program_counter = current_addr;
          command_run(key);
        } break;
      case 'C': current_addr = program_counter; break;
      case 'D': print_debug(); break;
      case '4': mode = 0; break;
      case '6': mode = 1; break;
    }
  } else {
    if (!mode) {
      current_addr <<= 4;
      current_addr |= ascii_to_hex(key);
    } else if (current_addr < MEMORY_SIZE) {
      memory[current_addr] <<= 4;
      memory[current_addr] |= ascii_to_hex(key);
    }
  }
}

// TODO: Figure out why button presses are not working in Dino
// TODO: FIX PC layout in Debug!!!
// TODO: LOAD/SAVE from EEPROM!!!


