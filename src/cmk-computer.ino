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
const char MESSAGE_LOAD[] PROGMEM = {"LOAD "};
const char MESSAGE_SAVE[] PROGMEM = {"SAVE "};
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
bool execute() {
  // user interrupt
  if (keypad.getKey() == 'F') return 0;
  
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
    case RCH: zero_flag = (register_A = keypad.getKey()) == 0; break;
    case IN: while ((register_A = keypad.getKey()) == NO_KEY); break;
    case OUT: lcd.print(char(register_A)); break;
    case SER: Serial.print(char(register_A)); break;
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
  if (key == 'F') while(execute());
  else execute();
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
  delay(300);
  lcd.clear();
  print_message_lcd(MESSAGE_WAITING);
  lcd.setCursor(0, 1);
  print_message_lcd(MESSAGE_INCOMING);
  while (Serial.available() == 0);
  lcd.clear();
  print_message_lcd(MESSAGE_LOADING);
  Serial.readBytes(memory, MEMORY_SIZE);
  
  // ascii to bytes       
  for (int i = 0; i < MEMORY_SIZE; i++) {
    memory[i] = ascii_to_hex(memory[i]);
    if ((i % 2) == 0) memory[i] <<= 4;
    else {
      memory[i - 1] |= memory[i];
      memory[i] = 0;
    }
  }
  
  // group bytes
  for (int i = 0; i < MEMORY_SIZE; i++) {
    if ((i % 2) == 0) {
      if (i) {
        memory[i - ((int)(i / 2))] = memory[i];
        memory[i] = 0;
      }
    }
  }

  // verify transfered bytes
  //Serial.println("Your program bytes loaded:");
  //for (int i = 0; i < MEMORY_SIZE; i++) print_hex(memory[i]);

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
  
  for (int i = 0; i < MEMORY_SIZE; i++) {
    if ( memory[i] < 0x10) print_message_serial(MESSAGE_ZERO);
    Serial.print(memory[i], HEX);
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
  
memory[0] =  0x01;
memory[1] =  0x00;
memory[2] =  0x03;
memory[3] =  0x02;
memory[4] =  0x00;
memory[5] =  0x12;
memory[6] =  0x0b;
memory[7] =  0x00;
memory[8] =  0x0c;
memory[9] =  0x00;
memory[10] =  0x20;
memory[11] =  0x1f;
memory[12] =  0xff;
memory[13] =  0x0f;
memory[14] =  0x09;
memory[15] =  0x08;
memory[16] =  0x00;
memory[17] =  0x03;
memory[18] =  0x48;
memory[19] =  0x65;
memory[20] =  0x6c;
memory[21] =  0x6c;
memory[22] =  0x6f;
memory[23] =  0x2c;
memory[24] =  0x20;
memory[25] =  0x77;
memory[26] =  0x6f;
memory[27] =  0x72;
memory[28] =  0x6c;
memory[29] =  0x64;
memory[30] =  0x21;
memory[31] =  0x00;
memory[32] =  0x01;
memory[33] =  0x01;
memory[34] =  0x03;
memory[35] =  0x01;
memory[36] =  0x00;
memory[37] =  0x1e;
memory[38] =  0x00;
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
  switch(key) {
    case 'A': command_load(); break;
    case 'B': command_save(); break;
    case '2': // current_addr/data++
      mode ? memory[current_addr]++ : current_addr++; break;
    case '8': // current_addr/data--
      mode ? memory[current_addr]-- : current_addr--; break;
    case 'F': // execute
    case 'E': // single step
      if (current_addr < MEMORY_SIZE) {
        program_counter = current_addr;
        command_run(key);
      }
    case 'C': current_addr = program_counter; break;
    case 'D': print_debug(); break;
    case '4': // ADDR mode
      mode = 0; break;
    case '6': // DATA mode
      mode = 1; break;
    
    /*default:
      if (!mode) {
        current_addr <<= 4;
        current_addr |= ascii_to_hex(key);
      } else if (current_addr < MEMORY_SIZE) {
        memory[current_addr] <<= 4;
        memory[current_addr] |= ascii_to_hex(key);
      } break;*/
  }
}
 
