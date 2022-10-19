/****************************************************************\
 ================================================================
                      
                        HARDWARE SETUP TEST
                     
================================================================
\****************************************************************/

// libraries
#include <LiquidCrystal.h>
#include <Keypad.h>


/****************************************************************\
 ================================================================
                    CIRCUIT for Arduino NANO/UNO
 ================================================================
           ----------------------------------------------
            Arduino LCD Shield 1602A D1 ROBOT (DF ROBOT)
           ----------------------------------------------

              LCD RS pin to arduino digital pin 8
              LCD E pin to arduino digital pin 9
              LCD Backlight LED pin to arduino pin 10
              LCD D4 pin to arduino digital pin 4
              LCD D5 pin to arduino digital pin 5
              LCD D6 pin to arduino digital pin 6
              LCD D7 pin to arduino digital pin 7
              LCD R/W pin to arduino ground pin
              LCD VSS pin to arduino ground pin
              LCD VCC pin to arduino 5V pin
              SHIELD A0 to arduino A0 (shield buttons analog control)
              SHIELD VIN to arduino VIN
              SHIELD GND to arduino GND
              SHIELD VCC to arduino VCC
              SHIELD RST to arduino RST

                      --------------------
                       Arduino 4x4 keypad
                      --------------------
              
              KEYPAD row 1 to arduino digital pin D12
              KEYPAD row 2 to arduino digital pin D11
              KEYPAD row 3 to arduino digital pin D3
              KEYPAD row 4 to arduino digital pin D2
              KEYPAD column 1 to arduino analog pin A4
              KEYPAD column 2 to arduino analog pin A3
              KEYPAD column 3 to arduino analog pin A2
              KEYPAD column 4 to arduino analog pin A1

 ================================================================
\****************************************************************/

// LCD pins
#define RS 8    // LCD reset pin
#define En 9    // LCD enable pin
#define D4 4    // LCD data pin 4
#define D5 5    // LCD data pin 5
#define D6 6    // LCD data pin 6
#define D7 7    // LCD data pin 7

// keypad size
const byte num_rows= 4;
const byte num_cols= 4;

// map keypad keys
uint8_t keymap[num_rows][num_cols] = {
  {'A', 'B', 'C', 'D'},    // { '1', '2', '3', 'A'},  // normal orientation
  {'3', '6', '9', 'F'},    // { '4', '5', '6', 'B'},
  {'2', '5', '8', '0'},    // { '7', '8', '9', 'C'},
  {'1', '4', '7', 'E'},    // { 'E', '0', 'F', 'D'}
};

// map keypad rows and columns
byte row_pins[num_rows] = {12, 11, 3, 2};     // Rows 0 to 3
byte col_pins[num_cols] = {A4, A3, A2, A1};   // Columns 0 to 3

// init kepad
Keypad myKeypad = Keypad(makeKeymap(keymap), row_pins, col_pins, num_rows, num_cols);

// init LCD
LiquidCrystal lcd(RS, En, D4, D5, D6, D7);  //interfacing pins

// setup arduino
void setup()
{
  // set up the LCD's number of columns and rows:
  lcd.begin(16, 2);
  
  // Print a message to the LCD.
  lcd.setCursor(0,0);
  lcd.print("LCD Key Shield");
  lcd.setCursor(0,1);
  lcd.print("Press Key:");
}

// main loop
void loop()
{
  // test LCD shield keys
  int x;
  x = analogRead (0);
  lcd.setCursor(10,1);
  if (x < 60) {
    lcd.print ("Right ");
  }
  else if (x < 200) {
    lcd.print ("Up    ");    //analog voltage 145
  }
  else if (x < 400){
    lcd.print ("Down  ");    //analog voltage 329
  }
  else if (x < 600){
    lcd.print ("Left  ");    //analog voltage 585
  }
  else if (x < 800){
    lcd.print ("Select");    //analog voltage 741
  }

  // test 4x4 keypad keys
  char key;
  key = myKeypad.getKey();
  if (key != NO_KEY) {
    lcd.print(key);
    lcd.print("     ");
  }
}
