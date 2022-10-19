# CMK computer
CMK computer is a bundle of arduino nano/uno, 16x2 LCD shield and 4x4 keypad<br>
8-bit CPU emulator and a hex editor are flashed into ATMega328 microcontroller<br>
Programs can be entered via machine codes from the keypad or loaded via serial port<br>

# Programming CMK computer tutorials
[![IMAGE ALT TEXT HERE](https://img.youtube.com/vi/Rvx2P4ulBlA/0.jpg)](https://www.youtube.com/watch?v=BFnaWnjz324&list=PLLfIBXQeu3aaMjeyPxJzT34DAG6v1vQqq)

# Try it online
[![IMAGE ALT TEXT HERE](https://raw.githubusercontent.com/maksimKorzh/cmk-computer/main/Keypad_connect.png?token=AIFH42PHZAH27XM37VCJ2NDBPQDDO)](https://maksimkorzh.github.io/cmk-computer/)


# Memory
 - 1024 bytes of RAM (0x0000 - 0x0400)
 - 2048 bytes of ROM (0x0400 - 0x0C00)
 - stack (stack pointer points to the last byte in RAM and growth downwards)
 - first 750 bytes are used to store program instructions and variables
 - last 750 bytes are shared between program variables and stack
 - next 7 bytes after RAM hold registers' state,<br>
   however manipulating them directly is not recommended<br>
   unless you know exactly what are you doing)

# Control commands
    FFFD  (LCD shield 'left' button)    load program via serial port
    FFFF  (LCD shield 'select' button)  run program
    FFFC  (LCD shield 'up' button)      show 4 byte in memory at entered address
    FFFA  (LCD shield 'down' button)    clear LCD screen
    FFFB  (LCD shield 'reset' button)   software/hardware reset
    FFFE  (LCD shield 'right' button)   save program via serial port (1k memory dump)

# Operation codes / Assembly mnemonics
    ----------------------------------------------------------
     hex  asm  arg   description
    ----------------------------------------------------------
    0x00  NOP        no operation, resets program counter
    0x01  LDI  byte  load immediate data to A register
    0x02  LDA  word  load data from memory address with register B offset to A register
    0x03  TAB        transfer data from A to B register
    0x04  ADD  byte  add immediate data to A register and store it
    0x05  SUB  byte  subtract immediate data from A register and store it
    0x06  STA  word  set value from A register at memory address with register B offset
    0x07  RCH        read character from keypad (non-blocking)
    0x08  LPC  word  load data from memory address to program counter
    0x09  INC        increment value in register B
    0x0a  DCR        decrement value in register B
    0x0b  CMP  byte  compare register A and immediate value, set zero flag
    0x0c  JMP  word  jump program counter to memory address if zero flag is true
    0x0d  DBG        print debug info to serial port
    ----------------------------------------------------------
    0x0e  IN         get user input from keypad to A register (blocking)
    0x0f  OUT        output character from A register to LCD
    ----------------------------------------------------------
    0x10  BIT  byte  bitwise AND register A with immediate data, set zero flag
    0x11  AND  byte  bitwise AND register A with immediate data, store result to A register
    0x12  OR   byte  bitwise OR register A with immediate data, store result to A register
    0x13  XOR  byte  bitwise XOR register A with immediate data, store result to A register
    0x14  NOT  byte  bitwise NOT immediate data, store result to A register
    0x15  SHL  byte  bitwise shift left register A with immediate data
    0x16  SHR  byte  bitwise shift right register A with immediate data
    ----------------------------------------------------------
    0x17  CLS        clear LCD display
    0x18  SDL        Scrolls the contents of the display one space to the left
    0x19  SDR        Scrolls the contents of the display one space to the right
    0x1a  CRS        enable cursor (blink)
    0x1b  NCR        disable cursor (no blink)
    0x1c  UDG        create user defined character (assumes: A equals character id (0-7), B points to byte array)
    0x1d  SPR  byte  draw sprite (0-7)
    0x1e  POS        set cursor at position (assumes: A equals to column, B equals to row)
    ----------------------------------------------------------
    0x1f  DLY  byte  delay execution
    0x20  RND  byte  load random number between 0 and immediate date into A register
    ----------------------------------------------------------
    0x21  PSH        push register A, then register B to stack
    0x22  POP        pop register B, then register A from stack
    0x23  SBR  word  call subroutine at memory address
    0x24  RET        return from subroutine
    ----------------------------------------------------------
    0x25  NUM  word  print decimal number at memory address
    0x26  INM  word  increment value at memory address
    0x27  DCM  word  decrement value at memory address
    ----------------------------------------------------------
    0x28  SER        output character from A register to serial port

# Hello world example program
    start:         ; program start
      ldi 0x00     ; load 0 to A register
      tab          ; transfer the 0 from A to B register

    print:         ; print string loop
      lda hello    ; load byte at 'hello' label address + B register offset
      cmp 0x00     ; zero terminating character?
      jmp exit     ; if so then exit the program
      dly 0xff     ; wait for 255 ms
      out          ; print character to LCD display
      inc          ; increment B reister by 1
      lpc print    ; jump to 'print' label

    hello:         ; 'Hello, world!' string
      byte 0x48    ; 'H'
      byte 0x65    ; 'e'
      byte 0x6c    ; 'l'
      byte 0x6c    ; 'l'
      byte 0x6f    ; 'o'
      byte 0x2c    ; ','
      byte 0x20    ; ' '
      byte 0x77    ; 'w'
      byte 0x6f    ; 'o'
      byte 0x72    ; 'r'
      byte 0x6c    ; 'l'
      byte 0x64    ; 'd'
      byte 0x21    ; '!'
      
    hello_end:     ; prorgam end
      byte 0x00    ; zero terminating character

    exit:          ; clean ups before exit
      ldi 0x02     ; load 2 to A register
      tab          ; transfer to from A to B register
      ldi 0x00     ; load 0 to A register
      pos          ; position cursor at A, B (col 0, row 2)
      byte 0x00    ; execution terminates here

# Assemble program
    python3 assembler.py hello.asm [0x200]

# Circuit example
![LCD connect](https://raw.githubusercontent.com/maksimKorzh/cmk-computer/main/LCD_connect.png?token=AIFH42ONBYZL4KWSQ5HZGL3BPQDE4)

