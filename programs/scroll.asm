start:         ; program start
  ncr          ; no cursor
  ldi 0x00     ; load 0 to A register
  tab          ; transfer zero from A to B register
  ldi 0x10     ; load 
  pos
  ldi 0x21     ; load ASCII '!' to register A
  
loop:          ; program loop
  add 0x01     ; increment A register by one
  out          ; print character to LCD
  sdl          ; scroll display left
  dly 0xff     ; wait 255 ms
  dly 0xff     ; wait 255 ms
  lpc loop     ; print next character

exit:          ; end of program
  byte 0x00    ; terminate execution
  

