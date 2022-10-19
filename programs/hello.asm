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
  dbg
  in
  cls
  ldi 0x00     ; load 2 to A register
  tab          ; transfer to from A to B register
  ldi 0x00     ; load 0 to A register
  pos          ; position cursor at A, B (col 0, row 0)
  byte 0x00    ; execution terminates here