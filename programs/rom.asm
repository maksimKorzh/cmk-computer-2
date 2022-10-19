start:                ; program start
  ldi 0x00            ; load 0 to A register
  tab                 ; transfer the 0 from A to B register

print_labels:         ; print string loop
  lda hello           ; load byte at 'hello' label address + B register offset
  cmp 0x00            ; zero terminating character?
  jmp print_values    ; if so then exit the program
  dly 0xff            ; wait for 255 ms
  out                 ; print character to LCD display
  in
  
  inc                 ; increment B reister by 1
  lpc print_labels    ; jump to 'print' label

hello:                ; 'Hello, world!' string
  byte 0x20           ; ' '
  byte 0x41           ; 'A'
  byte 0x44           ; 'D'
  byte 0x44           ; 'D'
  byte 0x52           ; 'R'
  byte 0x45           ; 'E'
  byte 0x53           ; 'S'
  byte 0x53           ; 'S'
  byte 0x20           ; ' '
  byte 0x20           ; ' '
  byte 0x44           ; 'D'
  byte 0x41           ; 'A'
  byte 0x54           ; 'T'
  byte 0x41           ; 'A'
  byte 0x00           ; terminate string

print_values:
  ldi 0x01     ; load 2 to A register
  tab          ; transfer to from A to B register
  ldi 0x00     ; load 0 to A register
  pos          ; position cursor at A, B (col 0, row 1)
  byte 0x00    ; execution terminates here









