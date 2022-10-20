start:                ; program start
  ncr                 ; disable cursor
  ldi 0x00            ; load 0 to A register
  tab                 ; transfer the 0 from A to B register
  pos                 ; set cursor at (0, 0)
  
  inm 0x0000
  lpc end

print_caps:           ; print string loop
  lda caps            ; load byte at 'hello' label address + B register offset
  cmp 0x00            ; zero terminating character?
  jmp print_vals      ; if so then exit the program
  out                 ; print character to LCD display
  inc                 ; increment B reister by 1
  lpc print_caps      ; jump to 'print' label

print_vals:
  num 0x1111
  lpc end

caps:                 ; ' ADDRESS  DATA' string
  byte 0x20           ; " '
  byte 0x41           ; "A'
  byte 0x44           ; "D'
  byte 0x44           ; "D'
  byte 0x52           ; "R'
  byte 0x45           ; "E'
  byte 0x53           ; "S'
  byte 0x53           ; "S'
  byte 0x20           ; " '
  byte 0x20           ; " '
  byte 0x44           ; "D'
  byte 0x41           ; "A'
  byte 0x54           ; "T'
  byte 0x41           ; "A'
  byte 0x00           ; zero terminating character

end:                  ; clean ups before exit
  lpc end             ; stuck in infinite loop