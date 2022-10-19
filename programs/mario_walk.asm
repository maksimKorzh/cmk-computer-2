start:                             ; program start
  lpc create_graphics              ; jump to create_graphics label

mario_head_left:                   ; left part of Mario's head
  byte 0x00                        ; 0000 0000
  byte 0x00                        ; 0000 0000
  byte 0x01                        ; 0000 0001
  byte 0x03                        ; 0000 0011
  byte 0x03                        ; 0000 0011
  byte 0x05                        ; 0000 0101
  byte 0x05                        ; 0000 0101
  byte 0x06                        ; 0000 0110

mario_head_right:                  ; right part of Mario's head
  byte 0x00                        ; 0000 0000
  byte 0x00                        ; 0000 0000
  byte 0x18                        ; 0001 1000
  byte 0x1f                        ; 0001 1111
  byte 0x08                        ; 0000 1000
  byte 0x08                        ; 0000 1000
  byte 0x04                        ; 0000 0100
  byte 0x0f                        ; 0000 1111

mario_body_left_1:                 ; left part of Mario's body standing
  byte 0x0f                        ; 0000 1111
  byte 0x1f                        ; 0001 1111
  byte 0x1e                        ; 0001 1110
  byte 0x18                        ; 0001 1000
  byte 0x01                        ; 0000 0001
  byte 0x07                        ; 0000 0111
  byte 0x07                        ; 0000 0111
  byte 0x04                        ; 0000 0100

mario_body_right_1:                ; right part of Mario's body standing
  byte 0x18                        ; 0001 1000
  byte 0x10                        ; 0001 0000
  byte 0x08                        ; 0000 1000
  byte 0x18                        ; 0001 1000
  byte 0x08                        ; 0000 1000
  byte 0x08                        ; 0000 1000
  byte 0x10                        ; 0001 0000
  byte 0x18                        ; 0001 1000

mario_body_left_2:                 ; left part of Mario's body stepping
  byte 0x0f                        ; 0000 1111
  byte 0x1f                        ; 0001 1111
  byte 0x0e                        ; 0000 1110
  byte 0x08                        ; 0000 1000
  byte 0x01                        ; 0000 0001
  byte 0x08                        ; 0000 1000
  byte 0x1f                        ; 0001 1111
  byte 0x0c                        ; 0000 1100

mario_body_right_2:                ; right part of Mario's body stepping
  byte 0x1c                        ; 0001 1100
  byte 0x18                        ; 0001 1000
  byte 0x08                        ; 0000 1000
  byte 0x0d                        ; 0000 1101
  byte 0x07                        ; 0000 0111
  byte 0x1f                        ; 0001 1111
  byte 0x03                        ; 0000 0011
  byte 0x00                        ; 0000 0000

create_graphics:                   ; create user defined graphics (cprites/characters)
  ldi mario_head_left              ; load address of mario_head_left to A register
  tab                              ; and transfer it to B register to init the data pointer
  ldi 0x00                         ; init index of 0 for the mario_head_left sprite
  udg                              ; create user defined character
  
end:
  lpc end
  
  
  ldi mario_head_right             ; load address of mario_head_right to A register
  tab                              ; and transfer it to B register to init the data pointer
  ldi 0x01                         ; init index of 1 for the mario_head_right sprite
  udg                              ; create user defined character
  ldi mario_body_left_1            ; load address of mario_body_left_1 to A register
  tab                              ; and transfer it to B register to init the data pointer
  ldi 0x02                         ; init index of 2 for the mario_body_left_1 sprite
  udg                              ; create user defined character
  ldi mario_body_right_1           ; load address of mario_body_right_1 to A register
  tab                              ; and transfer it to B register to init the data pointer
  ldi 0x03                         ; init index of 3 for the mario_body_right_1 sprite
  udg                              ; create user defined character
  ldi mario_body_left_2            ; load address of mario_body_left_2 to A register
  tab                              ; and transfer it to B register to init the data pointer
  ldi 0x04                         ; init index of 4 for the mario_body_left_2 sprite
  udg                              ; create user defined character
  ldi mario_body_right_2           ; load address of mario_body_right_2 to A register
  tab                              ; and transfer it to B register to init the data pointer
  ldi 0x05                         ; init index of 5 for the mario_body_right_2 sprite
  udg                              ; create user defined character

mario_walk:
  ldi 0x00                         ; reset register A
  tab                              ; reset offset in register B
  lda count                        ; load the value at count variable + 0 offset (in B register) to A register
  cmp 0x11                         ; did Mario went off the screen?
  jmp reset_mario                  ; if so place him into the starting point
  inm count                        ; increment count variable by 1
  ldi 0x00                         ; load 0x00 to A register
  tab                              ; and transfer it to B register to put cursor to the 2nd row
  ldi 0x00                         ; load 0x00 to A register to put cursor to the first column
  pos                              ; set cursor at (0, 0)
  spr 0x00                         ; draw mario_head_left on LCD screen
  spr 0x01                         ; draw mario_head_right on LCD screen
  ldi 0x01                         ; load 0x01 to A register
  tab                              ; and transfer it to B register to put cursor to the 2nd row
  ldi 0x00                         ; load 0x00 to A register to put cursor to the first column
  pos                              ; set cursor at (0, 1)
  spr 0x02                         ; draw mario_body_left_1 on LCD screen
  spr 0x03                         ; draw mario_head_right on LCD screen
  dly 0xff                         ; 255ms delay
  dly 0xff                         ; 255ms delay
  ldi 0x01                         ; load 0x01 to A register
  tab                              ; and transfer it to B register to put cursor to the 2nd row
  ldi 0x00                         ; load 0x00 to A register to put cursor to the first column
  pos                              ; set cursor at (0, 1)
  spr 0x04                         ; draw mario_body_left_1 on LCD screen
  spr 0x05                         ; draw mario_head_right on LCD screen
  dly 0xff                         ; 255ms delay
  dly 0xff                         ; 255ms delay
  sdr                              ; scroll display and cursor to the right to mimic Mario walk
  lpc mario_walk                   ; repeat mario_walk loop

count:                             ; variable holding the value of how far Mario walked from left to right
  byte 0x00                        ; init counter to 0

reset_mario:                       ; place Mario to the left part of the screen
  ldi 0x00                         ; reset A register
  sta 0x0000                        ; reset counter with the value of A register
  cls                              ; clear the LCD display
  lpc mario_walk                   ; continue main Mario walk loop

 



































