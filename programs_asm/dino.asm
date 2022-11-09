;----------------------------------------;
;
;    Chrome Dino port to CMK computer
;
;                   by
;
;            Code Monkey King
;
;----------------------------------------;

start:                  ; for programs bigger than around 200 bytes
  lpc init              ; make sure to store data in the beginning of the file
                        ; and then jump to program start for otherwise
                        ; local variable addresses may exceed 0xff resulting
                        ; in undefined behavior, however if you're using direct
                        ; addressing for LDA/STA instructions instead of labels
                        ; or %define directives you're free to put variables
                        ; wherever in memory you like.

score:                  ; game score
  byte 0x00             ; score byte

intro:                  ; 'ARE YOU READY?'
  byte 0x20             ; ' '
  byte 0x41             ; 'A'
  byte 0x52             ; 'R'
  byte 0x45             ; 'E'
  byte 0x20             ; ' '
  byte 0x59             ; 'Y'
  byte 0x4f             ; 'O'
  byte 0x55             ; 'U'
  byte 0x20             ; ' '
  byte 0x52             ; 'R'
  byte 0x45             ; 'E'
  byte 0x41             ; 'A'
  byte 0x44             ; 'D'
  byte 0x59             ; 'Y'
  byte 0x3f             ; '?'
  byte 0x20             ; ' '
  byte 0xfe             ; terminating byte

outro_lose:             ; 'GAME OVER!'
  byte 0x47             ; 'G'
  byte 0x41             ; 'A'
  byte 0x4d             ; 'M'
  byte 0x45             ; 'E'
  byte 0x20             ; ' '
  byte 0x4f             ; 'O'
  byte 0x56             ; 'V'
  byte 0x45             ; 'E'
  byte 0x52             ; 'R'
  byte 0x21             ; '!'
  byte 0xfe             ; terminating byte

outro_win:              ; 'YOU WIN!'
  byte 0x20             ; ' '
  byte 0x00             ; dino character
  byte 0x02             ; cactus character
  byte 0x20             ; ' '
  byte 0x59             ; 'Y'
  byte 0x4f             ; 'O'
  byte 0x55             ; 'U'
  byte 0x20             ; ' '
  byte 0x57             ; 'W'
  byte 0x49             ; 'I'
  byte 0x4e             ; 'N'
  byte 0x21             ; '!'
  byte 0x20             ; ' '
  byte 0x02             ; cactus character
  byte 0x00             ; dino character
  byte 0x20             ; ' '
  byte 0xfe             ; terminating byte

world_row_1:            ; game world map, 1st row (16 characters)
  byte 0x20             ; ' '
  byte 0x20             ; ' ', dino jumps here
  byte 0x20             ; ' '
  byte 0x20             ; ' '
  byte 0x20             ; ' '
  byte 0x20             ; ' '
  byte 0x20             ; ' '
  byte 0x53             ; 'S'
  byte 0x63             ; 'c'
  byte 0x6f             ; 'o'
  byte 0x72             ; 'r'
  byte 0x65             ; 'e'
  byte 0x3a             ; colon character
  byte 0x20             ; ' '
  byte 0x20             ; ' '
  byte 0x20             ; ' '

world_row_2:            ; game world map, 2nd row (16 characters)
  byte 0x20             ; ' '
  byte 0x00             ; dino character
  byte 0x20             ; ' '
  byte 0x20             ; ' '
  byte 0x20             ; ' '
  byte 0x20             ; ' '
  byte 0x20             ; ' '
  byte 0x20             ; ' '
  byte 0x20             ; ' '
  byte 0x20             ; ' '
  byte 0x20             ; ' '
  byte 0x20             ; ' '
  byte 0x20             ; ' '
  byte 0x20             ; ' '
  byte 0x20             ; ' '

random_object:          ; cactuses appearing here randomly
  byte 0x20             ; ' ' 
  byte 0xfe             ; terminating byte

dino_l:                 ; dino on left leg custom character
  byte 0x07             ; 00000111
  byte 0x05             ; 00000101
  byte 0x07             ; 00000111
  byte 0x16             ; 00010110
  byte 0x1f             ; 00011111
  byte 0x1e             ; 00011110
  byte 0x0e             ; 00001110
  byte 0x04             ; 00000100

dino_r:                 ; dino on right leg custom character
  byte 0x07             ; 00000111
  byte 0x05             ; 00000101
  byte 0x07             ; 00000111
  byte 0x16             ; 00010110
  byte 0x1f             ; 00011111
  byte 0x1e             ; 00011110
  byte 0x0e             ; 00001110
  byte 0x02             ; 00000010

cactus_big:             ; big cactus custom character
  byte 0x00             ; 00000000
  byte 0x04             ; 00000100
  byte 0x05             ; 00000101
  byte 0x15             ; 00010101
  byte 0x16             ; 00010110
  byte 0x0c             ; 00001100
  byte 0x04             ; 00000100
  byte 0x04             ; 00000100

cactus_small:           ; small cactus custom character
  byte 0x00             ; 00000000
  byte 0x00             ; 00000000
  byte 0x04             ; 00000100
  byte 0x05             ; 00000101
  byte 0x15             ; 00010101
  byte 0x16             ; 00010110
  byte 0x0c             ; 00001100
  byte 0x04             ; 00000100

init:                   ; program start
  ldi dino_l            ; load address of dino_l to A register
  tab                   ; and transfer it to B register
  ldi 0x00              ; store character ID 0x00 to A register
  udg                   ; create dino_l character
  ldi dino_r            ; load address of dino_r to A register
  tab                   ; and transfer it to B register
  ldi 0x01              ; store character ID 0x01 to A register
  udg                   ; create dino_r character
  ldi cactus_big        ; load address of cactus_big to A register
  tab                   ; and transfer it to B register
  ldi 0x02              ; store character ID 0x02 to A register
  udg                   ; create cactus_big character
  ldi cactus_small      ; load address of cactus_small to A register
  tab                   ; and transfer it to B register
  ldi 0x04              ; store character ID 0x04 to A register
  udg                   ; create cactus_small character
  ldi 0x00              ; reset A register
  tab                   ; reset B register
  cls                   ; clear LCD display
  
print_intro:            ; print intro string loop
  lda intro             ; load byte at 'intro' label address with B register offset
  cmp 0xfe              ; is it a zero terminating character?
  jmp get_ready         ; if so then jump to 'get_ready' label
  dly 0x50              ; 80ms delay
  out                   ; print character to LCD display
  inc                   ; increment B reister by 1
  lpc print_intro       ; jump to 'print_intro' label

get_ready:              ; wait for user keypress to start the game
  rch                   ; listen to user input
  cmp 0x30              ; if user presses '0' on keypad
  jmp clear_screen      ; jump to 'clear_screen' label
  lpc get_ready         ; otherwise repeat the loop

clear_screen:           ; clear screen before starting the game
  cls                   ; clear LCD display

game_loop:              ; main game loop
  rch                   ; read character from keypad
  cmp 0x30              ; is it '0' key?
  jmp dino_jump         ; if so make dino jump
  sbr update_world      ; otherwise render game scene
  lpc game_loop         ; repeat game loop

dino_jump:              ; jump over cactus
  ldi 0x00              ; reset A register
  tab                   ; reset B register
  ldi 0x00              ; reset A register (it is still 0, so this line is just for clarity)
  pos                   ; set cursor at (0, 0)
  ldi 0x01              ; load dino offset to A register
  tab                   ; and transfer it to B register
  lda world_row_2       ; load dino character from row 2 to A register
  sta world_row_1       ; and store it to row 1
  ldi 0x20              ; load empty space to A register
  sta world_row_2       ; and replace dino on row 2 with the space
  sbr update_world      ; scroll
  sbr update_world      ; game world
  sbr update_world      ; 3 times
  ldi 0x00              ; reset A register
  tab                   ; reset B register
  ldi 0x00              ; reset A register (it is still 0, so this line is just for clarity)
  pos                   ; set cursor at (0, 0)
  ldi 0x01              ; load dino offset to register A
  tab                   ; and transfer it to B register
  lda world_row_1       ; load dino character from row 1 to A register
  sta world_row_2       ; and store it back to row 2
  ldi 0x20              ; load empty space to A register
  sta world_row_1       ; and replace dino on row 1 with the space
  ldi 0x20              ; load empty space to A register
  dcr                   ; decrement register B by 1
  sta world_row_2       ; draw space behind the dino
  lpc game_loop         ; jump back to game loop

update_world:           ; print game world procedure
  dly 0xff              ; 255ms delay
  sbr scroll_world      ; scroll world by 1 cell
  ldi 0x00              ; reset A register
  tab                   ; reset B register

print_loop:             ; print world loop
  cmp 0x3a              ; A register equals to colon char?
  jmp next_line         ; if so it's time to update cursor position

load_next_char:         ; print next char in game world array
  lda world_row_1       ; load byte at 'world_row_1' label address + B register offset
  jmp update_dino       ; if encounter a dino_l then swap legs (since zero flag is set)
  cmp 0x01              ; if encounter a dino_l
  jmp update_dino       ; then swap legs as well

load_end:               ; continue routine
  cmp 0xfe              ; terminating character has been reached?
  jmp update_return     ; if so then exit the program
  out                   ; print character to LCD display
  inc                   ; increment B reister by 1
  lpc print_loop        ; jump to 'print_loop' label

update_dino:            ; mimic dino run
  xor 0x01              ; swap dino legs
  sta world_row_1       ; replace dino in game world map
  lpc load_end          ; continue loop

next_line:              ; handle LCD cursor placement
  psh                   ; preserve A and B registers on stack
  ldi 0x01              ; load 0x01 to A register
  tab                   ; transfer 0x01 to B register (cursor row)
  ldi 0x00              ; load 0x00 to A register (cursor column)
  pos                   ; set cursor at (0, 1)
  pop                   ; restore A and B registers
  inc                   ; skip
  inc                   ; three
  inc                   ; bytes
  lpc load_next_char    ; print the rest of the game world

update_return:          ; all done
  inm score             ; increment game score
  psh                   ; preserve A and B registers on stack
  ldi 0x00              ; set 0x00 to A register
  tab                   ; and transfer it to B register
  ldi 0x0d              ; set A register to offset 13
  pos                   ; set cursor at (0, 13)
  num score             ; print game score to screen
  ldi 0x20              ; load ' ' to A register
  out                   ; print space on LCD
  out                   ; print space on LCD
  lda score             ; load game score to A register
  jmp win               ; zero flag is set when 255 + 1 turns to 0, so the game is won
  pop                   ; restore A and B registers from stack
  ret                   ; return from procedure

scroll_world:           ; infinite scroll game world
  ldi 0x00              ; load 0x00 to A register
  tab                   ; reset B register offset
  rnd 0x20              ; generate random number in range from 0x00 to 0x20
  cmp 0x02              ; is it a big cactus?
  jmp set_object        ; if so then set it on game world map
  cmp 0x04              ; is it a small cactus?
  jmp set_object        ; if so then set it on game world map
  ldi 0x20              ; otherwise load ' ' to A register and set on game world map

set_object:             ; random source set
  sta random_object     ; set object (cactus_small/cactus_big/empty square)
  ldi 0x00              ; load 0x00 A register 
  tab                   ; and transfer it to B register to init the offset
  
scroll_loop:            ; loop over bytes in game world row 2
  inc                   ; increment B register by one to increase the memory lookup offset
  lda world_row_2       ; load value at address of world_row_2 to A register
  cmp 0x02              ; is it a big cactus?
  jmp scroll_object     ; if so then scroll it one cell to the left
  cmp 0x04              ; is it a small cactus?
  jmp scroll_object     ; if so then scroll it one cell to the left

scroll_loop_end:        ; continue scroll loop
  cmp 0xfe              ; end of row?
  jmp scroll_return     ; then all is done
  lpc scroll_loop       ; otherwise scroll next byte

scroll_object:          ; scroll a cactus
  psh                   ; preserve A and B registers on stack
  dcr                   ; decrement B register, so it points to next cell to scroll cactus to
  psh                   ; restore A and B registers
  lda world_row_2       ; get next cell's content
  cmp 0x00              ; is it a dino_l?
  jmp game_over         ; if so the game is over
  cmp 0x01              ; is it a dino_r
  jmp game_over         ; if so the game is over
  pop                   ; restore A and B registers from stack
  sta world_row_2       ; store big or small cactus on the next cell
  inc                   ; restore original cactus location cell offset
  inc                   ; point B register to previous cell offset
  lda world_row_2       ; source it's content
  dcr                   ; then drop back to current cell again
  sta world_row_2       ; and replace it's content byte the content of the previous cell
  pop                   ; restore A and B registers from stack
  lpc scroll_loop_end   ; jump to scroll_loop_end

scroll_return:          ; all done
  ret                   ; return from procedure

game_over:              ; dino hits a cactus
  pop                   ; adjust
  pop                   ; the stack pointer
  pop                   ; so it points to
  pop                   ; the original value of 0x3ff
  
reset_map:              ; needed to play the game again without reloading
  ldi 0x02              ; set register A to 0x02
  tab                   ; and transfer it to B register
  ldi 0x20              ; load ' ' to A register

reset_loop:             ; reset game world map
  psh                   ; preserve A and B registers
  lda world_row_2       ; load value at memory address of 'world_row_2' + B register offset to A register
  cmp 0xfe              ; is it a terminating byte?
  jmp reset_end         ; if so jump to 'reset_end' label
  pop                   ; otherwise preserve A and B registers from stack
  sta world_row_2       ; set ' ' to the next cell in game world map
  inc                   ; increment B register by 1 to increase the offset
  lpc reset_loop        ; repeat reset_loop

reset_end:              ; the game world map is now ready for a new game
  pop                   ; fix stack pointer
  ldi 0x01              ; set A register to 0x01
  tab                   ; set B register to 0x01
  ldi 0x03              ; set A register to 0x03
  pos                   ; set cursor at (0, 3)

reset_score:            ; reset game score
  dcm score             ; decrement value at the memory address of 'score'
  jmp all_done          ; until it's equal to 0 and zero flag is set, then jump to 'all_done'
  lpc reset_score       ; repeat reset score

all_done:               ; we are ready to start a new game
  ldi 0x00              ; reset A register
  tab                   ; reset B register

print_outro_lose:       ; print 'GAME OVER!'
  lda outro_lose        ; load byte at 'outro_lose' label address + B register offset
  cmp 0xfe              ; zero terminating character?
  jmp exit              ; if so then exit the program
  dly 0x50              ; 80ms delay
  out                   ; print character to LCD display
  inc                   ; increment B reister by 1
  lpc print_outro_lose  ; jump to 'print_outro_lose' label
  
exit:                   ; all done, exit the program
  crs                   ; show blinking cursor
  byte 0x00             ; program terminates here (program counter resets here)

win:                    ; if you've managed to win the game)
  pop                   ; fix up
  pop                   ; the stack pointer
  ldi 0x01              ; set register A to 0x01
  tab                   ; set register B to 0x01
  ldi 0x00              ; set register A to 0x00
  pos                   ; set cursor at (0, 1)
  ldi 0x00              ; reset A register
  tab                   ; reset B register

print_outro_win:        ; print outro_win string loop
  lda outro_win         ; load byte at 'outro_win' label address + B register offset
  cmp 0xfe              ; is it a zero terminating character?
  jmp idle              ; if so then exit the program
  dly 0x50              ; 80ms delay
  out                   ; print character to LCD display
  inc                   ; increment B reister by 1 to increase the offset
  lpc print_outro_win   ; jump to 'print_outro_win' label
  
idle:
  dly 0xff              ; wait
  dly 0xff              ; a little bit
  dly 0xff              ; before 'GAME OVER' is printed
  dly 0xff              ; to LCD after 'YOU WIN!'
  lpc reset_map         ; jump to 'reset_map' label

