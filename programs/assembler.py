###############################################
#
#           8-bit computer assembler
#
#                      by
#
#               Code Monkey King
#
###############################################

# packages
import sys
import json
import sys

# define opcodes
opcodes = {
    'NOP': 0x00,
    'LDI': 0x01,
    'LDA': 0x02,
    'TAB': 0x03,
    'ADD': 0x04,
    'SUB': 0x05,
    'STA': 0x06,
    'RCH': 0x07,
    'LPC': 0x08,
    'INC': 0x09,
    'DCR': 0x0a,
    'CMP': 0x0b,
    'JMP': 0x0c,
    'DBG': 0x0d,
    'IN' : 0x0e,
    'OUT': 0x0f,
    'BIT': 0x10,
    'AND': 0x11,
    'OR' : 0x12,
    'XOR': 0x13,
    'NOT': 0x14,
    'SHL': 0x15,
    'SHR': 0x16,
    'CLS': 0x17,
    'SDL': 0x18,
    'SDR': 0x19,
    'CRS': 0x1a,
    'NCR': 0x1b,
    'UDG': 0x1c,
    'SPR': 0x1d,
    'POS': 0x1e,
    'DLY': 0x1f,
    'RND': 0x20,
    'PSH': 0x21,
    'POP': 0x22,
    'SBR': 0x23,
    'RET': 0x24,
    'NUM': 0x25,
    'INM': 0x26,
    'DCM': 0x27,
    'SER': 0x28
}

# program labels
labels = {}

# check input file
if len(sys.argv) == 1:
    print('No input files!\nUsage: asm.py prog.asm')
    sys.exit(1)

# get input file name
filename = sys.argv[1];

# open asm source file
with open(filename) as input_file:
    # read source code
    src = [l.upper() for l in input_file.read().split('\n')]
    program = []
    byte_count = -1
    
    try: byte_count += int(sys.argv[2], 16)
    except: print('Origin should be a hexidecimal value, like 0x100')

    # init labels
    for line in src:
        if '%DEFINE' in line:
            if 'BYTE' in line.split(';')[0] or 'WORD' in line.split(';')[0]:
                print('"BYTE" or "WORD" can\'t be the part of the %define!')
                sys.exit(1)
            
            try:
                if int(line.split(';')[0].split()[1], 16):
                    print('Label "' + line.split(';')[0].split()[1] + '" would be treated as HEX value, please use a different name!')
                    sys.exit(1)

            except Exception as e:
                pass
            
            
            if len(line.split(';')[0].split()) and len(line.split(';')[0].split()) != 3:
                print('%define takes 2 args but ' + str(len(line.split())) + ' are given!');
                sys.exit(1)

            else:
                try:    
                    labels[line.split(';')[0].split()[1]] = line.split()[2].replace('X', 'x')
                    continue
                except:
                    pass

        if line != '':
            try:
                if ':' in line.split(';')[1]:
                    print('":" is not allowed in comments!')
                    sys.exit(1)
            
            except Exception as e:
                pass

            try:
                if line.strip()[0] == ';': continue
                byte_count += len(line.split(';')[0].split())

                try:
                    arg = line.split(';')[0].split()[1]
                    if '0x00' in arg and len(arg) == 6:
                        byte_count += 1
                    if arg[0].isalpha():
                        if line.split()[0] in ['LDA', 'STA', 'LPC', 'JMP', 'SBR', 'NUM', 'INM', 'DCM']:
                            byte_count += 1
                
                except:
                    pass
                
                if ':' in line:
                    if not line.split(';')[0].split(':')[1].strip() == '':
                        print('Instruction can\'t share the same line with label!')
                        sys.exit(1)

                    if 'BYTE' in line.split(';')[0].strip()[:-1] or 'WORD' in line.split(';')[0].strip()[:-1]:
                        print('"byte" and "word" are reserved keywords and can\'t be the part of a label!');
                        sys.exit(1)
                    
                    labels[line.split(';')[0].strip()[:-1]] = f'{byte_count:#0{6}x}'
                    byte_count -= 1
                
                if 'BYTE' in line.split(';')[0] or 'WORD' in line.split(';')[0]: byte_count -= 1
                
            except IndexError:
                pass

    # assemble code
    for line in src:
        if '%DEFINE' in line: continue
        if line != '':
            try:
                if ':' in line: continue
                if line.strip()[0] == ';': continue
                opcode = line.split()[0]
                
                try: program.append(opcodes[opcode])
                except:
                    if opcode != 'BYTE' and opcode != 'WORD':
                        print('Unknown opcode:', opcode);
                        sys.exit(1)
                  
                arg = line.split(';')[0].split()[1]
                
                if ('0X00') in arg and len(arg) == 6:
                    program.append(0);
                    
                
                try:
                    if arg[0] == "'" and arg[-1] == "'": print(arg[1])
                    value = int(arg, 16)

                except:
                    try:
                        if ('0x00') in labels[arg] and len(labels[arg]) == 6:
                            if opcode in ['LDA', 'STA', 'LPC', 'JMP', 'SBR', 'NUM', 'INM', 'DCM']:
                                program.append(0);

                        value = int(labels[arg.split(';')[0].strip()], 16)

                    except Exception as e:
                        print('Label "' + arg + '" doesn\'t exist!');
                        sys.exit(1)

                if value > 0xff:
                    program.append(value >> 8)
                    program.append(value & 0x00FF)
                
                else:
                    program.append(value)

            except IndexError:
                pass

if (len(program) > 1500):
    print('Your program exceeds limit by', len(program) - 1500, 'bytes!');
    sys.exit();

print('\nYOUR PROGRAM LABELS:')
print(json.dumps(labels, indent=2))
print('\nYOUR PROGRAM BYTES:');
print(', '.join([f'{i:#0{4}x}' for i in  program]))

print('\nYOUR PROGRAM LENGTH:')
print(len(program), 'bytes')
#print('\n1. Type "FFFD" on CMK computer to load the program');
#print('2. Open Arduino IDE => Serial Monitor')
#print('3. Copy and paste above bytes to Arduino Serial Monitor ans click "send"');
#print('4. Type "FFFF" on CMK computer to run the program');
print('\nEnjoy epic 8-bit computing experience!\n');
