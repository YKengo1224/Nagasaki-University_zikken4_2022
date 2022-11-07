fin = open("sim_risc16b.mnem","rt")
fout = open("sim_risc16b.mem","wt")
text = ''

line = fin.readlines()
fin.close()

num = 0
list_00000 = {"nop":'00000',"mov":'00001',"not":'00010',"xor":'00011',"add":'00100',"sub":'00101',"sl8":'00110',"sr8":'00111',"sl":'01000',"sr":"01001","and":'01010',"or":'01011',"andf0":'01100',"and0f":'01101',"sl4":'01110',"No":'01111',"sw":'10000',"lw":'10001',"sbu":'10010',"lbu":'10011'}

list_xx = {"addi":'00100','andi':'01010','ori':'01011','lli':'00001','lui':'00110','xori':'00011','muli':'01100','divi':'01101','modi':'01110','beqz':'10000','bnez':'10001','bmi':'10010','bpl':'10011','bna2':'11001','bna4':'11010','bna8':'11011','bnzm':'11100'}

register = {"r0":'000',"r1":'001',"r2":'010','r3':'011','r4':'100','r5':'101','r6':'110','r7':'111'}
for i in range(len(line)):
    if line[i] == '\n':
        break
    if '//' in line[i]:
        continue

    code1 = ''
    code2 = ''
    num_text = format(num,'x')
    text += '@'+num_text.zfill(4)+' '
    line_sp = line[i].split()
    if line_sp[0] in list_00000:
        code1 += '00000'
        if line_sp[2] in register:
            code2 += register[line_sp[2]] + list_00000[line_sp[0]]
        else:
            print('error',i)
            break
        
        if line_sp[1] in register:
            code1 += register[line_sp[1]]
        else:
            print("error",i)
            break        

    elif line_sp[0] in list_xx:
        code1 += list_xx[line_sp[0]]
        
        try:
            if '0x' in line_sp[2]:
                baf = int(line_sp[2],16)
            else:
                baf = int(line_sp[2])
            
            if baf >= 0:
                inn = format(baf,'b')
                code2 += inn.zfill(8)
            else:
                inn = format(baf&0b11111111,'b')
                code2 += inn
        
        except Exception as e:
            print(e)
            print(i)
            break
        
        
        if line_sp[1] in register:
            code1 += register[line_sp[1]]
        else:
            print('error',i)
            break
        
    elif line_sp[0] == 'j':
        code1 += '11000000'
    
        
        try:
            if '0x' in line_sp[1]:
                baf = int(line_sp[1],16)
            else:
                baf = int(line_sp[1],10)
            if baf >= 0:
                inn = format(baf,'b')
                code2 += inn.zfill(8)
            else:
                inn = format(baf&0b11111111,'b')
                code2 += inn

        except Exception as e:
            print(e)
            print(i,"hoge")
            break

    
    else:
        print("error",i)
        break


    text += code1+' '+code2+' //'+line[i]
    num+=2


fout.write(text)
fout.close()

