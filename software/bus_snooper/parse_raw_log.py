'''
parse_raw_log.py - brute force parser for raw bus traffic from faraday bike.

the faraday bus uses a serial BREAK which is not supported by pyserial, making real-time 
logging tricky with standard RS485 adapters. This script makes some basic assumptions about
the packet size, then looks for valid packets which match the CRC. Known packets are printed.

To use:

1. Use putty or other serial terminal to log ALL bus traffic. The Faraday uses 115200/8N1 on with differential drivers.
2. > python parse.py <file_path> -v

'''

import sys

# globals
library = {}
verbose = False
prev_was_req = False
prev_req = object()
progress = 0
progress_rounded = 0

def modbusCrc(msg):
    crc = 0xFFFF
    for n in range(len(msg)):
        crc ^= msg[n]
        for i in range(8):
            if crc & 1:
                crc >>= 1
                crc ^= 0xA001
            else:
                crc >>= 1
    return crc.to_bytes(2,byteorder='little')

def add_to_library(packet,prev_was_req):
    
    # determine if request or response
    was_req = False
    global prev_req
    global library

    # it's a response
    if len(packet) == 5+packet[2]:
        if (prev_was_req):
            idx = sum(packet)
            library[idx] = {'req':prev_req,'resp':packet}
            was_req = False
        
    # it's a request 
    else:
        was_req = True
        prev_req = packet
    return was_req

def parse(data,max_characters):
    marker = 0
    tries = 0
    prev_was_req = False

    global progress
    global progress_rounded
    global verbose

    while True:
        if tries > max_characters:
            break

        if not verbose:
            progress = marker/min(len(data),max_characters)*100
            if round(progress,0) > progress_rounded:
                print('Parsing...%d%%'% progress_rounded)
                progress_rounded = round(progress,0) 

        for n in range(6,37): #shortest message is 6, longest is 29
            start = marker
            end = marker + n
            if end >= len(data)-1:
                end = len(data)-1
            packet = data[start:end]

            if len(packet) < 6:
                break

            crc_packet = (packet[len(packet)-2] << 8) + packet[len(packet)-1]
            
            crc_calculated = int.from_bytes(modbusCrc(packet[:len(packet)-2]),byteorder='big',signed = False)
            if crc_packet == crc_calculated and packet[-1] != 0x00:
                if verbose:
                    print_packet(packet)
                
                prev_was_req = add_to_library(packet,prev_was_req)

        marker += 1;
        tries+=1
        if marker > len(data):
            break

def print_packet(packet, mute = 'SOC'):
#def print_packet(packet, mute = None):

    # battery messages
    bat_soc_request = [0x02,0x3,0x0,0x02,0x0,0xC,0xE4,0x3c]
    bat_bal_request = [0x02,0x3,0x0,0x17,0x0,0x01,0x34,0x3d]
    bat_status_req_1 = [0x02,0x3,0x0, 0x0, 0x0, 0x1, 0x84, 0x39]
    bat_status_req_2 = [0x02, 0x3, 0x0, 0x1, 0x0, 0x1, 0xd5, 0xf9]
    #bat_write_balance = []

    # mode selector messages
    mode_status_req_1 = [0x3, 0x3, 0x0, 0x0, 0x0, 0x5, 0x84, 0x2b]
    mode_status_req_2 = [0x3, 0x3, 0x0, 0x2, 0x0, 0x1, 0x24, 0x28]
    mode_status_req_3 = [0x3, 0x3, 0x0, 0x0, 0x0, 0x3, 0x4, 0x29]  
    mode_write_req_1 = [0x3,0x10, 0x0, 0x2, 0x0, 0x1, 0x2]
    mode_write_resp_1 = [0x3, 0x10, 0x0, 0x2, 0x0, 0x1, 0xa1, 0xeb]

    if packet[0] != 0x02:
        return

    if packet[2] == 0x18 and mute == 'SOC':
        return

    if packet == bat_soc_request and mute == 'SOC':
        return

    if packet == bat_soc_request:
        print("bat soc request-->")
    elif packet == bat_bal_request:
        print("bat bal request-->")
    elif packet == bat_status_req_1:
        print("bat_status_req_1-->")
    elif packet == bat_status_req_2:
        print("bat_status_req_2-->")
    elif packet == mode_status_req_1:
        print("modsel_req_1-->")
    elif packet == mode_status_req_2:
        print("modsel_req_2-->")
    elif packet == mode_status_req_3:
        print("modsel_req_3-->")
    elif mode_write_req_1 in packet :
        print("modsel_write_1 -->",(' '.join(format(x, '#x') for x in packet[6:7])))
    elif mode_write_resp_1 in packet :
        print("modsel_write_1_resp-->",(' '.join(format(x, '#x') for x in packet)))

    elif packet[2] == 0x18:
        print("bat soc reponse")
    else:
        desc = ""

        if packet[0] == 0x02:
            desc += 'bat '

        if packet[0] == 0x03:
            desc += 'modsel'

        if packet[1] == 0x03:
            desc += " read "

        if packet[1] == 0x10:
            desc += " write "

        print(desc,(' '.join(format(x, '#x') for x in packet)),"\n")


def parse_faraday_RS485_dump(file_path):

    with open(file_path, 'rb') as fp:
        # Read the file content
        log = list()
        byte = fp.read(1)
        while byte != b"":
            # Do stuff with byte.
            byte = fp.read(1)
            log.append(byte)

        # convert to ints
        int_log = [int.from_bytes(k,byteorder='little',signed=False) for k in log]
        parse(int_log, 1000000)


        # print summary
        print('All message types\n======================\n')
        for pair in library.values():
            req = pair['req']
            resp = pair['resp']
            print('req',(' '.join(format(x, '#x') for x in req)),'\nresp',(' '.join(format(x, '#x') for x in resp)),"\n\n")
        
def main():
    global verbose
    if len(sys.argv) < 2:
        print("Usage: python parse.py <file_path> -v")
        sys.exit(1)

    if '-v' in sys.argv:
        print('Printing all messages')
        verbose = True

    file_path = sys.argv[1]
    byte_array = parse_faraday_RS485_dump(file_path)

if __name__ == "__main__":
    main()
