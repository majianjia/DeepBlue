//八位二进制转十六进制的宏

#ifndef __BIN_H__
#define __BIN_H__

#define Bin00000000 0x00
#define Bin00000001 0x01
#define Bin00000010 0x02
#define Bin00000011 0x03
#define Bin00000100 0x04
#define Bin00000101 0x05
#define Bin00000110 0x06
#define Bin00000111 0x07
#define Bin00001000 0x08
#define Bin00001001 0x09
#define Bin00001010 0x0A
#define Bin00001011 0x0B
#define Bin00001100 0x0C
#define Bin00001101 0x0D
#define Bin00001110 0x0E
#define Bin00001111 0x0F
#define Bin00010000 0x10
#define Bin00010001 0x11
#define Bin00010010 0x12
#define Bin00010011 0x13
#define Bin00010100 0x14
#define Bin00010101 0x15
#define Bin00010110 0x16
#define Bin00010111 0x17
#define Bin00011000 0x18
#define Bin00011001 0x19
#define Bin00011010 0x1A
#define Bin00011011 0x1B
#define Bin00011100 0x1C
#define Bin00011101 0x1D
#define Bin00011110 0x1E
#define Bin00011111 0x1F
#define Bin00100000 0x20
#define Bin00100001 0x21
#define Bin00100010 0x22
#define Bin00100011 0x23
#define Bin00100100 0x24
#define Bin00100101 0x25
#define Bin00100110 0x26
#define Bin00100111 0x27
#define Bin00101000 0x28
#define Bin00101001 0x29
#define Bin00101010 0x2A
#define Bin00101011 0x2B
#define Bin00101100 0x2C
#define Bin00101101 0x2D
#define Bin00101110 0x2E
#define Bin00101111 0x2F
#define Bin00110000 0x30
#define Bin00110001 0x31
#define Bin00110010 0x32
#define Bin00110011 0x33
#define Bin00110100 0x34
#define Bin00110101 0x35
#define Bin00110110 0x36
#define Bin00110111 0x37
#define Bin00111000 0x38
#define Bin00111001 0x39
#define Bin00111010 0x3A
#define Bin00111011 0x3B
#define Bin00111100 0x3C
#define Bin00111101 0x3D
#define Bin00111110 0x3E
#define Bin00111111 0x3F
#define Bin01000000 0x40
#define Bin01000001 0x41
#define Bin01000010 0x42
#define Bin01000011 0x43
#define Bin01000100 0x44
#define Bin01000101 0x45
#define Bin01000110 0x46
#define Bin01000111 0x47
#define Bin01001000 0x48
#define Bin01001001 0x49
#define Bin01001010 0x4A
#define Bin01001011 0x4B
#define Bin01001100 0x4C
#define Bin01001101 0x4D
#define Bin01001110 0x4E
#define Bin01001111 0x4F
#define Bin01010000 0x50
#define Bin01010001 0x51
#define Bin01010010 0x52
#define Bin01010011 0x53
#define Bin01010100 0x54
#define Bin01010101 0x55
#define Bin01010110 0x56
#define Bin01010111 0x57
#define Bin01011000 0x58
#define Bin01011001 0x59
#define Bin01011010 0x5A
#define Bin01011011 0x5B
#define Bin01011100 0x5C
#define Bin01011101 0x5D
#define Bin01011110 0x5E
#define Bin01011111 0x5F
#define Bin01100000 0x60
#define Bin01100001 0x61
#define Bin01100010 0x62
#define Bin01100011 0x63
#define Bin01100100 0x64
#define Bin01100101 0x65
#define Bin01100110 0x66
#define Bin01100111 0x67
#define Bin01101000 0x68
#define Bin01101001 0x69
#define Bin01101010 0x6A
#define Bin01101011 0x6B
#define Bin01101100 0x6C
#define Bin01101101 0x6D
#define Bin01101110 0x6E
#define Bin01101111 0x6F
#define Bin01110000 0x70
#define Bin01110001 0x71
#define Bin01110010 0x72
#define Bin01110011 0x73
#define Bin01110100 0x74
#define Bin01110101 0x75
#define Bin01110110 0x76
#define Bin01110111 0x77
#define Bin01111000 0x78
#define Bin01111001 0x79
#define Bin01111010 0x7A
#define Bin01111011 0x7B
#define Bin01111100 0x7C
#define Bin01111101 0x7D
#define Bin01111110 0x7E
#define Bin01111111 0x7F
#define Bin10000000 0x80
#define Bin10000001 0x81
#define Bin10000010 0x82
#define Bin10000011 0x83
#define Bin10000100 0x84
#define Bin10000101 0x85
#define Bin10000110 0x86
#define Bin10000111 0x87
#define Bin10001000 0x88
#define Bin10001001 0x89
#define Bin10001010 0x8A
#define Bin10001011 0x8B
#define Bin10001100 0x8C
#define Bin10001101 0x8D
#define Bin10001110 0x8E
#define Bin10001111 0x8F
#define Bin10010000 0x90
#define Bin10010001 0x91
#define Bin10010010 0x92
#define Bin10010011 0x93
#define Bin10010100 0x94
#define Bin10010101 0x95
#define Bin10010110 0x96
#define Bin10010111 0x97
#define Bin10011000 0x98
#define Bin10011001 0x99
#define Bin10011010 0x9A
#define Bin10011011 0x9B
#define Bin10011100 0x9C
#define Bin10011101 0x9D
#define Bin10011110 0x9E
#define Bin10011111 0x9F
#define Bin10100000 0xA0
#define Bin10100001 0xA1
#define Bin10100010 0xA2
#define Bin10100011 0xA3
#define Bin10100100 0xA4
#define Bin10100101 0xA5
#define Bin10100110 0xA6
#define Bin10100111 0xA7
#define Bin10101000 0xA8
#define Bin10101001 0xA9
#define Bin10101010 0xAA
#define Bin10101011 0xAB
#define Bin10101100 0xAC
#define Bin10101101 0xAD
#define Bin10101110 0xAE
#define Bin10101111 0xAF
#define Bin10110000 0xB0
#define Bin10110001 0xB1
#define Bin10110010 0xB2
#define Bin10110011 0xB3
#define Bin10110100 0xB4
#define Bin10110101 0xB5
#define Bin10110110 0xB6
#define Bin10110111 0xB7
#define Bin10111000 0xB8
#define Bin10111001 0xB9
#define Bin10111010 0xBA
#define Bin10111011 0xBB
#define Bin10111100 0xBC
#define Bin10111101 0xBD
#define Bin10111110 0xBE
#define Bin10111111 0xBF
#define Bin11000000 0xC0
#define Bin11000001 0xC1
#define Bin11000010 0xC2
#define Bin11000011 0xC3
#define Bin11000100 0xC4
#define Bin11000101 0xC5
#define Bin11000110 0xC6
#define Bin11000111 0xC7
#define Bin11001000 0xC8
#define Bin11001001 0xC9
#define Bin11001010 0xCA
#define Bin11001011 0xCB
#define Bin11001100 0xCC
#define Bin11001101 0xCD
#define Bin11001110 0xCE
#define Bin11001111 0xCF
#define Bin11010000 0xD0
#define Bin11010001 0xD1
#define Bin11010010 0xD2
#define Bin11010011 0xD3
#define Bin11010100 0xD4
#define Bin11010101 0xD5
#define Bin11010110 0xD6
#define Bin11010111 0xD7
#define Bin11011000 0xD8
#define Bin11011001 0xD9
#define Bin11011010 0xDA
#define Bin11011011 0xDB
#define Bin11011100 0xDC
#define Bin11011101 0xDD
#define Bin11011110 0xDE
#define Bin11011111 0xDF
#define Bin11100000 0xE0
#define Bin11100001 0xE1
#define Bin11100010 0xE2
#define Bin11100011 0xE3
#define Bin11100100 0xE4
#define Bin11100101 0xE5
#define Bin11100110 0xE6
#define Bin11100111 0xE7
#define Bin11101000 0xE8
#define Bin11101001 0xE9
#define Bin11101010 0xEA
#define Bin11101011 0xEB
#define Bin11101100 0xEC
#define Bin11101101 0xED
#define Bin11101110 0xEE
#define Bin11101111 0xEF
#define Bin11110000 0xF0
#define Bin11110001 0xF1
#define Bin11110010 0xF2
#define Bin11110011 0xF3
#define Bin11110100 0xF4
#define Bin11110101 0xF5
#define Bin11110110 0xF6
#define Bin11110111 0xF7
#define Bin11111000 0xF8
#define Bin11111001 0xF9
#define Bin11111010 0xFA
#define Bin11111011 0xFB
#define Bin11111100 0xFC
#define Bin11111101 0xFD
#define Bin11111110 0xFE
#define Bin11111111 0xFF

#endif 
