#!/usr/bin/env python3
"""
BMP to bytes
用途
  BMPファイルからパレットと画像を得る
使用方法
  readBmp() を呼び出す
"""

def read256BmpFile(filename):
    with open(filename, 'rb') as f:
    # いわゆる '256bmp' 専用。それ以外での動作は未定義。
    # BMP file header
        f.read(2) #bfType         = f.read(2) # 今後エラーチェック追加時に利用したいときなど用。以降の類似したコメントアウトも同様
        f.read(4) #bfSize         = int.from_bytes(f.read(4), byteorder='little')
        f.read(2) #bfReserved1    = int.from_bytes(f.read(2), byteorder='little')
        f.read(2) #bfReserved2    = int.from_bytes(f.read(2), byteorder='little')
        f.read(4) #bfOffBits      = int.from_bytes(f.read(4), byteorder='little')

        # BMP information header
        f.read(4) #bcSize         = int.from_bytes(f.read(4), byteorder='little')
        bcWidth                   = int.from_bytes(f.read(4), byteorder='little')
        bcHeight                  = int.from_bytes(f.read(4), byteorder='little')
        f.read(2) #bcPlanes       = int.from_bytes(f.read(2), byteorder='little')
        f.read(2) #bcBitCount     = int.from_bytes(f.read(2), byteorder='little')
        f.read(4) #biCompression  = int.from_bytes(f.read(4), byteorder='little')
        f.read(4) #biSizeImage    = int.from_bytes(f.read(4), byteorder='little')
        f.read(4) #biXPixPerMeter = int.from_bytes(f.read(4), byteorder='little')
        f.read(4) #biYPixPerMeter = int.from_bytes(f.read(4), byteorder='little')
        biClrUsed                 = int.from_bytes(f.read(4), byteorder='little')
        f.read(4) #biCirImportant = int.from_bytes(f.read(4), byteorder='little')

        if not biClrUsed:
            biClrUsed = 256

        # color table
        colorTable = f.read(biClrUsed * 4)

        # pixels
        pixels = f.read()

    return (bcWidth, bcHeight, colorTable, pixels)

(width, height, colorTable, pixels) = read256BmpFile('temperature.bmp')
print(width, height, colorTable, pixels)
