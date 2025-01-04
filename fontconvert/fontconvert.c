//
// TrueType to bb_font converter
// Copyright (c) 2024 BitBank Software, inc.
// Written by Larry Bank
// August 31, 2024
// The bb_font format is a losslessly compressed bitmap font of a single point size
// The data is compressed with a compression scheme based on CCITT T.6
// The font structure includes overall size, per-character glyph info and then the
// compressed image data at the end.
// The font file format is designed to allow both dynamic loading of font data from
// external memory/disk or compiling the data as const into a progarm.
//
// Example usage:
// ./fontconvert <my_font.ttf> <out.bbf> <pt size> <start char> <end char>
// This code requires the freetype library
// found here: www.freetype.org
//
#include <ctype.h>
#include <ft2build.h>
#include <stdint.h>
#include <stdio.h>
#include FT_GLYPH_H
#include FT_MODULE_H
#include FT_TRUETYPE_DRIVER_H
#include "../src/g5enc.inl" // Group5 image compression library
G5ENCIMAGE g5enc; // Group5 encoder state

#define DPI 141 // Approximate resolution of common displays
#define OUTBUF_SIZE 65536
//
// Rotate a character 90/180/270 degrees
//
int RotateBitmap(int iAngle, uint8_t *pSrc, int iWidth, int iHeight, int iSrcPitch, uint8_t *pDst)
{
    int x, y, tx, ty, iDstPitch;
    uint8_t *s, *d, uc, ucMask;
    
    switch (iAngle) {
        case 90:
            iDstPitch = (iHeight+7)/8;
            for (y=0; y<iWidth; y++) {
                ucMask = 0x80 >> (y & 7);
                s = &pSrc[(y>>3) + (iHeight-1) * iSrcPitch]; // work from bottom up
                d = &pDst[y * iDstPitch];
                memset(d, 0, iDstPitch);
                for (x=0; x<iHeight; x++) {
                    if (s[0] & ucMask) {
                        d[x >> 3] |= (0x80 >> (x & 7));
                    }
                    s -= iSrcPitch;
                } // for x
            } // for y
            break;
        case 180:
            iDstPitch = iSrcPitch;
            for (y=0; y<iHeight; y++) {
                s = &pSrc[(iHeight-y-1) * iSrcPitch]; // work from bottom up
                d = &pDst[y * iDstPitch];
                memset(d, 0, iDstPitch);
                for (x=0; x<iWidth; x++) {
                    tx = (iWidth-x-1); // reverse x direction
                    // This code doesn't need to be efficient
                    uc = s[tx>>3] & (0x80 >> (tx & 7)); // source pixel
                    if (uc) {  // set
                        d[x >> 3] |= (0x80 >> (x & 7));
                    }
                } // for x
            } // for y
            break;
        case 270:
            iDstPitch = (iHeight+7)/8;
            for (y=0; y<iWidth; y++) {
                ty = (iWidth-1-y);
                ucMask = 0x80 >> (ty & 7);
                s = &pSrc[ty>>3]; // work from bottom up
                d = &pDst[y * iDstPitch];
                memset(d, 0, iDstPitch);
                for (x=0; x<iHeight; x++) {
                    if (s[0] & ucMask) {
                        d[x >> 3] |= (0x80 >> (x & 7));
                    }
                    s += iSrcPitch;
                } // for x
            } // for y
            break;
        default:
            return -1; // invalid
    }
    return iDstPitch;
} /* RotateBitmap() */
//
// Create the comments and const array boilerplate for the hex data bytes
//
void StartHexFile(FILE *f, int iLen, const char *fname)
{
    int i;
    char szTemp[256];
    fprintf(f, "//\n// Created with fontconvert, written by Larry Bank\n");
    fprintf(f, "// compressed font data size = %d bytes\n//\n", iLen);
    strcpy(szTemp, fname);
    i = strlen(szTemp);
    if (szTemp[i-2] == '.') szTemp[i-2] = 0; // get the leaf name for the data
    fprintf(f, "const uint8_t %s[] = {\n", szTemp);
} /* StartHexFile() */
//
// Add N bytes of hex data to the output
// The data will be arranged in rows of 16 bytes each
//
void AddHexBytes(FILE *f, void *pData, int iLen, int bLast)
{
    static int iCount = 0; // number of bytes processed so far
    int i;
    uint8_t *s = (uint8_t *)pData;
    for (i=0; i<iLen; i++) { // process the given data
        fprintf(f, "0x%02x", *s++);
        iCount++;
        if (i < iLen-1 || !bLast) fprintf(f, ",");
        if ((iCount & 15) == 0) fprintf(f, "\n"); // next row of 16
    }
    if (bLast) {
        fprintf(f, "};\n");
    }
} /* AddHexBytes() */

int main(int argc, char *argv[])
{
    int i, err, size, first = ' ', last = '~', y;
    int iLen ,iOffset = 0;
    int iRotation = 0;
    uint8_t *pBitmap, *pTemp;
    FILE *fOut;
    // TrueType library structures
    FT_Library library;
    FT_Face face;
    FT_Glyph glyph;
    FT_Bitmap *bitmap;
    FT_BitmapGlyphRec *g;
    // BitBank Font structures
    BB_GLYPH *pGlyphs;
    BB_FONT bbff;
    int bHFile; // flag indicating if the output will be a .H file of hex data
    
    if (argc < 4) {
        printf("Usage: %s <in.ttf> <out.bbf or out.h> point_size [first_char] [last_char] [rotation (90/180/270)]\n", argv[0]);
        return 1;
    }
    size = atoi(argv[3]);
    pTemp = (uint8_t *)argv[2] + strlen(argv[2]) - 1;
    bHFile = (pTemp[0] == 'H' || pTemp[0] == 'h'); // output an H file?

    if (argc == 5) {
        last = atoi(argv[4]); // only ending character was provided
    } else if (argc >= 6) {
        first = atoi(argv[4]); // start + end character codes were provided
        last = atoi(argv[5]);
    }
    if (argc == 7) { // rotation angle provided
        iRotation = atoi(argv[6]);
        if (iRotation != 90 && iRotation != 180 && iRotation != 270) {
            printf("Rotation angle can only be 90/180/270\n");
            return 1;
        }
    }
    if (last < first) {
        printf("Something went wrong - the starting character comes after the ending character. Try again...\n");
        return 1;
    }
    pGlyphs = (BB_GLYPH *)malloc((last - first + 1) * sizeof(BB_GLYPH));
    if (!pGlyphs) {
        printf("Error allocating memory for glyph data\n");
        return 1;
    }
    pBitmap = (uint8_t *)malloc(OUTBUF_SIZE); // Enough to hold the output
    pTemp = (uint8_t *)malloc(OUTBUF_SIZE); // for rotated bitmaps
    if (!pBitmap || ! pTemp) {
        printf("Error allocating memory for bitmap data\n");
        return 1;
    }
    
    // Init FreeType lib, load font
    if ((err = FT_Init_FreeType(&library))) {
        printf("FreeType init error: %d", err);
        return err;
    }
    // Print parameters
    printf("fontconvert %s to %s, size: %dpt, first: %d, last: %d, rotation: %d\n", argv[1], argv[2], size, first, last, iRotation);
    
    // Use TrueType engine version 35, without subpixel rendering.
    // This improves clarity of fonts since this library does not
    // support rendering multiple levels of gray in a glyph.
    // See https://github.com/adafruit/Adafruit-GFX-Library/issues/103
    FT_UInt interpreter_version = TT_INTERPRETER_VERSION_35;
    FT_Property_Set(library, "truetype", "interpreter-version",
                    &interpreter_version);
    
    if ((err = FT_New_Face(library, argv[1], 0, &face))) {
        printf("Font load error: %d", err);
        FT_Done_FreeType(library);
        return err;
    }
    
    // Shift the size left by 6 because the library uses '26dot6' fixed-point format
    FT_Set_Char_Size(face, size << 6, 0, DPI, 0);
    
    // Currently all symbols from 'first' to 'last' are processed.
    // Fonts may contain WAY more glyphs than that, but this code
    // will need to handle encoding stuff to deal with extracting
    // the right symbols, and that's not done yet.
    
    // Process glyphs and output huge bitmap data array
    for (i = first; i <= last; i++) {
        int index = i - first;
        uint8_t *s;
        int iPitch;
        // MONO renderer provides clean image with perfect crop
        // (no wasted pixels) via bitmap struct.
        if ((err = FT_Load_Char(face, i, FT_LOAD_TARGET_MONO))) {
            printf("Error %d loading char '%c'\n", err, i);
            continue;
        }
        
        if ((err = FT_Render_Glyph(face->glyph, FT_RENDER_MODE_MONO))) {
            printf("Error %d rendering char '%c'\n", err, i);
            continue;
        }
        
        if ((err = FT_Get_Glyph(face->glyph, &glyph))) {
            printf("Error %d getting glyph '%c'\n", err, i);
            continue;
        }
        
        bitmap = &face->glyph->bitmap;
        g = (FT_BitmapGlyphRec *)glyph;
        
        // Minimal font and per-glyph information is stored to
        // reduce flash space requirements.  Glyph bitmaps are
        // fully bit-packed; no per-scanline pad, though end of
        // each character may be padded to next byte boundary
        // when needed.  16-bit offset means 64K max for bitmaps,
        // code currently doesn't check for overflow.  (Doesn't
        // check that size & offsets are within bounds either for
        // that matter...please convert fonts responsibly.)
	
	// Check that the requested font size fits within the limits
        // of the 8-bit member variables
        if (bitmap->width > 255 || (face->glyph->advance.x>>6) > 255) {
            printf("The requested size is too large to fit in the BB_GLYPH member vars\nEither request a smaller size or change the member variables to be 16-bits\n");
            return -1;
        }
        pGlyphs[index].bitmapOffset = iOffset;
        pGlyphs[index].width = bitmap->width;
        pGlyphs[index].height = bitmap->rows;
        pGlyphs[index].xAdvance = face->glyph->advance.x >> 6;
        pGlyphs[index].xOffset = g->left;
        pGlyphs[index].yOffset = 1 - g->top;
        s = bitmap->buffer;
        iPitch = bitmap->pitch;
        if (iRotation != 0) {
            iPitch = RotateBitmap(iRotation, bitmap->buffer, bitmap->width, bitmap->rows, bitmap->pitch, pTemp);
            s = pTemp;
            g5_encode_init(&g5enc, bitmap->rows, bitmap->width, &pBitmap[iOffset], OUTBUF_SIZE-iOffset);
            for (y = 0; y < bitmap->width; y++) {
                g5_encode_encodeLine(&g5enc, &s[y * iPitch]);
            } // for y
        } else { // not rotated
            g5_encode_init(&g5enc, bitmap->width, bitmap->rows, &pBitmap[iOffset], OUTBUF_SIZE-iOffset);
            for (y = 0; y < bitmap->rows; y++) {
                g5_encode_encodeLine(&g5enc, &s[y * iPitch]);
            } // for y
        }
        iLen = g5_encode_getOutSize(&g5enc);
        iOffset += iLen;
        
        FT_Done_Glyph(glyph);
    } // for each glyph
    
    // Try to create the output file
    fOut = fopen(argv[2], "w+b");
    if (!fOut) {
        printf("Error creating output file: %s\n", argv[2]);
        return 1;
    }
    // Write the file header
    bbff.u16Marker = BB_FONT_MARKER;
    bbff.first = first;
    bbff.last = last;
    bbff.rotation = iRotation; // save rotation angle
    if (face->size->metrics.height == 0) {
        // No face height info, assume fixed width and get from a glyph.
        bbff.height = pGlyphs[0].height;
    } else {
        bbff.height = (face->size->metrics.height >> 6);
    }
    iLen = sizeof(bbff) + (last-first+1)*sizeof(BB_GLYPH) + iOffset;
    if (bHFile) { // create an H file of hex values
        StartHexFile(fOut, iLen, argv[2]);
        AddHexBytes(fOut, &bbff, sizeof(bbff), 0);
        AddHexBytes(fOut, pGlyphs, (last-first+1) * sizeof(BB_GLYPH), 0);
        AddHexBytes(fOut, pBitmap, iOffset, 1);
    } else {
        fwrite(&bbff, 1, sizeof(bbff), fOut);
        // Write the glyph table
        fwrite(pGlyphs, (last-first+1), sizeof(BB_GLYPH), fOut);
        // Write the compressed bitmap data
        fwrite(pBitmap, 1, iOffset, fOut);
    }
    fflush(fOut);
    fclose(fOut); // done!
    FT_Done_FreeType(library);
    printf("Success!\nFont file size: %d bytes\n", iLen);
    
    return 0;
} /* main() */
