//
// C++ example for FastEPD library
// written by Larry Bank (bitbank@pobox.com)
// Project started 10/15/2025
// Copyright (c) 2025 BitBank Software, Inc.
//
// SPDX-License-Identifier: Apache-2.0
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//    http://www.apache.org/licenses/LICENSE-2.0
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//===========================================================================
//
#include <FastEPD.h>
#include <PNGdec.h>
#include <JPEGDEC.h>
#include "cJSON.h"
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <dirent.h>
#include <linux/fb.h>
#include <sys/mman.h>
#include <sys/ioctl.h>
#define SHOW_DETAILS

FASTEPD bbep;
int iPanel1Bit, iMode;
int iInvert = 0; // assume not inverted
int iBGR = 0; // reversed R/B order
int iStretch = -1;

enum {
	STRETCH_NONE = 0,
	STRETCH_FILL,
	STRETCH_ASPECTFILL
};
const char *szModes[] = {"full", "fast", "partial", NULL};
const char *szStretch[] = {"none", "fill", "aspectfill", NULL};
const char *szPanels[] = {
    "BBEP_DISPLAY_EC060TC1",
    "BBEP_DISPLAY_EC060KD1",
    "BBEP_DISPLAY_ED0970TC1",
    "BBEP_DISPLAY_ED103TC2",
    "BBEP_DISPLAY_ED052TC4",
    "BBEP_DISPLAY_ED1150C1",
    NULL // must be last entry
};

PNG png;
JPEGDEC jpg;
int iWidth, iHeight, iBpp, iPixelType;
uint8_t *pBitmap, *pPalette=NULL;

const char *szPNGErrors[] = {"Success", "Invalid Parameter", "Decoding", "Out of memory", "No buffer allocated", "Unsupported feature", "Invalid file", "Too big", "Quit early"};
const char *szJPEGErrors[] = {"Success", "Invalid Parameter", "Decoding", "Unsupported feature", "Invalid file", "Out of memory"};

//
// Find the index value of a string within a list
// The list must be terminated with a NULL pointer
// Returns a value 0-N or -1 for not found
//
int FindItemName(const char **pList, const char *pName, const char *szLabel)
{
int i = 0;
    while (pList[i] != NULL && strcasecmp(pName, pList[i]) != 0) {
	    i++;
    }
    if (pList[i] == NULL) {
        printf("Invalid %s; must be one of: ", szLabel); 
        // Print the list of valid values
        i = 0;
        while (pList[i] != NULL) {
            printf("%s, ", pList[i]);
            i++; 
        }
        printf("\b\b  \n"); // erase the last comma
        return -1; // not found
    }
    return i;
} /* FindItemName() */
//
// The user passed a file which has 2 or more bits per pixel
// convert it to 1 or 2-bpp grayscale
//
int ConvertBpp(uint8_t *pBMP, int w, int h, int iBpp, uint8_t *palette)
{
    int gray, r=0, g=0, b=0, x, y, iDelta, iPitch, iDestPitch, iDestBpp;
    uint8_t *s, *d, *pPal, u8, count;

//    if (iPanel2Bit == -1) { // only 1 bit panel available
        iDestBpp = 1;
//    } else {
//        iDestBpp = 2;
//    }
    if (iDestBpp == 1) {
        iDestPitch = (w+7)/8;
    } else {
        iDestPitch = (w+3)/4;
    }
    // The bits per pixel info from PNG files is per color channel
    // Convert the value into a true bits per pixel
    switch (iPixelType) {
        case PNG_PIXEL_INDEXED:
            break;
        case PNG_PIXEL_TRUECOLOR:
	    if (iBpp <= 8) {
                iBpp *= 3;
	    }
            palette = NULL;
            break;
        case PNG_PIXEL_TRUECOLOR_ALPHA:
	    if (iBpp <= 8) {
                iBpp *= 4;
	    }
            palette = NULL;
            break;
        case PNG_PIXEL_GRAYSCALE:
            palette = NULL;
            break;
    } // switch on pixel type
    
    // Loop through the source image and convert each pixel to 2-bit grayscale
    // Overwrite the source image with the converted image since it will be smaller or
    // equal in size to the original. This is needed even for 2-bit images which may
    // use a palette with random color entries.
    iPitch = (w * iBpp)/8;    
    iDelta = iBpp/8;
    for (y=0; y<h; y++) {
        s = &pBMP[iPitch * y];
        d = &pBMP[iDestPitch * y]; // overwrite the original data as we change it
        count = 8; // bits in a byte
        u8 = 0; // start with all black
        for (x=0; x<w; x++) { // slower code, but less code :)
            u8 <<= iDestBpp;
            switch (iBpp) {
                case 24:
                case 32:
                    r = s[0];
                    g = s[1];
                    b = s[2];
                    s += iDelta;
                    break;
                case 16:
                    r = s[1] & 0xf8; // red
                    g = ((s[0] | s[1] << 8) >> 3) & 0xfc; // green
                    b = s[0] << 3;
                    s += 2;
                    break;
                case 8:
                    if (palette) {
                        pPal = &palette[s[0] * 3];
                        r = pPal[0];
                        g = pPal[1];
                        b = pPal[2];
                    } else {
                        r = g = b = s[0];
                    }
                    s++;
                    break;
                case 4:
                    if (palette) {
                        if (x & 1) {
                            pPal = &palette[(s[0] & 0xf) * 3];
                            s++;
                        } else {
                            pPal = &palette[(s[0]>>4) * 3];
                        }
                        r = pPal[0];
                        g = pPal[1];
                        b = pPal[2];
                    } else {
                        if (x & 1) {
                           r = g = b = (s[0] & 0xf) | (s[0] << 4);
                           s++;
                        } else {
                            r = g = b = (s[0] >> 4) | (s[0] & 0xf0);
                        }
                    }
                    break;
		case 2:
		    if (palette) {
			pPal = &palette[(s[0] >> ((3-(x&3))*2)) & 3];
			r = pPal[0]; g = pPal[1]; b = pPal[2];
		    } else {
			r = g = b = (s[0] << ((x&3)*2)) & 0xc0;
		    }
		    if ((x & 3) == 3) s++;
		    break;
            } // switch on bpp
            // Convert the source rgb into gray with a simple formula which favors green
            gray = (r + g*2 + b)/4;
            u8 |= gray >> (8-iDestBpp); // pack 1 or 2 bit gray pixels into a destination byte
            count -= iDestBpp;
            if (count == 0) { // byte is full, store it and prepare the next
                *d++ = u8;
                u8 = 0;
                count = 8;
            }
        } // for x
        if (count != 8) {
            *d++ = (u8 << count); // store last partial byte
        }
    } // for y
    return iDestBpp;
} /* ConvertBpp() */
//
// Decode the BMP file
//
int DecodeBMP(uint8_t *pData, int iSize)
{
    int iOffBits; // offset to bitmap data
    int y, iDestPitch=0, iPitch;
    uint8_t bFlipped = 0;
    uint8_t *s, *d;

    iWidth = *(int16_t *)&pData[18];
    iHeight = *(int16_t *)&pData[22];
    if (iHeight < 0) {
        iHeight = -iHeight;
    } else {
	bFlipped = 1;
    }
    iBpp = *(int16_t *)&pData[28];
    iOffBits = *(uint16_t *)&pData[10];
    switch (iBpp) {
        case 1:
	    iDestPitch = ((iWidth+7)>>3);
	    iPixelType = PNG_PIXEL_INDEXED;
            break;
	case 4:
	    iDestPitch = ((iWidth+1)>>1);
	    iPixelType = PNG_PIXEL_INDEXED;
	    break;
	case 8:
	    iDestPitch = iWidth;
	    iPixelType = PNG_PIXEL_INDEXED;
	    break;
	case 24:
	    iDestPitch = iWidth*3;
	    iPixelType = PNG_PIXEL_TRUECOLOR;
	    iBGR = 1; // reversed R/B order
	    break;
	case 32:
	    iDestPitch = iWidth*4;
	    iPixelType = PNG_PIXEL_TRUECOLOR_ALPHA;
	    iBGR = 1;
	    break;
    } // switch on bpp
    iPitch = (iDestPitch + 3) & 0xfffc; // must be DWORD aligned
    if (bFlipped)
    {
        iOffBits += ((iHeight-1) * iPitch); // start from bottom
        iPitch = -iPitch;
    }
    pBitmap = (uint8_t *)malloc(iHeight * iDestPitch);
    s = &pData[iOffBits];
    d = pBitmap;
    for (y=0; y<iHeight; y++) { // copy the bitmap to the common format
        memcpy(d, s, iDestPitch);
	s += iPitch;
	d += iDestPitch;
    }
    // Adjust the palette for 3-byte entries (if there is one)
    if (iBpp <= 8) {
	int iColors = 1<<iBpp;
	d = pPalette = pData;
        iOffBits = *(uint16_t *)&pData[10];
        s = &pData[iOffBits - (4 * iColors)];
        for (y=0; y<iColors; y++) {
            d[0] = s[0]; d[1] = s[1]; d[2] = s[2];
	    s += 4;
	    d += 3;
	}
    } else {
        pPalette = NULL;
    }
    return PNG_SUCCESS; // re-use this return code
} /* DecodeBMP() */
//
// Decode the JPEG file into an uncompressed bitmap
//
int DecodeJPEG(uint8_t *pData, int iSize)
{
int rc, iPitch;
    rc = jpg.openRAM(pData, iSize, NULL);
    if (!rc) {
        rc = jpg.getLastError();
        printf("JPEG open returned error: %s\n", szJPEGErrors[rc]);
        return -1; // only show the error once
    }
    if (jpg.getWidth() > bbep.width() || jpg.getHeight() > bbep.height()) {
        printf("Requested image is too large for the EPD.\n");
        printf("Image Size: %d x %d\nEPD size: %d x %d\n", png.getWidth(), png.getHeight(), bbep.width(), bbep.height());
        return -1;
    }
    iWidth = jpg.getWidth();
    iHeight = jpg.getHeight();
    iBpp = jpg.getBpp();
    if (iBpp == 8) {
        iPixelType = PNG_PIXEL_GRAYSCALE;
        jpg.setPixelType(EIGHT_BIT_GRAYSCALE);
        iPitch = iWidth;
    } else {
        iPixelType = PNG_PIXEL_TRUECOLOR_ALPHA;
        jpg.setPixelType(RGB8888);
        iPitch = iWidth*4;
        iBpp = 32; // output is 32-bits
    }
    pBitmap = (uint8_t *)malloc(iPitch * (iHeight+15));
    jpg.setFramebuffer(pBitmap);
    jpg.decode(0, 0, 0);
    return jpg.getLastError();
} /* DecodeJPEG() */

//
// Decode the PNG file into an uncompressed bitmap
//
int DecodePNG(uint8_t *pData, int iSize)
{
int rc;
    rc = png.openRAM(pData, iSize, NULL);
    if (rc != PNG_SUCCESS) {
        printf("PNG open returned error: %s\n", szPNGErrors[rc]);
        return -1; // only show the error once
    }
    if (png.getWidth() > bbep.width() || png.getHeight() > bbep.height()) {
        printf("Requested image is too large for the EPD.\n");
        printf("Image Size: %d x %d\nEPD size: %d x %d\n", png.getWidth(), png.getHeight(), bbep.width(), bbep.height());
        return -1;
    }
    iWidth = png.getWidth();
    iHeight = png.getHeight();
    iBpp = png.getBpp();
    pPalette = png.getPalette();
    iPixelType = png.getPixelType();
    if (iPixelType != PNG_PIXEL_INDEXED) pPalette = NULL; // tell other code that there's no palette present
    pBitmap = (uint8_t *)malloc(png.getBufferSize());
    png.setBuffer(pBitmap);
    rc = png.decode(NULL, 0);
    return rc;
} /* DecodePNG() */
//
// Prepare the decoded image for the internal memory layout of the EPD
// For 1-bit images, just memcpy since PNG uses the same layout.
// For 2-bit images, the EPD splits the memory into 2x 1-bit planes.
void PrepareImage(void)
{
	int x, y, iPlaneOffset, iSrcPitch, iDestPitch;
	uint8_t *s, *d, uc=0;
	s = pBitmap;
	d = (uint8_t *)bbep.currentBuffer();
	iDestPitch = (bbep.width()+7)/8;
        // Convert the source bitmap to 1 or 2-bit grayscale
        if (iBpp >= 2) {
            iBpp = ConvertBpp(s, iWidth, iHeight, iBpp, pPalette);
        }
        iSrcPitch = (iWidth+7)/8;
        for (y=0; y<iHeight; y++) {
                memcpy(d, s, iSrcPitch);
	        if (iWidth & 7) { // fill partial byte with white
                     d[iWidth>>3] |= (0xff >> (iWidth & 7));
	        }
		s += iSrcPitch;
		d += iDestPitch;
        }
	free(pBitmap);
} /* PrepareImage() */

void ShowHelp(void)
{
    printf("show_img utility - display PNG (and BMP) images on ePaper displays\nwritten by Larry Bank (bitbank@pobox.com)\nCopyright(c) 2025 BitBank Software, inc.\n");
    printf("A JSON file (~/.config/trmnl/show_img.json) can contain the setup\nor the parameters can be passed on the command line (in any order):\n");
    printf("file=<filename> any PNG or BMP file\nmode=<update mode> can be full, fast or partial\nadapter=<epaper PCB> can be waveshare_2 or pimoroni\npanel_1bit=<bb_epaper panel name>\npanel_2bit=<bb_epaper panel name>\n");
    printf("Color images and bit depths greater than 2-bpp will be\nautomatically converted to 2-bit (4 grays).\n");
    printf("example: ./show_img file=\"/home/me/test.png\" mode=fast panel_1bit=EP75_800x480 adapter=waveshare_2\n");
} /* ShowHelp() */
//
// Main program entry point
//
int main(int argc, const char * argv[]) {
int rc, iSize;
FILE *ihandle;
uint8_t *pData;
char szJSON[256]; // current dir
cJSON *pJSON, *pItem;
char szFile[256];

    iPanel1Bit = iMode = -1;
    szFile[0] = 0;

    pData = (uint8_t*)getenv("HOME"); // try to get the home directory
    if (pData) {
         strcpy(szJSON, (const char *)pData);
    }
    strcat(szJSON, "/.config/trmnl/show_img.json"); // name of local config file
    //printf("config name: %s\n", szJSON);
    ihandle = fopen(szJSON, "r+b");
    if (ihandle) {
#ifdef SHOW_DETAILS
	    printf("show_img.json found!\n");
#endif
	    fseek(ihandle, 0, SEEK_END);
            iSize = (int)ftell(ihandle);
	    fseek(ihandle, 0, SEEK_SET);
	    pData = (uint8_t *)malloc(iSize);
	    rc = fread(pData, 1, iSize, ihandle);
	    if (rc != iSize) {
		    printf("Error reading file!\n");
		    fclose(ihandle);
		    free(pData);
		    return -1;
	    }
	    fclose(ihandle);
	    pJSON = cJSON_ParseWithLength((const char *)pData, iSize);
	    if (pJSON) {
#ifdef SHOW_DETAILS
		    printf("JSON parsed successfully!\n");
#endif
		    if (cJSON_HasObjectItem(pJSON, "stretch")) {
			 pItem = cJSON_GetObjectItem(pJSON, "stretch");
			 iStretch = FindItemName(szStretch, pItem->valuestring, "stretch");
			 if (iStretch >= 0) {
#ifdef SHOW_DETAILS
				 printf("stretch = %s\n", szStretch[iStretch]);
#endif
			 }
		    }
                    if (cJSON_HasObjectItem(pJSON, "invert")) {
                         pItem = cJSON_GetObjectItem(pJSON, "invert");
                         iInvert = !strcmp(pItem->valuestring, "true");
#ifdef SHOW_DETAILS
                         printf("invert = %s\n", (iInvert) ? "true" : "false");
#endif
                    }
		    if (cJSON_HasObjectItem(pJSON, "panel_1bit")) {
		         pItem = cJSON_GetObjectItem(pJSON, "panel_1bit");
			 iPanel1Bit = FindItemName(szPanels, pItem->valuestring, "1-bit panel");
                         if (iPanel1Bit >= 0) {
#ifdef SHOW_DETAILS
                             printf("panel1bit = %s\n", szPanels[iPanel1Bit]);
#endif
                         }
		    }
		    if (cJSON_HasObjectItem(pJSON, "mode")) {
			 pItem = cJSON_GetObjectItem(pJSON, "mode");
			 iMode = FindItemName(szModes, pItem->valuestring, "update mode");
                         if (iMode >= 0) {
#ifdef SHOW_DETAILS
			     printf("mode = %s\n", szModes[iMode]);
#endif
                         }
		    }
		    if (cJSON_HasObjectItem(pJSON, "file")) {
                         pItem = cJSON_GetObjectItem(pJSON, "file");
			 strcpy(szFile, pItem->valuestring);
		    }
		    cJSON_Delete(pJSON);
	    } else {
		    printf("Error parsing JSON!\n");
	    }
	    free(pData);
    } // if show_img.json file exists

    if (argc > 1) {
	    printf("cli parameters overriding JSON...\n");
    }
    for (int i=1; i<argc; i++) {
        char *pName, *pValue, *saveptr;
        pName = strtok_r((char *)argv[i], "=", &saveptr);
	pValue = strtok_r(NULL, "=", &saveptr);
	printf("%d: %s %s\n", i, pName, pValue); 
        if (strcmp(pName, "mode") == 0) {
		iMode = FindItemName(szModes, pValue, "update mode");
	} else if (strcmp(pName, "stretch") == 0) {
		iStretch = FindItemName(szStretch, pValue, "stretch");
	} else if (strcmp(pName, "file") == 0) {
		strcpy(szFile, pValue);
	} else if (strcmp(pName, "panel_1bit") == 0) {
		iPanel1Bit = FindItemName(szPanels, pValue, "1-bit panel");
	} else if (strcmp(pName, "invert") == 0) {
                iInvert = !strcmp(pValue, "true");
        }
    }
    if (szFile[0] == 0 || iMode == -1 || iPanel1Bit == -1) { // print instructions
        ShowHelp();
        return -1;
    }

    if (iStretch < 0) iStretch = STRETCH_ASPECTFILL; // default
    bbep.initPanel(BB_PANEL_RPI);
    bbep.setPanelSize(iPanel1Bit);
    bbep.fillScreen(BBEP_WHITE);
#ifdef SHOW_DETAILS
    printf("Decoding image...\n");
#endif

    // Read the file into RAM
    ihandle = fopen(szFile, "r+b");
    if (ihandle == NULL) {
        printf("Error opening file %s\n", szFile);
        return -1;
    }
    fseek(ihandle, 0, SEEK_END);
    iSize = (int)ftell(ihandle);
    fseek(ihandle, 0, SEEK_SET);
    pData = (uint8_t *)malloc(iSize);
    rc = fread(pData, 1, iSize, ihandle);
    fclose(ihandle);
    if (rc != iSize) {
	    printf("Error reading file!\n");
	    return -1;
    }
    if (iSize < 64) { // invalid file
        printf("Invalid image file\n");
	return -1;
    }
    if (pData[0] == 'B' && pData[1] == 'M') { // it's a BMP file
        rc = DecodeBMP(pData, iSize);
    } else if (pData[0] == 0xff && pData[1] == 0xd8) { // JPEG
        rc = DecodeJPEG(pData, iSize);
        if (rc != JPEG_SUCCESS) {
            if (rc > 0) {
                printf("JPEG decode returned error: %s\n", szJPEGErrors[rc]);
            }
            return -1;
        }
    } else {
        rc = DecodePNG(pData, iSize);
        if (rc != PNG_SUCCESS) {
            if (rc > 0) {
                printf("PNG decode returned error: %s\n", szPNGErrors[rc]);
            }
            return -1;
        }
    }
#ifdef SHOW_DETAILS
    printf("image specs: %d x %d, %d-bpp\n", iWidth, iHeight, iBpp);
#endif

//        if (bbep.width() < bbep.height()) {
//            bbep.setRotation(270); // assume landscape mode for all images
//        }
        // convert+copy the image into the local EPD framebuffer
#ifdef SHOW_DETAILS
        printf("Preparing image for EPD...\n");
#endif
    PrepareImage();
    bbep.fullUpdate(CLEAR_SLOW, false);
#ifdef SHOW_DETAILS
    printf("Refresh complete, shutting down...\n");
#endif
    bbep.deInit(); // turn off the epaper power circuit
    return 0;
} /* main() */


