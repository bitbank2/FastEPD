//
// File navigator and TIFF/JPEG image viewer demo sketch
// written by Larry Bank
//
#include <SPI.h>
#include <AnimatedGIF.h>
#include <JPEGDEC.h>
#include <PNGdec.h>
#include <TIFF_G4.h>
#include "Roboto_Black_30.h"
#include <FastEPD.h>
#include <bb_captouch.h>
#include <SD.h>
#include <stdlib.h>
#include "DejaVu_Sans_Mono_Bold_16.h"
// Touch button rectangles
#define NUM_BUTTONS 3
#define RKEY_UP 1
#define RKEY_DOWN 2
#define RKEY_BUTT1 4
#define RKEY_EXIT 8

int iButtonX[NUM_BUTTONS], iButtonY[NUM_BUTTONS];
int iButtonCX[NUM_BUTTONS], iButtonCY[NUM_BUTTONS];
int iButtonValues[NUM_BUTTONS] = {RKEY_UP, RKEY_DOWN, RKEY_BUTT1};
const void *pSmallFont = (void *)DejaVu_Sans_Mono_Bold_16;
const void *pBigFont = (void *)Roboto_Black_30;
bool bRotated = true;
JPEGDEC *jpg;
PNG *png;
AnimatedGIF *gif;
TIFFG4 *tif;
FASTEPD epd;
BBCapTouch bbct;

#define DISPLAY_WIDTH 960
#define DISPLAY_HEIGHT 540
#define NAMES_TOP_Y 64
#define NAMES_LEFT_X 8
#define TEXT_HEIGHT 32
uint8_t ucTemp[DISPLAY_WIDTH * 2];
static int iDisplayWidth = DISPLAY_HEIGHT, iDisplayHeight = DISPLAY_WIDTH; // we're working 90 degrees rotated
static int iOldSelected;
int iXOff, iYOff;
static int iIdleCount;
// one minute of inactivity (10 ms ticks)
#define IDLE_TIMEOUT (100 * 60)

// GT911 capacitive touch sensor
#define TOUCH_SDA 41
#define TOUCH_SCL 42
#define TOUCH_INT 48
#define TOUCH_RST -1

// uSD card
#define SD_CS 47
#define SD_SCK 39
#define SD_MOSI 38
#define SD_MISO 40

volatile uint32_t u32PrevBits, u32GPIOBits;

void SG_WriteString(int x, int y, char *pString, bool bSelected, int bLarge)
{
int16_t rx, ry;
uint16_t rw, rh;
BBEPRECT rect;

   epd.setTextColor((bSelected) ? BBEP_WHITE : BBEP_BLACK, BBEP_TRANSPARENT);
   epd.setFont((bLarge) ? pBigFont : pSmallFont);
   y -= 8;
   epd.setCursor(x, y + NAMES_TOP_Y);
   if (!bLarge) { // draw a rectangle to allow seeing the current selection
      epd.getStringBox("A", &rect);
      epd.fillRect(0, rect.y, iDisplayWidth, rect.h, (bSelected) ? BBEP_BLACK : BBEP_WHITE);
   }
   epd.print(pString);
} /* SG_WriteString() */

void GUIDrawNames(char *pDir, char *pDirNames, char *pFileNames, int *pSizes, int iDirCount, int iFileCount, int iSelected)
{
int i, iLen, iNumLines, iNumCols;
int iCurrent;
char szTemp[512];
      // Get the height of the small font in pixels
        SG_WriteString(6,0,(char*)"SD File Explorer", false, 1);
        iNumLines = (iDisplayHeight - (NAMES_TOP_Y + TEXT_HEIGHT))/TEXT_HEIGHT;
        iNumCols = 29; //(iDisplayWidth*3)/(TEXT_HEIGHT*2);
        strcpy(szTemp, pDir);
        strcat(szTemp, "                                        ");
        szTemp[iNumCols-2] = 0;
        SG_WriteString(8,TEXT_HEIGHT,szTemp, false,0);
        if (iSelected >= iNumLines) iCurrent = iSelected-(iNumLines-1);
        else iCurrent = 0;
        for (i=0; i<(iDirCount + iFileCount); i++) { // draw all lines on the display
            if (iCurrent >= (iDirCount+iFileCount)) { // short list, fill with spaces
                 strcpy(szTemp, (char*)"                                         ");
            } else {
                if (iCurrent >= iDirCount) { // use filenames
                   strcpy(szTemp, &pFileNames[(iCurrent-iDirCount)*256]);
                } else { // use dir names
                   strcpy(szTemp, &pDirNames[iCurrent*256]);
                }
                iLen = strlen(szTemp);
            }
            if (iCurrent >= iDirCount) { // use filenames
                int iKBytes = (pSizes[iCurrent-iDirCount] >> 10);
                char szTemp2[16]; 
                strcat(szTemp, "                                                   "); // to erase any old text
                if (iKBytes == 0) iKBytes = 1;
                strcpy(szTemp, &pFileNames[(iCurrent-iDirCount)*256]);
                strcat(szTemp, "                                                  ");          
                if (iKBytes > 1024) {
                  iKBytes >>= 10;
                  if (iKBytes < 1) iKBytes = 1;
                  sprintf(szTemp2, "%dM", iKBytes);
                } else {
                  sprintf(szTemp2, "%dK", iKBytes);
                }
                strcpy(&szTemp[iNumCols-1-strlen(szTemp2)], szTemp2);
            } else {
                strcat(szTemp, "        "); // to erase any old text
            }
            SG_WriteString(NAMES_LEFT_X, NAMES_TOP_Y+(i*TEXT_HEIGHT), szTemp, (iCurrent == iSelected), 0);
//           Serial.println(szTemp);
            iCurrent++;
        }
} /* GUIDrawNames() */

void UpdateGPIOButtons(void)
{
  u32GPIOBits = 0;
  TOUCHINFO ti;
  int i, x, y;
  bbct.getSamples(&ti);
  if (ti.count > 0) {
    if (ti.count == 2) { // special case for "EXIT" button
        u32GPIOBits |= RKEY_EXIT;
        return;
    }
    // search for match with button area (x and y are reverse relative to the display)
    x = DISPLAY_HEIGHT - 1 - ti.x[0];
    y = DISPLAY_WIDTH - 1 - ti.y[0];
    for (i = 0; i<NUM_BUTTONS; i++) {
        if (x >= iButtonX[i] && x < (iButtonX[i] + iButtonCX[i]) &&
        y >= iButtonY[i] && (y < iButtonY[i] + iButtonCY[i])) {
          u32GPIOBits |= iButtonValues[i]; // got one
          //Serial.printf("Button %d pressed\n", i);
          return;
        }
    }
  }
} /* UpdateGPIOButtons() */

void GetLeafName(char *fname, char *leaf)
{
int i, iLen;

   iLen = strlen(fname);
   for (i=iLen-1; i>=0; i--)
      {
      if (fname[i] == '/')
         break;
      }
   strcpy(leaf, &fname[i+1]);
} /* GetLeafName() */

//
// Adjust the give path to point to the parent directory
//
void GetParentDir(char *szDir)
{
int i, iLen;
        iLen = strlen(szDir);
        for (i=iLen-1; i>=0; i--)
        {
                if (szDir[i] == '/') { // look for the next slash 'up'
                   szDir[i] = 0;
                   break;
                }
        }
        if (strlen(szDir) == 0)
           strcat(szDir, "/"); // we hit the root dir
} /* GetParentDir() */

int name_compare(const void *ina, const void *inb)
{
char *a = (char *)ina;
char *b = (char *)inb;

 while (*a && *b) {
        if (tolower(*a) != tolower(*b)) {
            break;
        }
        ++a;
        ++b;
    }
    return (int)(tolower(*a) - tolower(*b));
} /* name_compare() */

// Increment the idle counter
void IncrementIdle(void)
{
  iIdleCount++;
  if (iIdleCount > IDLE_TIMEOUT) {
     //EPD.Power(false); // turn off EPD
     //deepSleep(1000 * 60 * 60 * 24 * 7); // sleep for a week
  }
}

void ClearIdle(void)
{
  iIdleCount = 0;
}
void UpdateDisplay(int x, int y, int w, int h, bool bFullUpdate)
{
      if (bFullUpdate) {
        epd.fullUpdate(true);
      } else {
        epd.partialUpdate(true); //y, iDisplayWidth-x-w, h, w, UPDATE_MODE_GC16); //UPDATE_MODE_DU4); //UPDATE_MODE_GLD16);
      }
} /* UpdateDisplay() */

//
// Calculate the update rectangle to just draw the name being deselected and newly selected
//
void UpdateSelection(int iSelected)
{
  epd.partialUpdate(true);
}
// Draw the touch navigation buttons at the bottom of the display
void DrawButtons(void)
{
  epd.setFont(pSmallFont);
  epd.setTextColor(BBEP_WHITE, BBEP_TRANSPARENT);
  // Up button
  iButtonX[0] = 8;
  iButtonY[0] = iButtonY[1] = iButtonY[2] = iDisplayHeight-70;
  iButtonCX[0] = iButtonCX[1] = iDisplayWidth/6;
  iButtonCY[0] = iButtonCY[1] = iButtonCY[2] = 60;
  epd.fillRoundRect(iButtonX[0], iButtonY[0] , iButtonCX[0], iButtonCY[0], 12, BBEP_BLACK);
  epd.setCursor(32, iDisplayHeight-29);
  epd.print("UP");

  // Down button
  iButtonX[1] = 8 + iDisplayWidth / 5;
  epd.fillRoundRect(iButtonX[1], iButtonY[1], iButtonCX[1], iButtonCY[1], 12, BBEP_BLACK);
  epd.setCursor(15 + iDisplayWidth/5, iDisplayHeight-29);
  epd.print("DOWN");

  // Select button
  iButtonX[2] = 8 + 3 * (iDisplayWidth / 5);
  iButtonCX[2] = iDisplayWidth/4;
  epd.fillRoundRect(iButtonX[2], iButtonY[2], iButtonCX[2], iButtonCY[2], 12, BBEP_BLACK);
  epd.setCursor(22 + 3 * (iDisplayWidth/5), iDisplayHeight-29);
  epd.print("SELECT");
} /* DrawButtons() */

void NavigateFiles(char *cDir, char *szDestName)
{
File root, dir;
int iSelected;
int iReturn = 0;
int iDirCount, iFileCount, iDir, iFile;
int bDone = 0;
int iRepeat = 0;
char *pDirNames = NULL;
char *pFileNames = NULL;
int *pSizes = NULL;
char szTemp[256];
uint32_t u32Quit;
int iMaxSelection;
int bDirValid;


   DrawButtons();
   iDir = iFile = iDirCount = iMaxSelection = iFileCount = 0;
      while (!bDone) {
      root = SD.open(cDir);
      if (root) {
         dir = root.openNextFile();
         if (dir) {
            // count the number of non-hidden directories and files
            iDirCount = 1;
            iFileCount = 0;
            while (dir) { 
              GetLeafName((char *)dir.name(), szTemp);
              if (dir.isDirectory() && szTemp[0] != '.')
                iDirCount++;
              else if (!dir.isDirectory() && szTemp[0] != '.')
                iFileCount++;
              dir.close();
              dir = root.openNextFile();
              delay(5);
            }
            root.rewindDirectory();
            if (pDirNames) {  
               free(pDirNames);
               free(pFileNames);
               free(pSizes);
            }
            pDirNames = (char *)malloc(256 * (iDirCount+1));
            pFileNames = (char *)malloc(256 * (iFileCount+1));
            pSizes = (int *)malloc((iFileCount+1) * sizeof(int));
            // now capture the names
            iDir = 1; // store ".." as the first dir
            strcpy(pDirNames, "..");
            iFile = 0;
            dir = root.openNextFile();
            while (dir) {  
               GetLeafName((char *)dir.name(), szTemp);
               if (dir.isDirectory() && szTemp[0] != '.') {  
                  strcpy(&pDirNames[iDir*256], szTemp);
                  iDir++;
               } else if (!dir.isDirectory() && szTemp[0] != '.') {
                 pSizes[iFile] = dir.size();
                 strcpy(&pFileNames[iFile*256], szTemp);
                 iFile++;
               }
               dir.close();
               dir = root.openNextFile();
               delay(5);
            }
         }
         root.close();
         iDirCount = iDir;
         iFileCount = iFile; // get the usable names count
         iMaxSelection = iDirCount + iFileCount;
         Serial.printf("dirs = %d, files = %d\n", iDirCount, iFileCount);
         Serial.flush();
        // Sort the names
       // qsort(pDirNames, iDirCount, 256, name_compare);
       // qsort(pFileNames, iFileCount, 256, name_compare);
      } // dir opened
restart:
      iSelected = 0;
      GUIDrawNames(cDir, pDirNames, pFileNames, pSizes, iDirCount, iFileCount, iSelected);
      UpdateDisplay(0,0,iDisplayWidth, iDisplayHeight, false); // redraw everything

      bDirValid = 1;
      while (bDirValid) {
         UpdateGPIOButtons();
         if (u32GPIOBits == u32PrevBits) {
            IncrementIdle();
            lightSleep(10); // save some power while we wait
            iRepeat++;
            if (iRepeat < 100) // 1 second starts a repeat
               continue;
         } else { // change means cancel repeat
            iRepeat = 0;
            ClearIdle();
         }
         if (((u32GPIOBits & u32Quit) == u32Quit) && ((u32PrevBits & u32Quit) != u32Quit)) { // quit menu
//                int rc, i = QuitMenu();
//                if (i==0){bDirValid = 0; continue;}
//              else if (i==1) {Terminal(); continue;}
//                else if (i==2) {bDirValid=0; bDone=1; continue;}
//              else if (i==3) {spilcdFill(0); rc = system("sudo shutdown now");}
//              else if (i==4) rc = system("sudo reboot");
//                else {bDirValid=0; continue;} // continue after 'About'
//                if (rc < 0) {};
         }
//        if (((u32GPIOBits & RKEY_EXIT) == RKEY_EXIT) && ((u32PrevBits & RKEY_EXIT) != RKEY_EXIT))
//        {
        // quit menu - 0=resume, 1=quit, 2=shutdown
//        }
        if (u32GPIOBits & RKEY_UP && (iRepeat || !(u32PrevBits & RKEY_UP))) { // navigate up
            if (iSelected > 0) {
               iSelected--;
               GUIDrawNames(cDir, pDirNames, pFileNames, pSizes, iDirCount, iFileCount, iSelected);
               UpdateDisplay(0,0,iDisplayWidth, iDisplayHeight, false); // redraw everything
            }
         }
         if (u32GPIOBits & RKEY_DOWN && (iRepeat || !(u32PrevBits & RKEY_DOWN))) { // navigate down
            if (iSelected < iMaxSelection-1) {
               iSelected++;
               GUIDrawNames(cDir, pDirNames, pFileNames, pSizes, iDirCount, iFileCount, iSelected);
               UpdateDisplay(0,0,iDisplayWidth, iDisplayHeight, false); // redraw everything
            }
         }
         if (u32GPIOBits & RKEY_BUTT1 && !(u32PrevBits & RKEY_BUTT1)) {
            bDirValid = 0;
            if (iSelected == 0) // the '..' dir goes up 1 level
            {
               if (strcmp(cDir, "/") != 0) // navigating beyond root will mess things up
               {
                  GetParentDir(cDir);
               }
            } else {
               if (iSelected < iDirCount) { // user selected a directory
                  if (strcmp(cDir, "/") != 0)
                     strcat(cDir, "/");
                  strcat(cDir, &pDirNames[iSelected*256]);
               } else { // user selected a file, leave
//                  strcpy(szDestName, "/sd");
                  strcpy(szDestName, "/");
                  if (strcmp(cDir, "/") != 0)
                     strcat(szDestName, "/");
                  strcat(szDestName, &pFileNames[(iSelected-iDirCount)*256]);
                  bDone = 1; // exit the main loop
                  iReturn = 1;
               }
            }
         }
         u32PrevBits = u32GPIOBits;
      } // while bDirValid
   }
} /* NavigateFiles() */

// Functions to access a file on the SD card
File myfile;

void * myOpen(const char *filename, int32_t *size) {
  myfile = SD.open(filename);
  *size = myfile.size();
  return &myfile;
}
void myClose(void *handle) {
  if (myfile) myfile.close();
}

int32_t TIFFRead(TIFFFILE *handle, uint8_t *buffer, int32_t length) {
  if (!myfile) return 0;
  return myfile.read(buffer, length);
}
int32_t TIFFSeek(TIFFFILE *handle, int32_t position) {
  if (!myfile) return 0;
  return myfile.seek(position);
}

int32_t myRead(JPEGFILE *handle, uint8_t *buffer, int32_t length) {
  if (!myfile) return 0;
  return myfile.read(buffer, length);
}
int32_t mySeek(JPEGFILE *handle, int32_t position) {
  if (!myfile) return 0;
  return myfile.seek(position);
}
int32_t GIFReadFile(GIFFILE *pFile, uint8_t *pBuf, int32_t iLen)
{ 
    int32_t iBytesRead;
    iBytesRead = iLen;
    File *f = static_cast<File *>(pFile->fHandle);
    // Note: If you read a file all the way to the last byte, seek() stops working
    if ((pFile->iSize - pFile->iPos) < iLen)
       iBytesRead = pFile->iSize - pFile->iPos - 1; // <-- ugly work-around
    if (iBytesRead <= 0)
       return 0;
    iBytesRead = (int32_t)f->read(pBuf, iBytesRead);
    pFile->iPos = f->position();
    return iBytesRead;
} /* GIFReadFile() */

int32_t GIFSeekFile(GIFFILE *pFile, int32_t iPosition)
{
  File *f = static_cast<File *>(pFile->fHandle);
  f->seek(iPosition);
  pFile->iPos = (int32_t)f->position();
  return pFile->iPos;
} /* GIFSeekFile() */

int32_t pngRead(PNGFILE *handle, uint8_t *buffer, int32_t length) {
  if (!myfile) return 0;
  return myfile.read(buffer, length);
}
int32_t pngSeek(PNGFILE *handle, int32_t position) {
  if (!myfile) return 0;
  return myfile.seek(position);
}

// Function to draw pixels to the display
void PNGDraw(PNGDRAW *pDraw) {
  uint16_t u16, *s;
  uint8_t *d;
  int iPitch, x, g0, g1;
  // easier to use this than to rewrite all of that code
  png->getLineAsRGB565(pDraw, (uint16_t *)ucTemp, PNG_RGB565_LITTLE_ENDIAN, 0xffffffff);
  s = (uint16_t *)ucTemp;
  d = ucTemp;
  for (x=0; x<pDraw->iWidth; x+=2) {
      u16 = *s++;
      g0 = (u16 & 0x7e0) >> 5; // calculate gray level
      g0 += (u16 & 0x1f);
      g0 += ((u16 & 0xf800) >> 11);
      g0 = (g0 >> 3); // get 4-bit value
      u16 = *s++;
      g1 = (u16 & 0x7e0) >> 5; // calculate gray level
      g1 += (u16 & 0x1f);
      g1 += ((u16 & 0xf800) >> 11);
      g1 = (g1 >> 3); // get 4-bit value
      *d++ = (uint8_t)((g0 << 4) | g1);
  }
  d = epd.currentBuffer();
  iPitch = DISPLAY_WIDTH/2;
  d += ((iYOff + pDraw->y) * iPitch);
  d += iXOff / 2;
  memcpy(d, ucTemp, pDraw->iWidth/2);
} /* PNGDraw() */

void GIFDraw(GIFDRAW *pDraw)
{
    uint8_t *s, *d;
    uint16_t u16, *usPalette;
    int x, g0, g1, iWidth; 
    
    iWidth = pDraw->iWidth;
    if (iWidth > DISPLAY_WIDTH) 
       iWidth = DISPLAY_WIDTH;
    usPalette = pDraw->pPalette;
    // Since we only render the first frame, no need to worry about transparency
    s = pDraw->pPixels;
    d = ucTemp;
    for (x=0; x<iWidth; x+=2) {
       u16 = usPalette[*s++];
       g0 = (u16 & 0x7e0) >> 5; // 6 bits of green
       g0 += (u16 >> 11); // 5 bits of red
       g0 += (u16 & 0x1f); // 5 bits of blue
       g0 = ~(g0 >> 3); // keep 4 bits and invert
       u16 = usPalette[*s++];
       g1 = (u16 & 0x7e0) >> 5; // 6 bits of green
       g1 += (u16 >> 11); // 5 bits of red
       g1 += (u16 & 0x1f); // 5 bits of blue
       g1 = ~(g1 >> 3); // keep 4 bits and invert
       *d++ = (g0 << 4) | g0;
    }
    //EPD.WritePartGram4bpp(iXOff, iYOff + pDraw->y, (pDraw->iWidth + 7) & 0xfff8, 1, (const uint8_t *)ucTemp); // This writes one line into the EPD framebuffer
} /* GIFDraw() */

void TIFFDraw(TIFFDRAW *pDraw)
{ 
  int x, iPitch;
  uint8_t *d, *s, uc, out;
  // need to invert the colors
  s = pDraw->pPixels;
  d = epd.currentBuffer();
  iPitch = DISPLAY_WIDTH/2;
  d += (iPitch * iYOff);
  d += iXOff/2;
  for (x=0; x<pDraw->iWidth+7; x+=8) {
    uc = *s++;
    out = 0xff;
    if (uc & 0x80) out ^= 0xf0;
    if (uc & 0x40) out ^= 0xf;
    *d++ = out;
    out = 0xff;
    if (uc & 0x20) out ^= 0xf0;
    if (uc & 0x10) out ^= 0xf;
    *d++ = out;
    out = 0xff;
    if (uc & 0x8) out ^= 0xf0;
    if (uc & 0x4) out ^= 0xf;
    *d++ = out;
    out = 0xff;
    if (uc & 0x2) out ^= 0xf0;
    if (uc & 0x1) out ^= 0xf;
    *d++ = out;
  }
} /* TIFFDraw() */

// Function to draw pixels to the display
int JPEGDraw(JPEGDRAW *pDraw) {
  int x, y, iPitch = DISPLAY_WIDTH / 2; // native orientation
  uint8_t *s, *d, *pBuffer = epd.currentBuffer();
  for (y = 0; y < pDraw->iHeight && y + pDraw->y < DISPLAY_HEIGHT; y++) {
    d = &pBuffer[((pDraw->y + y) * iPitch) + (pDraw->x / 2)];
    s = (uint8_t *)pDraw->pPixels;
    s += (y * pDraw->iWidth/2);
    for (x = 0; x < pDraw->iWidth; x+=2) { // 2 pixels per byte
      *d++ = *s++;
    } // for x
  } // for y
  return 1;
} /* JPEGDraw() */

int ViewFile(char *szName)
{
int rc, x, y, w, h, iScale, iOptions;
uint8_t *pDitherBuffer;

    epd.setMode(BB_MODE_4BPP); // switch to 16-gray mode
    epd.fillScreen(0xf);
    // Try to open the file
    Serial.printf("Opening %s\n", szName);
    rc = strlen(szName);
    if (strcmp(&szName[rc-3], "jpg") == 0) {
    jpg = (JPEGDEC *)malloc(sizeof(JPEGDEC));
    if (!jpg) return 0;
    rc = jpg->open((const char *)szName, myOpen, myClose, myRead, mySeek, JPEGDraw);
    if (rc) {
      Serial.printf("jpeg opened, size = %dx%d\n", jpg->getWidth(), jpg->getHeight());
      // See if we need to scale down the image to fit the display
      iScale = 1; iOptions = 0;
      w = jpg->getWidth(); h = jpg->getHeight();
      if (w >= DISPLAY_WIDTH*8 || h >= DISPLAY_HEIGHT * 8) {
         iScale = 8;
         iOptions = JPEG_SCALE_EIGHTH;
      } else if (w >= DISPLAY_WIDTH * 4 || h >= DISPLAY_HEIGHT * 4) {
         iScale = 4;
         iOptions = JPEG_SCALE_QUARTER;
      } else if (w >= DISPLAY_WIDTH * 2 || h >= DISPLAY_HEIGHT * 2) {
         iScale = 2;
         iOptions = JPEG_SCALE_HALF;
      }
      // Center the image on the display
      x = (DISPLAY_WIDTH - w/iScale) / 2;
      if (x < 0) x = 0;
      y = (DISPLAY_HEIGHT - h/iScale) / 2;
      if (y < 0) y = 0;
      Serial.printf("image offset: %d,%d\n", x, y);
      jpg->setPixelType(FOUR_BIT_DITHERED);
      pDitherBuffer = (uint8_t *)malloc((w+16) * 16);
      jpg->decodeDither(x, y, pDitherBuffer, iOptions);
      jpg->close();
      free(pDitherBuffer);
      free(jpg);
      epd.fullUpdate(true);
      Serial.println("Finished jpeg decode");
      return 1;
    } // jpeg opened
    } else if (strcmp(&szName[rc-3], "png") == 0) {
      png = (PNG *)malloc(sizeof(PNG));
      if (!png) return 0;
        rc = png->open((const char *)szName, myOpen, myClose, pngRead, pngSeek, PNGDraw); 
       if (rc == PNG_SUCCESS) {
          w = png->getWidth();
          h = png->getHeight();
          Serial.printf("image specs: (%d x %d), %d bpp, pixel type: %d\n", w, h, png->getBpp(), png->getPixelType());
          // Center the image on the display
          x = (DISPLAY_WIDTH - w) / 2;
          if (x < 0) x = 0;
          y = (DISPLAY_HEIGHT - h) / 2;
          if (y < 0) y = 0;
          iXOff = x; iYOff = y; // DEBUG
          rc = png->decode(NULL, 0);
          png->close();
          free(png);
          epd.fullUpdate(true);
          Serial.println("Finished png decode");
       }
    } else if (strcmp(&szName[rc-3], "gif") == 0) {
      gif = (AnimatedGIF *)malloc(sizeof(AnimatedGIF));
      if (!gif) return 0;
      gif->begin(LITTLE_ENDIAN_PIXELS);
      if (gif->open(szName, myOpen, myClose, GIFReadFile, GIFSeekFile, GIFDraw)) {
         iXOff = (DISPLAY_WIDTH - gif->getCanvasWidth())/2;
         if (iXOff < 0) iXOff = 0;
         iYOff = (DISPLAY_HEIGHT - gif->getCanvasHeight())/2;
         if (iYOff < 0) iYOff = 0;
         Serial.printf("Successfully opened GIF; Canvas size = %d x %d\n", gif->getCanvasWidth(), gif->getCanvasHeight());
         gif->playFrame(false, NULL); // decode one frame
         gif->close();
         free(gif);
         epd.fullUpdate(true);
         Serial.println("Finished gif decode");
      } // gif opened
    } else if (strcmp(&szName[rc-3], "tif") == 0) {
      tif = (TIFFG4 *)malloc(sizeof(TIFFG4));
      if (!tif) return 0;
      rc = tif->openTIFF(szName, myOpen, myClose, TIFFRead, TIFFSeek, TIFFDraw);
      if (rc) {
         w = tif->getWidth();
         h = tif->getHeight();
          Serial.printf("TIFF opened, size = (%d x %d)\n", w, h);
          // Center the image on the display
          x = (DISPLAY_WIDTH - w) / 2;
          if (x < 0) x = 0;
          y = (DISPLAY_HEIGHT - h) / 2;
          if (y < 0) y = 0;
          iXOff = x; iYOff = y; // DEBUG
          //tif->setDrawParameters(1.0, TIFF_PIXEL_4BPP, 0, 0, DISPLAY_WIDTH, DISPLAY_HEIGHT, ucTemp);
          tif->decode(x, y);
          tif->close();
          free(tif);
          epd.fullUpdate(true);
          Serial.println("Finished tiff decode");
      } else {
        Serial.printf("Error opening TIFF = %d\n", rc);
      }
    } // tif
    return 0;
} /* ViewFile() */

void lightSleep(uint64_t time_in_ms)
{
  delay(time_in_ms);
  //esp_sleep_enable_timer_wakeup(time_in_ms * 1000L);
  //esp_light_sleep_start();
}
void deepSleep(uint64_t time_in_ms) 
{
#ifdef FUTURE
  esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_OFF);
  esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_SLOW_MEM, ESP_PD_OPTION_OFF);
  esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_FAST_MEM, ESP_PD_OPTION_OFF);
  esp_sleep_pd_config(ESP_PD_DOMAIN_XTAL,         ESP_PD_OPTION_OFF);
  esp_sleep_enable_timer_wakeup(time_in_ms * 1000L); 
  esp_deep_sleep_start();
#endif
}

void setup() {
  int rc;
  Serial.begin(115200);
  delay(3000);
  Serial.println("Starting...");
  rc = bbct.init(TOUCH_SDA, TOUCH_SCL, TOUCH_RST, TOUCH_INT);
  if (rc == CT_SUCCESS) {
     int iType = bbct.sensorType();
     Serial.printf("Sensor type = %d\n", iType);
  } else {
    Serial.println("Cap touch initialization failed!");
    while (1) {};
  }
  epd.initPanel(BB_PANEL_M5PAPERS3);
  epd.setRotation(270);
  epd.fillScreen(BBEP_WHITE);
  epd.fullUpdate(true, true);
  SPI.begin(SD_SCK, SD_MISO, SD_MOSI, SD_CS);
  while (!SD.begin(SD_CS, SPI, 10000000)) {
     Serial.println("Unable to access SD card");
     delay(1000);
  }
  Serial.println("SD card success");
} /* setup() */

void loop() {
char szDir[256], szName[256];

  strcpy(szDir, "/"); // this needs to be a non-const string because it will be modified in NavigateFiles()
  while (1) {
    int x, y, w, h, rc, bDone;
    int iOptions = 0, iScale = 1;
    NavigateFiles(szDir, szName);
    if (ViewFile(szName)) {
    } else {
      SG_WriteString(0, 20, (char *)"Error opening file", false, 0);
      SG_WriteString(16, 40, (char *)"Press action key", false, 0);
    }
    u32PrevBits = u32GPIOBits;
    bDone = 0;
    while (!bDone) {
      UpdateGPIOButtons();
      if (u32GPIOBits & RKEY_EXIT && !(u32PrevBits & RKEY_EXIT)) {// newly pressed
         bDone = 1;
         ClearIdle();
      }
      u32PrevBits = u32GPIOBits;
      lightSleep(50); // save power while waiting for the user
      IncrementIdle();
    } // while waiting for key press
   epd.setMode(BB_MODE_1BPP);
   epd.fillScreen(BBEP_WHITE);
   epd.fullUpdate(true, true); // prepare to navigate to a new file
  } // while 1
} /* loop() */
