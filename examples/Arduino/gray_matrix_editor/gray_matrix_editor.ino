//
// FastEPD gray matrix editor
// A more efficient way to interactively edit the
// 16-gray level table to achieve 16 good looking gray values
// on each EInk panel (they are all slightly different).
//
// Written by Larry Bank (bitbank@pobox.com)
// April 12, 2025
//
// This program presents a command line interface through the ESP32 serial port
// It interacts line by line and can be driven from the Arduino Serial Terminal or any other
// terminal program.
//
#include <FastEPD.h>
FASTEPD epaper;
uint8_t ucTemp[64];
uint8_t u8_last[16 * 48]; // to allow UNDO of a single change
// Starting gray matrix. Copy your current one here before compiling/running the program
uint8_t u8_graytable[] = {
/* 0 */	  0,  0,  0,  0,  0,  0,  0,  1,  1,  1,  1,  1,  1,  1,  1,  0,  0,  0,
/* 1 */		0,	0,	0,	0,	0,	0,	0,	0,	0,	1,	2,	1,	1,	1,	0,	0,	0,	0,
/* 2 */		0,	0,	0,	0,	0,	1,	1,	1,	2,	1,	1,	1,	2,	1,	0,	0,	0,	0,
/* 3 */		0,	0,	0,	0,	0,	0,	0,	0,	1,	1,	1,	2,	1,	0,	0,	0,	0,	0,
/* 4 */	  0,	0,	0,	0,	0,	0,	0,	1,	1,	1,	2,	1,	0,	0,	0,	0,	0,	0,
/* 5 */		0,	0,	0,	0,	0,	1,	0,	0,	1,	2,	0,	1,	0,	0,	0,	0,	0,	0,
/* 6 */		0,	0,	0,	0,	0,	1,  2,	0,	1,	2,	0,	1,	0,	0,	0,	0,	0,	0,
/* 7 */ 	0,	0,	0,	0,	0,	1,	1,	1,	1,	1,	1,	1,	1,	1,	2,	0,	0,	0,
/* 8 */ 	0,	0,	0,	0,	0,	0,	0,	1,	2,	2,	1,	2,	1,	0,	0,	0,	0,	0,
/* 9 */ 	0,	0,	0,	0,	0,	0,	0,	0,	1,	1,	1,	1,	1,	1,	2,	0,	0,	0,
/* 10 */	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	1,	1,	1,	1,	2,	0,	0,	0,
/* 11 */	0,	0,	0,	0,	0,	0,	1,	1,	1,	1,	1,	1,	1,	2,	1,	2,	0,	0,
/* 12 */	0,	0,	0,	0,	0,	1,	1,	1,	2,	1,	2,	0,	0,	0,	0,	0,	0,	0,
/* 13 */	0,	0,	0,	0,	0,	1,	1,	2,	2,	2,	2,	1,	2,	0,	0,	0,	0,	0,
/* 14 */	1,	1,	1,	1,	1,	1,	2,	2,	1,	2,	2,	0,	0,	0,	0,	0,	0,	0,
/* 15 */	0,	1,	1,	1,	1,	1,	1,	2,	2,	2,	2,	2,	2,	2,	2,	2,	2,	2
};

static int passes; // Calculated at startup based on the matrix size
// List of supported commands
const char *szCMDs[] = {"HELP", "LIST", "SHOW", "COPY", "SWAP", "EDIT", "UNDO", 0};
enum {
  CMD_HELP = 0,
  CMD_LIST,
  CMD_SHOW,
  CMD_COPY,
  CMD_SWAP,
  CMD_EDIT,
  CMD_UNDO,
  CMD_COUNT
};

// 125 seconds of no input will send a nag msg
#define MAX_WAIT 2500
bool getCmd(char *szString)
{
bool bFim = false;
char c, *d = szString;

int r, iTimeout = 0;

   d[0] = 0; // start with a null string in case we timeout
   while (!bFim && iTimeout < MAX_WAIT) {
     while (!Serial.available() && iTimeout < MAX_WAIT) {
       delay(50); // wait for characters to arrive
       iTimeout++;
     }
     if (Serial.available()) {
       c = Serial.read();
       if (c == 0xa || c == 0xd) { // end of line
          *d++ = 0; // terminate the string
          return false; // break out of loop
       }
       *d++ = c;
     } // if character in buffer
   } // while waiting for complete response
   return (iTimeout >= MAX_WAIT);
} /* getCmd() */

// Return the length and value of the parsed substring
int delimitedValue(char *szText, int *pValue, int iLen)
{
int i, j, val;
  // skip leading spaces
  i = 0;
  while (i < iLen && (szText[i] == ' ' || szText[i] == ',')) i++;
  j = i;
  while (j < iLen && szText[j] != ' ' && szText[j] != ',') j++;
  if (szText[i] == '0' && (szText[i+1] == 'x' || szText[i+1] == 'X')) { // interpret as HEX
    sscanf(&szText[i+2], "%x", &val);
  } else { // interpret as decimal
    val = atoi(&szText[i]);
  }
  *pValue = val;
  return j;
} /* delimitedValue() */

int tokenizeCMD(char *szText, int iLen)
{
int i = 0;
char szTemp[16];

  memcpy(szTemp, szText, iLen);
  szTemp[iLen] = 0;
  while (szCMDs[i]) {
     if (strcasecmp(szCMDs[i], szTemp) == 0) { // found it
       return i;
     }
     i++;
  }
  return -1; // not found
} /* tokenizeCMD() */

int parseCmd(char *szText, int *pData)
{
int iLen, i, iCount, iValue;
int iStart;

  iStart = 0; // starting offset to parse string
  iLen = strlen(szText);
  if (iLen == 0) return 0;

  i = delimitedValue(szText, &iValue, iLen-iStart); // first one must be the command
  pData[0] = tokenizeCMD(szText, i); // command is stored at data index 0
  if (pData[0] < 0 || pData[0] >= CMD_COUNT) {// invalid command
    Serial.print("Invalid command encountered: ");
    szText[i] = 0;
    Serial.println(szText);
    Serial.println("Type HELP for a list of commands");
    return 0;
  }
  iCount = 1; // output data index
  iStart += i;
  while (iStart < iLen) {
    i = delimitedValue(&szText[iStart], &iValue, iLen-iStart);
    pData[iCount++] = iValue;
    iStart += i;
  }
  return iCount; // number of numerical values parsed 
} /* parseCmd() */

// Display the help text
void showHelp()
{
  Serial.println("Interactive FastEPD gray matrix editor command list");
  Serial.println("(case insensitive, decimal numbers assumed, space or comma delimited)");
  Serial.println("HELP - This command list");
  Serial.println("LIST - show the current gray matrix values");
  Serial.println("SHOW - Display the gray matrix on the EPD panel");
  Serial.println("COPY n m - copy row n to row m");
  Serial.println("SWAP n m - swap the contents of row n with row m");
  Serial.println("EDIT n 0 1 2 2 1 0 0... write new values for row n");
  Serial.println("UNDO - undo the last change (only 1 step is reversible)");
} /* showHelp() */

// List the current values of the gray matrix in a form that can be easily copied
// back into your code
void ListMatrix(void)
{
  int i, j;
  uint8_t *s;
  Serial.println("Current Matrix:");
  // Show in a format that can be copied as code from the serial terminal
  Serial.println("const uint8_t u8_graytable[] = {");
  s = u8_graytable;
  for (i=0; i<16; i++) {
      Serial.printf("/* %d */  ", i);
      for (j=0; j<passes; j++) {
          if (i == 15 && j == passes-1) {
            Serial.printf("%d", *s++); // last value doesn't get a comma
          } else {
            Serial.printf("%d, ", *s++);
          }
      } // for j
      Serial.print("\n");
  } // for i
  Serial.print("};\n");
} /* ListMatrix() */

// Display the grayscale test image on the panel to see how your current gray matrix performs
void ShowMatrix(void)
{
  int i, rc;
    rc = epaper.setCustomMatrix(u8_graytable, sizeof(u8_graytable));
    if (rc != BBEP_SUCCESS) {
      Serial.printf("setCustomMatrix returned %d\n", rc);
      return;
    }
  epaper.fillScreen(0xf);
  for (i=0; i<800; i+=50) {
    epaper.fillRect(i, 0, 50, 250, i/50);
  }
  epaper.drawRect(0, 0, 800, 250, 0); // draw black outline around
  epaper.setFont(FONT_12x16);
  epaper.setTextColor(BBEP_BLACK);
  for (i=0; i<16; i++) {
    epaper.setCursor(i*50+12, 252);
    epaper.print(i, DEC);
  }
  epaper.fullUpdate();
} /* ShowMatrix() */

// Execute the command that the user typed
void executeCmd(int *pData, int iCount)
{
int i;
uint8_t *s, *d;

  switch (pData[0]) {
    case CMD_HELP:
      showHelp();
      break;
    case CMD_LIST:
      ListMatrix();
      break;
    case CMD_SHOW:
      ShowMatrix();
      break;
    case CMD_COPY:
      if (pData[1] < 0 || pData[1] > 15 || pData[2] < 0 || pData[2] > 15) {
        Serial.println("COPY: Source/Dest rows must be in the range 0 to 15");
        return;
      }
      memcpy(u8_last, u8_graytable, 16 * passes); // save current before changing it
      Serial.printf("Copying row %d to %d\n", pData[1], pData[2]);
      s = &u8_graytable[passes * pData[1]];
      d = &u8_graytable[passes * pData[2]];
      memcpy(d, s, passes);
      break;
    case CMD_SWAP:
      if (pData[1] < 0 || pData[1] > 15 || pData[2] < 0 || pData[2] > 15) {
        Serial.println("SWAP: Rows must be in the range 0 to 15");
        return;
      }
      memcpy(u8_last, u8_graytable, 16 * passes); // save current before changing it
      Serial.printf("Swapping row %d with %d\n", pData[1], pData[2]);
      s = &u8_graytable[passes * pData[1]];
      d = &u8_graytable[passes * pData[2]];
      memcpy(ucTemp, s, passes);
      memcpy(s, d, passes);
      memcpy(d, ucTemp, passes);
      break;
    case CMD_EDIT:
      for (i=0; i<iCount-2; i++) {
        if (pData[i+2] < 0 || pData[i+2] > 2) {
          Serial.println("EDIT: data values must be one of: 0=neutral, 1=darken, 2=lighten");
          return;
        } 
      } // for i
      if (pData[1] < 0 || pData[1] > 15 || iCount != passes+2) {
        Serial.printf("EDIT: Row must be in the range (0..15), and list length must be the number of passes (%d)\n", passes);
        return;
      }
      memcpy(u8_last, u8_graytable, 16 * passes); // save current before changing it
      d = &u8_graytable[pData[1] * passes];
      for (i=0; i<passes; i++) {
        d[i] = pData[i+2];
      }
      Serial.printf("EDIT: Row %d updated\n", pData[1]);
      break;
    case CMD_UNDO:
      memcpy(u8_graytable, u8_last, 16 * passes);
      Serial.println("UNDO - last change undone");
      break;
  } // switch
} /* executeCmd() */

void setup() {
  int rc;
  Serial.begin(115200);
  delay(3000); // wait for CDC serial to start
  passes = sizeof(u8_graytable) / 16;
  epaper.initPanel(BB_PANEL_EPDIY_V7_16);
  rc = epaper.setPanelSize(1872, 1404, BB_PANEL_FLAG_MIRROR_X);
  if (rc != BBEP_SUCCESS) {
      Serial.printf("setPanelSize returned %d\n", rc);
  }
  epaper.setMode(BB_MODE_4BPP);
  ShowMatrix();
  memcpy(u8_last, u8_graytable, 16 * passes); // in case the user tries to UNDO first :)
  Serial.println("Ready! Enter a command or HELP");
} /* setup() */

void loop() {
char szText[256];
int iData[32], iCount;

// Do this forever
  if (getCmd(szText)) {
    // timed out
    Serial.println("Enter a command or HELP");
    return;
  }
  iCount = parseCmd(szText, iData);
  if (iCount > 0) {
    executeCmd(iData, iCount);
  }
} /* loop() */