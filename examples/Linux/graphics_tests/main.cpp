//
//  FastEPD graphics API tests
//  Written by Larry Bank
//
//  A set of function tests to ensure all of the bb_epaper drawing functions work correctly
//
#define __MEM_ONLY__
#define __LINUX__
#include "../../../src/FastEPD.cpp"

FASTEPD bbep;

//
// Simple logging print
//
void BBLOG(int line, char *string, const char *result)
{
    printf("Line: %d: msg: %s%s\n", line, string, result);
} /* BBLOG() */
//
// Read a 1-bit pixel value (0/1) given the buffer pointer, x, y and bytes per line of the image
//
uint8_t GetPixel(uint8_t *s, int x, int y, int iPitch)
{
uint8_t uc;
    
    s += (y * iPitch);
    s += (x / 8);
    uc = s[0] << (x & 7);
    return (uc >> 7);
} /* GetPixel() */

#define EPD_WIDTH 800
#define EPD_HEIGHT 480
int main(int argc, const char * argv[]) {
    int i, j, rc, iPitch, iTotal;
    uint8_t *pCompare;
    uint16_t u16;
    BB_RECT rect;
    char *szTestName;
    int iTotalPass, iTotalFail;
    int iBufferSize;
    const char *szStart = " - START";

    iTotalPass = iTotalFail = iTotal = 0;
    bbep.initPanel(BB_PANEL_VIRTUAL);
    bbep.setPanelSize(EPD_WIDTH, EPD_HEIGHT); // create 1-bit black/white drawing surface
    iBufferSize = (EPD_WIDTH * EPD_HEIGHT)/8;
    iPitch = EPD_WIDTH/8;
    
    // Test 0 - fillscreen to black and white
    iTotal++;
    szTestName = (char *)"Fillscreen to black and white";
    BBLOG(__LINE__, szTestName, szStart);
    bbep.fillScreen(BBEP_WHITE);
    rc = BBEP_SUCCESS;
    pCompare = bbep.currentBuffer();
    for (i=0; i<iBufferSize; i++) {
        if (pCompare[i] != 0xff) { // make sure every byte is white (8 one bits = 0xff)
            rc = BBEP_ERROR_BAD_DATA;
            break;
        }
    }
    if (rc == BBEP_SUCCESS) {
        bbep.fillScreen(BBEP_BLACK);
        for (i=0; i<iBufferSize; i++) {
            if (pCompare[i] != 0x00) { // make sure every byte is black (8 zero bits = 0x00)
                rc = BBEP_ERROR_BAD_DATA;
                break;
            }
        }
    }
    if (rc == BBEP_SUCCESS) {
        iTotalPass++;
        BBLOG(__LINE__, szTestName, " - PASSED");
    } else {
        iTotalFail++;
        BBLOG(__LINE__, szTestName, " - FAILED");
    }
    // Test 1 - Test fillRect
    iTotal++;
    szTestName = (char *)"Check rectangle invalid parameters";
    BBLOG(__LINE__, szTestName, szStart);
    bbep.fillRect(0,0,EPD_WIDTH/2, EPD_HEIGHT, BBEP_BLACK); // left half black
    bbep.fillRect(EPD_WIDTH/2,0,EPD_WIDTH/2,EPD_HEIGHT, BBEP_WHITE); // right half white
    rc = BBEP_SUCCESS;
    pCompare = (uint8_t *)bbep.currentBuffer();
    for (int y=0; y<EPD_HEIGHT; y++) {
        uint8_t *s = pCompare + y * (EPD_WIDTH/8);
        for (int x=0; x<iPitch/2; x++) {
            // Test that the left half of the buffer is white and the right half is black
            if (s[x] != 0 || s[x+iPitch/2] != 0xff) {
                rc = BBEP_ERROR_BAD_DATA;
                break;
            }
        } // for x
    } // for y
    if (rc == BBEP_SUCCESS) {
        iTotalPass++;
        BBLOG(__LINE__, szTestName, " - PASSED");
    } else {
        iTotalFail++;
        BBLOG(__LINE__, szTestName, " - FAILED");
    }
    // Test 2 - Check rectangle invalid parameters
    iTotal++;
    szTestName = (char *)"Check rectangle invalid parameters";
    BBLOG(__LINE__, szTestName, szStart);
    bbep.clearLastError();
    bbep.drawRect(0,0,EPD_WIDTH, EPD_HEIGHT, BBEP_BLACK); // valid area
    if (bbep.getLastError() != BBEP_SUCCESS) rc = BBEP_ERROR_BAD_PARAMETER;
    bbep.drawRect(0,0,EPD_WIDTH+1,EPD_HEIGHT, BBEP_BLACK); // invalid area (too wide)
    if (bbep.getLastError() != BBEP_ERROR_BAD_PARAMETER) rc = BBEP_ERROR_BAD_PARAMETER;
    if (rc == BBEP_SUCCESS) {
        iTotalPass++;
        BBLOG(__LINE__, szTestName, " - PASSED");
    } else {
        iTotalFail++;
        BBLOG(__LINE__, szTestName, " - FAILED");
    }
    // Test 3 - Test drawLine
    iTotal++;
    szTestName = (char *)"Test drawLine";
    BBLOG(__LINE__, szTestName, szStart);
    bbep.fillScreen(BBEP_WHITE);
    pCompare = (uint8_t *)bbep.currentBuffer();
    // The Bresenham line algorithm is used internally and it has a X major path and Y major path
    // Test both paths by making specific lines
    
    // Make sure endpoints in both major directions are not ovverun
    bbep.drawLine(10, 10, 40, 20, BBEP_BLACK); // X major path
    rc = BBEP_SUCCESS;
    if (GetPixel(pCompare, 10, 10, iPitch) != 0) { // check the starting pixel is the correct color
        rc = BBEP_ERROR_BAD_DATA;
    }
    if (GetPixel(pCompare, 9, 10, iPitch) != 1) { // check that pixel to the left is not changed
        rc = BBEP_ERROR_BAD_DATA;
    }
    if (GetPixel(pCompare, 10, 9, iPitch) != 1) { // check that pixel above is not changed
        rc = BBEP_ERROR_BAD_DATA;
    }
    if (GetPixel(pCompare, 40, 20, iPitch) != 0) { // check that the end pixel is the correct color
        rc = BBEP_ERROR_BAD_DATA;
    }
    bbep.drawLine(40, 40, 50, 80, BBEP_BLACK); // Y major path
    if (GetPixel(pCompare, 40, 40, iPitch) != 0) { // check the starting pixel is the correct color
        rc = BBEP_ERROR_BAD_DATA;
    }
    if (GetPixel(pCompare, 41, 40, iPitch) != 1) { // check that pixel to the right is not changed
        rc = BBEP_ERROR_BAD_DATA;
    }
    if (GetPixel(pCompare, 40, 39, iPitch) != 1) { // check that pixel above is not changed
        rc = BBEP_ERROR_BAD_DATA;
    }
    if (GetPixel(pCompare, 50, 80, iPitch) != 0) { // check that the end pixel is the correct color
        rc = BBEP_ERROR_BAD_DATA;
    }
    if (rc == BBEP_SUCCESS) {
        iTotalPass++;
        BBLOG(__LINE__, szTestName, " - PASSED");
    } else {
        iTotalFail++;
        BBLOG(__LINE__, szTestName, " - FAILED");
    }
    // Test 4 - Test fillRoundRect vs drawRoundRect
    iTotal++;
    szTestName = (char *)"Test fillRoundRect vs drawRoundRect";
    BBLOG(__LINE__, szTestName, szStart);
    bbep.fillScreen(BBEP_WHITE);
    pCompare = (uint8_t *)bbep.currentBuffer();
    // Check that the inside is properly filled versus a regular round rect
    // There should be no gaps or 'holes' in the filled rectangle
    bbep.drawRoundRect(10,  10, 100, 100, 7, BBEP_BLACK);
    bbep.fillRoundRect(210, 10, 100, 100, 7, BBEP_BLACK);
    rc = BBEP_SUCCESS;
    for (int y=10; y<110; y++) {
        bool bInside = false;
        for (int x=0; x<110; x++) {
            if (bInside) {
                if (GetPixel(pCompare, x + 200, y, iPitch) != 0) { // inside of filled roundRect is not filled!
                    rc = BBEP_ERROR_BAD_DATA;
                    break;
                }
                if (GetPixel(pCompare, x, y, iPitch) == 0 && GetPixel(pCompare, x+1, y, iPitch) == 1) { // right edge of outline
                    break;
                }
            } else {
                if (GetPixel(pCompare, x, y, iPitch) == 0) { // start of inside portion
                    bInside = true;
                }
            }
        }
    }
    if (rc == BBEP_SUCCESS) {
        iTotalPass++;
        BBLOG(__LINE__, szTestName, " - PASSED");
    } else {
        iTotalFail++;
        BBLOG(__LINE__, szTestName, " - FAILED");
    }
    // Test 5 - Test fillCircle vs drawCircle
    iTotal++;
    szTestName = (char *)"Test fillCircle vs drawCircle";
    BBLOG(__LINE__, szTestName, szStart);
    bbep.fillScreen(BBEP_WHITE);
    pCompare = (uint8_t *)bbep.currentBuffer();
    // Check that the inside is properly filled versus a regular circle
    // There should be no gaps or 'holes' in the filled circle
    bbep.drawCircle(100, 100, 90, BBEP_BLACK);
    bbep.fillCircle(300, 100, 90, BBEP_BLACK);
    rc = BBEP_SUCCESS;
    for (int y=8; y<192; y++) {
        bool bInside = false;
        for (int x=8; x<192; x++) {
            if (bInside) {
                if (GetPixel(pCompare, x + 200, y, iPitch) != 0) { // inside of filled circle is not filled!
                    rc = BBEP_ERROR_BAD_DATA;
                    break;
                }
                if (GetPixel(pCompare, x, y, iPitch) == 0 && GetPixel(pCompare, x+1, y, iPitch) == 1) { // right edge of outline
                    break;
                }
            } else {
                if (GetPixel(pCompare, x, y, iPitch) == 0) { // start of inside portion
                    bInside = true;
                }
            }
        }
    }
    if (rc == BBEP_SUCCESS) {
        iTotalPass++;
        BBLOG(__LINE__, szTestName, " - PASSED");
    } else {
        iTotalFail++;
        BBLOG(__LINE__, szTestName, " - FAILED");
    }
    // Test 6 - Test fillEllipse vs drawEllipse
    iTotal++;
    szTestName = (char *)"Test fillEllipse vs drawEllipse";
    BBLOG(__LINE__, szTestName, szStart);
    bbep.fillScreen(BBEP_WHITE);
    pCompare = (uint8_t *)bbep.currentBuffer();
    // Check that the inside is properly filled versus a regular ellipse
    // There should be no gaps or 'holes' in the filled ellipse
    bbep.drawEllipse(100, 50, 90, 40, BBEP_BLACK);
    bbep.fillEllipse(300, 50, 90, 50, BBEP_BLACK);
    rc = BBEP_SUCCESS;
    for (int y=8; y<92; y++) {
        bool bInside = false;
        for (int x=8; x<192; x++) {
            if (bInside) {
                if (GetPixel(pCompare, x + 200, y, iPitch) != 0) { // inside of filled circle is not filled!
                    rc = BBEP_ERROR_BAD_DATA;
                    break;
                }
                if (GetPixel(pCompare, x, y, iPitch) == 0 && GetPixel(pCompare, x+1, y, iPitch) == 1) { // right edge of outline
                    break;
                }
            } else {
                if (GetPixel(pCompare, x, y, iPitch) == 0) { // start of inside portion
                    bInside = true;
                }
            }
        }
    }
    if (rc == BBEP_SUCCESS) {
        iTotalPass++;
        BBLOG(__LINE__, szTestName, " - PASSED");
    } else {
        iTotalFail++;
        BBLOG(__LINE__, szTestName, " - FAILED");
    }
    // Test 7 - Test setFont with enumerated values
    iTotal++;
    szTestName = (char *)"Test setFont with enumerated values";
    BBLOG(__LINE__, szTestName, szStart);
    rc = BBEP_SUCCESS;
    for (int i=FONT_6x8; i<FONT_COUNT; i++) {
        if (bbep.setFont(i) != BBEP_SUCCESS) { // test all valid font enumerated values
            rc = BBEP_ERROR_BAD_PARAMETER;
            break;
        }
    }
    // Check invalid font enumerated values
    if (bbep.setFont(-1) != BBEP_ERROR_BAD_PARAMETER || bbep.setFont(FONT_COUNT) != BBEP_ERROR_BAD_PARAMETER) rc = BBEP_ERROR_BAD_PARAMETER;
    if (rc == BBEP_SUCCESS) {
        iTotalPass++;
        BBLOG(__LINE__, szTestName, " - PASSED");
    } else {
        iTotalFail++;
        BBLOG(__LINE__, szTestName, " - FAILED");
    }
    // Test 8 - Test setFont with binary data
    iTotal++;
    szTestName = (char *)"Test setFont with binary data";
    BBLOG(__LINE__, szTestName, szStart);
    rc = BBEP_SUCCESS;
    u16 = 0; // bad font signature
    if (bbep.setFont((void *)&u16) != BBEP_ERROR_BAD_PARAMETER) rc = BBEP_ERROR_BAD_PARAMETER;
    u16 = BB_FONT_MARKER; // valid font header marker
    if (bbep.setFont((void *)&u16) == BBEP_ERROR_BAD_PARAMETER) rc = BBEP_ERROR_BAD_PARAMETER;
    u16 = BB_FONT_MARKER_SMALL; // valid font header marker (small font)
    if (bbep.setFont((void *)&u16) == BBEP_ERROR_BAD_PARAMETER) rc = BBEP_ERROR_BAD_PARAMETER;
    if (rc == BBEP_SUCCESS) {
        iTotalPass++;
        BBLOG(__LINE__, szTestName, " - PASSED");
    } else {
        iTotalFail++;
        BBLOG(__LINE__, szTestName, " - FAILED");
    }
    // Test 9 - Test printing with the built-in font
    iTotal++;
    szTestName = (char *)"Test printing with the built-in font";
    BBLOG(__LINE__, szTestName, szStart);
    rc = BBEP_SUCCESS;
    bbep.fillScreen(BBEP_WHITE);
    pCompare = (uint8_t *)bbep.currentBuffer();
    bbep.setFont(FONT_8x8);
    bbep.setTextColor(BBEP_BLACK);
    bbep.setCursor(3, 3); // draw on odd byte boundary
    bbep.print("Hello"); // simple text to see if the built-in font draws correctly on odd pixel boundaries
    bbep.setCursor(0, 16);
    bbep.print("Hello"); // simple text to see if the built-in font draws correctly on odd pixel boundaries
    for (int y=3; y<11; y++) {
        for (int x=3; x<43; x++) {
            if (GetPixel(pCompare, x, y, iPitch) != GetPixel(pCompare, x-3, y+13, iPitch)) {
                rc = BBEP_ERROR_BAD_DATA;
                break;
            }
        }
    }
    if (rc == BBEP_SUCCESS) {
        iTotalPass++;
        BBLOG(__LINE__, szTestName, " - PASSED");
    } else {
        iTotalFail++;
        BBLOG(__LINE__, szTestName, " - FAILED");
    }
    // Test 10 - Test font clipping to EPD bounds
    iTotal++;
    szTestName = (char *)"Test font clipping to EPD bounds";
    BBLOG(__LINE__, szTestName, szStart);
    rc = BBEP_SUCCESS;
    bbep.fillScreen(BBEP_WHITE);
    pCompare = (uint8_t *)bbep.currentBuffer();
    bbep.setFont(FONT_8x8);
    bbep.setTextColor(BBEP_BLACK);
    bbep.setCursor(780, 477); // draw in the bottom right corner
    bbep.print("Hello");
    i = 0; // total black pixels
    j = 0; // count
    for (int y=477; y<480; y++) {
        for (int x=780; x<800; x++) {
            j++;
            if (GetPixel(pCompare, x, y, iPitch) == 0) {
                i++; // count black pixels
            }
        }
    }
    if (i == 0 || i == j) rc = BBEP_ERROR_BAD_DATA; // can't be all white or all black
    // Check that pixels were not drawn "wrapped around" the right edge
    for (int y=477; y<480; y++) {
        for (int x=0; x<20; x++) {
            if (GetPixel(pCompare, x, y, iPitch) == 0) {
                rc = BBEP_ERROR_BAD_DATA; // a stray pixel was drawn on the window's left side
                break;
            }
        }
    }
    if (rc == BBEP_SUCCESS) {
        iTotalPass++;
        BBLOG(__LINE__, szTestName, " - PASSED");
    } else {
        iTotalFail++;
        BBLOG(__LINE__, szTestName, " - FAILED");
    }
    // Test 11 - Test drawString vs print
    iTotal++;
    szTestName = (char *)"Test drawString vs print";
    BBLOG(__LINE__, szTestName, szStart);
    rc = BBEP_SUCCESS;
    bbep.fillScreen(BBEP_WHITE);
    pCompare = (uint8_t *)bbep.currentBuffer();
    bbep.setFont(FONT_8x8);
    bbep.setTextColor(BBEP_BLACK);
    bbep.setCursor(0, 0);
    bbep.print("Hello");
    bbep.drawString("Hello", 0, 8); // draw the same text below it
    i = 0; // total black pixels
    j = 0; // count
    for (int y=0; y<8; y++) {
        for (int x=0; x<40; x++) {
            j++;
            if (GetPixel(pCompare, x, y, iPitch) == 0) {
                i++; // count black pixels
            }
            if (GetPixel(pCompare, x, y, iPitch) != GetPixel(pCompare, x, y+8, iPitch)) {
                rc = BBEP_ERROR_BAD_DATA; // print and drawString created different results
                break;
            }
        }
    }
    if (i == 0 || i == j) rc = BBEP_ERROR_BAD_DATA; // can't be all white or all black
    if (rc == BBEP_SUCCESS) {
        iTotalPass++;
        BBLOG(__LINE__, szTestName, " - PASSED");
    } else {
        iTotalFail++;
        BBLOG(__LINE__, szTestName, " - FAILED");
    }
    // Test 12 - Test sprite drawing
    iTotal++;
    szTestName = (char *)"Test sprite drawing";
    BBLOG(__LINE__, szTestName, szStart);
    {
        FASTEPD sprite;
        uint8_t *pCompare2;
        rc = BBEP_SUCCESS;
        sprite.initSprite(64, 32);
        pCompare2 = (uint8_t *)sprite.currentBuffer();
        sprite.fillScreen(BBEP_BLACK);
        // Draw a white X through the sprite (sprite active color is only white)
        sprite.drawLine(0,0, 63, 31, BBEP_WHITE);
        sprite.drawLine(0,63, 0, 31, BBEP_WHITE);
        bbep.fillScreen(BBEP_BLACK);
        pCompare = (uint8_t *)bbep.currentBuffer();
        bbep.drawSprite(&sprite, 0, 0);
        i = 0; // total black pixels
        j = 0; // count
        for (int y=0; y<32; y++) {
            for (int x=0; x<64; x++) {
                j++;
                if (GetPixel(pCompare, x, y, iPitch) == 0) {
                    i++; // count black pixels
                }
                if (GetPixel(pCompare, x, y, iPitch) != GetPixel(pCompare2, x, y, 64/8)) {
                    rc = BBEP_ERROR_BAD_DATA; // print and drawString created different results
                    break;
                }
            }
        }
        if (i == 0 || i == j) rc = BBEP_ERROR_BAD_DATA; // can't be all white or all black
        if (rc == BBEP_SUCCESS) {
            iTotalPass++;
            BBLOG(__LINE__, szTestName, " - PASSED");
        } else {
            iTotalFail++;
            BBLOG(__LINE__, szTestName, " - FAILED");
        }
    }
    // Test 13 - Check getStringBox for different size fonts
    iTotal++;
    szTestName = (char *)"Check getStringBox for different size fonts";
    BBLOG(__LINE__, szTestName, szStart);
    rc = BBEP_SUCCESS;
    bbep.setFont(FONT_8x8);
    bbep.getStringBox("Hello World!", &rect);
    if (rect.w != 12*8 || rect.h != 8) rc = BBEP_ERROR_BAD_DATA;
    bbep.setFont(FONT_12x16);
    bbep.getStringBox("Hello World!", &rect);
    if (rect.w != 12*12 || rect.h != 16) rc = BBEP_ERROR_BAD_DATA;
    if (rc == BBEP_SUCCESS) {
        iTotalPass++;
        BBLOG(__LINE__, szTestName, " - PASSED");
    } else {
        iTotalFail++;
        BBLOG(__LINE__, szTestName, " - FAILED");
    }
    // Test 14 - Check drawPixel clips properly for EPD size
    iTotal++;
    szTestName = (char *)"Check drawPixel clips properly for EPD size";
    BBLOG(__LINE__, szTestName, szStart);
    rc = BBEP_SUCCESS;
    bbep.fillScreen(BBEP_WHITE);
    pCompare = (uint8_t *)bbep.currentBuffer();
    for (i=460; i<500; i++) {
        bbep.drawPixel(i+300, i, BBEP_BLACK); // a total of 20 black pixels should be drawn
    }
    i = 0; // total black pixels
    for (int y=0; y<EPD_HEIGHT; y++) {
        for (int x=0; x<EPD_WIDTH; x++) {
            if (GetPixel(pCompare, x, y, iPitch) == 0) {
                i++; // count black pixels
            }
        }
    }
    if (i != 20) rc = BBEP_ERROR_BAD_DATA; // can't be all white or all black
    if (rc == BBEP_SUCCESS) {
        iTotalPass++;
        BBLOG(__LINE__, szTestName, " - PASSED");
    } else {
        iTotalFail++;
        BBLOG(__LINE__, szTestName, " - FAILED");
    }
    printf("Total tests: %d, %d passed, %d failed\n", iTotal, iTotalPass, iTotalFail);

    if (iTotal == iTotalPass) {
        return EXIT_SUCCESS;
    }
    return -1; // something failed
}
