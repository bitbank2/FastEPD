//
// Eink Monitor Demo
// written by Larry Bank Nov 20, 2025
// Copyright(c) 2025 BitBank Software, Inc.
//
#include <drm.h>
#include <drm_fourcc.h>
#include <drm_mode.h>
#include <errno.h>
#include <fcntl.h>
#include <inttypes.h>
#include <sys/mman.h>
#include <unistd.h>
#include <xf86drm.h>
#include <xf86drmMode.h>
#include <unistd.h>
#include <pthread.h>
#include <FastEPD.h>
#include <arm_neon.h>
FASTEPD epaper;
volatile int bRunning = 0;
uint8_t *pGUI, *pFramebuffer;
int center_x, center_y, iInvert = 0;
// Variable that keeps count on how much screen has been partially updated
int n = 0;

//
// Convert the 16 or 32-bit RPI GUI desktop to 1-bit per pixel
//
void ConvertGUI(uint8_t *pGUI, int w, int h, int bpp)
{
uint8_t *d;
int iPitch;

    iPitch = (epaper.width() + 7)/8;
    if (bpp == 32) {
    uint8_t *s;
    s = pGUI;
    for (int y=0; y<epaper.height(); y++) {
        d = &pFramebuffer[y * iPitch];
	s = &pGUI[y * w * 4];
#ifdef OLD_WAY
	uc = 0; ucMask = 0x80;
        for (int x=0; x<epaper.width(); x++) {
            gray = s[0] + (s[1]*2) + s[2];
            s += 4;
            if (gray >= 512) uc |= ucMask;
            ucMask >>= 1;
            if (ucMask == 0) {
                *d++ = uc;
                ucMask = 0x80;
                uc = 0;
            }
        } // for x
#else // NEON
        static const uint8_t u8Mask[8] = {0x80,0x40,0x20,0x10,0x8,0x4,0x2,0x1};
        uint8x8_t u8pixels, u8x8Mask = vld1_u8(u8Mask);
        uint16x8_t u16_128 = vdupq_n_u16(384);
        uint16x8_t u16x8Invert = vdupq_n_u16((iInvert) ? 0xffff : 0x0000);
        for (int x=0; x<epaper.width(); x+=8) { // work 8 pixels at a time
            uint8x8x4_t pixels = vld4_u8(s);
            uint16x8_t grays;
            s += 32;
            grays = vaddl_u8(pixels.val[0], pixels.val[1]);
            grays = vaddw_u8(grays, pixels.val[2]);
            grays = vcgtq_u16(grays, u16_128); 
            grays = veorq_u16(grays, u16x8Invert); // optional inversion
            u8pixels = vreinterpret_u8_s8(vmovn_s16(vreinterpretq_s16_u16(grays))); // narrow to 8 bits
            u8pixels = vand_u8(u8pixels, u8x8Mask);
            *d++ = vaddv_u8(u8pixels); // final result 
        } // for x
#endif
    } // for y

    } else if (bpp == 16) {
        uint16_t *s;
//        uint8_t uc, ucInvert = (iInvert) ? 0xff : 0x00;
//	uc = 0;
        for (int y=0; y<epaper.height(); y++) {
            s = (uint16_t *)&pGUI[y * w * 2];
	    d = &pFramebuffer[y * iPitch];
#ifdef OLD_WAY
            for (int x=0; x<epaper.width(); x+=8) {
                uint16_t gray, u16;
                for (int j=0; j<8; j++) {
                    u16 = *s++;
                    gray = ((u16 & 0x7e0)>>5) + (u16 & 0x1f) + ((u16 & 0xf800) >> 11);
                    uc <<= 1;
		    uc |= (gray >= 80);
                } // for j
                *d++ = uc ^ ucInvert;
            } // for x
#else // use NEON
            const uint16x8_t u16mask5 = vdupq_n_u16(0x1f);
            const uint16x8_t u16mask6 = vdupq_n_u16(0x3f);
            const uint16x8_t u16cmp = vdupq_n_u16(70); // 7-bit white threshold
            const uint16_t u16masks[] = {0x80,0x40,0x20,0x10,0x08,0x04,0x02,0x01};
            const uint16x8_t u16bits = vld1q_u16(u16masks);
            const uint16x8_t u16invert = vdupq_n_u16((iInvert) ? 0xffff : 0x0000);
            for (int x=0; x<epaper.width(); x+= 16) { // work 16 pixels at a time
                uint16x8_t u16pixels0, u16pixels1, u16gray0, u16gray1;
                u16pixels0 = vld1q_u16(s);
                s += 8;
                u16pixels1 = vld1q_u16(s);
                s += 8;
                u16gray0 = vandq_u16(u16pixels0, u16mask5); // blue
                u16gray1 = vandq_u16(u16pixels1, u16mask5);
                u16gray0 = vaddq_u16(vshrq_n_u16(u16pixels0, 11), u16gray0); // red
                u16gray1 = vaddq_u16(vshrq_n_u16(u16pixels1, 11), u16gray1);
                u16pixels0 = vshrq_n_u16(u16pixels0, 5); // get green
                u16pixels1 = vshrq_n_u16(u16pixels1, 5);
                u16pixels0 = vandq_u16(u16pixels0, u16mask6); 
                u16pixels1 = vandq_u16(u16pixels1, u16mask6);
                u16gray0 = vaddq_u16(u16gray0, u16pixels0); // add green
                u16gray1 = vaddq_u16(u16gray1, u16pixels1);
                u16gray0 = vcgeq_u16(u16gray0, u16cmp); // threshold to 1-bit
                u16gray1 = vcgeq_u16(u16gray1, u16cmp);
                u16gray0 = veorq_u16(u16gray0, u16invert); // optional invert
                u16gray1 = veorq_u16(u16gray1, u16invert);
                u16gray0 = vandq_u16(u16gray0, u16bits); // convert to 1-bit pixels
                u16gray1 = vandq_u16(u16gray1, u16bits);
                *d++ = (uint8_t)vaddvq_u16(u16gray0); // merge pixels into 1 byte
                *d++ = (uint8_t)vaddvq_u16(u16gray1);
            } // for x
#endif
        } // for y
    } // 16-bpp
} /* ConvertGUI() */

static void *copy_thread(void *pArg)
{
drmModeFB *fb = (drmModeFB *)pArg;

    while (bRunning) {
        ConvertGUI(pGUI, fb->width, fb->height, fb->bpp); // convert RPI 16-bit desktop to 1-bit for Eink
	epaper.videoUpdate();
    }
    pthread_exit(NULL);
} /* copy_thread() */

int main(int argc, char *argv[])
{
int err, drm_fd, prime_fd;
unsigned int i, card;
uint32_t fb_id, crtc_id;
drmModePlaneRes *plane_res;
drmModePlane *plane;
drmModeFB *fb;
pthread_t host;
uint64_t has_dumb;
char buf[256];

    if (argc == 2) iInvert = 1; // any parameter turns on invert
    epaper.initPanel(BB_PANEL_RPI);
    epaper.setPanelSize(BBEP_DISPLAY_EC060TC1);
    epaper.fillScreen(BBEP_WHITE); // start with white
    epaper.fullUpdate(CLEAR_SLOW, true);
    pFramebuffer = epaper.currentBuffer(); // we want to write directly into the framebuffer (faster)

    for (card=0; card<=10; card++) {
	snprintf(buf, sizeof(buf), "/dev/dri/card%u", card);
        drm_fd = open(buf, O_RDWR | O_CLOEXEC);
        if (drm_fd < 0) {
            fprintf(stderr, "Could not open KMS/DRM device %s.\n", buf);
            return -1;
        }
        if (drmGetCap(drm_fd, DRM_CAP_DUMB_BUFFER, &has_dumb) >= 0 && has_dumb) {
		close(drm_fd);
		break;
	} else {
            close(drm_fd);
	}
    } // for each card
    drm_fd = open(buf, O_RDWR | O_CLOEXEC);
    if (drmSetClientCap(drm_fd, DRM_CLIENT_CAP_ATOMIC, 1)) {
        fprintf(stderr, "Unable to set atomic cap.\n");
        close(drm_fd);
        return -1;
    }
    if (drmSetClientCap(drm_fd, DRM_CLIENT_CAP_UNIVERSAL_PLANES, 1)) {
        fprintf(stderr, "Unable to set universal planes cap.\n");
        close(drm_fd);
        return -1;
    }
    plane_res = drmModeGetPlaneResources(drm_fd);
    if (!plane_res) {
        fprintf(stderr, "Unable to get plane resources.\n");
        close(drm_fd);
        return -1;
    }
    for (i = 0; i < plane_res->count_planes; i++) {
        plane = drmModeGetPlane(drm_fd, plane_res->planes[i]);
        fb_id = plane->fb_id;
        crtc_id = plane->crtc_id;
        drmModeFreePlane(plane);
        if (fb_id != 0 && crtc_id != 0) break;
    }
    if (i == plane_res->count_planes) {
        fprintf(stderr, "No planes found\n");
        drmModeFreePlaneResources(plane_res); 
        close(drm_fd);
        return -1;
    }
    fb = drmModeGetFB(drm_fd, fb_id);
    if (!fb) {
        fprintf(stderr, "Failed to get framebuffer %" PRIu32 ": %s\n",
                       fb_id, strerror(errno));
        drmModeFreePlaneResources(plane_res);
        close(drm_fd);
        return -1;
    }
    err = drmPrimeHandleToFD(drm_fd, fb->handle, O_RDONLY, &prime_fd);
    if (err < 0) {
        fprintf(stderr, "Failed to retrieve prime handler: %s\n",
                        strerror(-err));
        drmModeFreePlaneResources(plane_res);
        drmModeFreeFB(fb);
        close(drm_fd);
        return -1;
    }
    pGUI = (uint8_t *)mmap(NULL, (fb->bpp >> 3) * fb->width * fb->height,
                      PROT_READ, MAP_PRIVATE, prime_fd, 0);
    if (pGUI == MAP_FAILED) {
        fprintf(stderr, "Unable to mmap prime buffer\n");
        drmModeFreePlaneResources(plane_res);
        drmModeFreeFB(fb);
        close(drm_fd);
        return -1;
    }

    printf("Starting video copying of fb: %dx%dx%d-bpp\n", fb->width, fb->height, fb->bpp);
    bRunning = 1;
    // Start the thread which does the work
    pthread_create(&host, NULL, copy_thread, (void *)fb);
    printf("Press ENTER to quit\n");
    getchar();
    bRunning = 0;
    usleep(20000); // allow time for it to finish
    // clean up the handles
    munmap(pFramebuffer, (fb->bpp >> 3) * fb->width * fb->height);
    drmModeFreePlaneResources(plane_res);
    drmModeFreeFB(fb);
    close(drm_fd);
    // clear the display
    epaper.clearBlack(true);
    epaper.clearWhite(true);
    epaper.einkPower(false);
    epaper.deInit();
    return 0;
} /* main () */

