// display_e1004.cpp
//
// Dedicated display backend for the Seeed Studio reTerminal E1004.
//
//   - 13.3" Spectra 6 ePaper, 1200 x 1600 (Black/White/Red/Green/Blue/Yellow)
//   - Panel controller: T133A01 (dual-chip, each chip drives half the width)
//   - Host MCU: ESP32-S3 (N32R8, 32 MB flash / 8 MB OPI PSRAM)
//
// This file REPLACES display.cpp for the E1004 board only (see the
// build_src_filter in the [env:seeed_reTerminal_E1004] PlatformIO target).
// It implements the same public API declared in include/display.h, but uses
// the GxEPD2 graphics stack together with the vendored GxEPD2_T133A01 driver
// (lib/GxEPD2_T133A01) instead of bb_epaper / FastEPD.
//
// Only the core display functions are provided here.  The TRMNL-X-only helpers
// (otg_*, enter_shipment_sleep, BQ27427_*, check_usb_power, ...) are guarded by
// BOARD_TRMNL_X at their call sites and are therefore never referenced on E1004.

#ifdef BOARD_SEEED_RETERMINAL_E1004

#include <Arduino.h>
#include <SPI.h>
#include <SPIFFS.h>
#include <Preferences.h>
#include <PNGdec.h>

#include <GxEPD2_7C.h>
#include "GxEPD2_T133A01_1200x1600.h"

#include <Fonts/FreeSansBold24pt7b.h>
#include <Fonts/FreeSans12pt7b.h>
#include <Fonts/FreeSans9pt7b.h>

#include <display.h>
#include <config.h>
#include <trmnl_log.h>
#include "DEV_Config.h"

#ifndef FW_VERSION_STRING
#define FW_VERSION_STRING "unknown"
#endif

// ---------------------------------------------------------------------------
// Panel instance
// ---------------------------------------------------------------------------

// Dedicated HSPI bus for the panel (matches the manufacturer reference).
static SPIClass hspi(HSPI);

// Paged drawing page buffer sizing: (WIDTH/2) * page_height bytes.
// 24000 bytes -> page_height = 40 rows -> 40 pages for the 1600px tall panel.
#define MAX_DISPLAY_BUFFER_SIZE 24000u
#define MAX_HEIGHT(EPD)                                            \
    (EPD::HEIGHT <= (MAX_DISPLAY_BUFFER_SIZE) / (EPD::WIDTH / 2)   \
         ? EPD::HEIGHT                                             \
         : (MAX_DISPLAY_BUFFER_SIZE) / (EPD::WIDTH / 2))

static GxEPD2_7C<GxEPD2_T133A01_1200x1600, MAX_HEIGHT(GxEPD2_T133A01_1200x1600)>
    display(GxEPD2_T133A01_1200x1600(EPD_CS_PIN, EPD_DC_PIN, EPD_RST_PIN,
                                     EPD_BUSY_PIN, EPD_CS1_PIN, EPD_EN_PIN));

// Runtime control for light sleep (mirrors display.cpp semantics).
static bool g_light_sleep_enabled = true;

// ---------------------------------------------------------------------------
// Color handling (RGB -> Spectra 6)
// ---------------------------------------------------------------------------

// Reference RGB values for the six Spectra colors.  As in display.cpp's E1002
// path these are not the exact panel primaries, but give the best simple
// nearest-color matching results.
struct SpectraColor {
    uint8_t r, g, b;
    uint16_t gx; // GxEPD2 RGB565 color constant
};

static const SpectraColor kSpectra[6] = {
    {  0,   0,   0, GxEPD_BLACK },
    {192, 192, 192, GxEPD_WHITE },
    {192, 192,   0, GxEPD_YELLOW},
    {192,   0,   0, GxEPD_RED   },
    {  0,   0, 192, GxEPD_BLUE  },
    {  0, 192,   0, GxEPD_GREEN },
};

// Map an arbitrary RGB triple to the closest Spectra 6 GxEPD2 color constant.
static inline uint16_t spectra_nearest(int r, int g, int b)
{
    int best = 0;
    long best_dist = 0x7fffffffL;
    for (int i = 0; i < 6; i++) {
        long dr = r - kSpectra[i].r;
        long dg = g - kSpectra[i].g;
        long db = b - kSpectra[i].b;
        long d = dr * dr + dg * dg + db * db;
        if (d < best_dist) {
            best_dist = d;
            best = i;
        }
    }
    return kSpectra[best].gx;
}

// GxEPD2 RGB565 color -> 7-color controller index, matching the private
// color7() in GxEPD2_7C so that rows packed here line up with the driver's
// native mapping.
static inline uint8_t gx_color7(uint16_t color)
{
    switch (color) {
        case GxEPD_BLACK:  return 0x00;
        case GxEPD_WHITE:  return 0x01;
        case GxEPD_GREEN:  return 0x02;
        case GxEPD_BLUE:   return 0x03;
        case GxEPD_RED:    return 0x04;
        case GxEPD_YELLOW: return 0x05;
        case GxEPD_ORANGE: return 0x06;
        default:           return 0x01; // white fallback
    }
}

// ---------------------------------------------------------------------------
// PNG decoding straight into the driver's full-screen framebuffer
// ---------------------------------------------------------------------------

// One packed 4bpp row (two pixels per byte) for the full panel width.
static uint8_t s_rowbuf[GxEPD2_T133A01_1200x1600::WIDTH / 2];

// Centering offsets for images smaller than the panel.
static int s_img_x_offset = 0;
static int s_img_y_offset = 0;

static inline void row_set_pixel(int x, uint8_t idx)
{
    if (x < 0 || x >= GxEPD2_T133A01_1200x1600::WIDTH) return;
    uint32_t i = (uint32_t)x >> 1;
    if (x & 1) s_rowbuf[i] = (s_rowbuf[i] & 0xF0) | (idx & 0x0F);
    else       s_rowbuf[i] = (s_rowbuf[i] & 0x0F) | ((idx & 0x0F) << 4);
}

// PNGDraw callback: decode one source line into Spectra colors and push it
// directly into the panel framebuffer via writeNative (1 row at a time).
static int png_draw_e1004(PNGDRAW *pDraw)
{
    const int panelW = GxEPD2_T133A01_1200x1600::WIDTH;
    const int panelH = GxEPD2_T133A01_1200x1600::HEIGHT;

    int y = pDraw->y + s_img_y_offset;
    if (y < 0 || y >= panelH) return 1;

    uint8_t *s = pDraw->pPixels;
    uint8_t *pPalette = pDraw->pPalette;
    int iBpp = pDraw->iBpp;

    switch (pDraw->iPixelType) {
        case PNG_PIXEL_TRUECOLOR:        if (iBpp <= 8) iBpp *= 3; pPalette = NULL; break;
        case PNG_PIXEL_TRUECOLOR_ALPHA:  if (iBpp <= 8) iBpp *= 4; pPalette = NULL; break;
        case PNG_PIXEL_GRAYSCALE:        pPalette = NULL; break;
        case PNG_PIXEL_INDEXED:          break;
    }
    int iDelta = iBpp / 8;

    // Start the row as white so uncovered pixels stay white.
    memset(s_rowbuf, (gx_color7(GxEPD_WHITE) << 4) | gx_color7(GxEPD_WHITE),
           sizeof(s_rowbuf));

    int maxX = pDraw->iWidth;
    if (maxX > panelW - s_img_x_offset) maxX = panelW - s_img_x_offset;

    uint8_t r = 0, g = 0, b = 0, *pPal;
    for (int x = 0; x < pDraw->iWidth; x++) {
        switch (iBpp) {
            case 24:
            case 32:
                r = s[0]; g = s[1]; b = s[2];
                s += iDelta;
                break;
            case 16:
                r = s[1] & 0xf8;
                g = ((s[0] | s[1] << 8) >> 3) & 0xfc;
                b = s[0] << 3;
                s += 2;
                break;
            case 8:
                if (pPalette) { pPal = &pPalette[s[0] * 3]; r = pPal[0]; g = pPal[1]; b = pPal[2]; }
                else { r = g = b = s[0]; }
                s++;
                break;
            case 4:
                if (pPalette) {
                    if (x & 1) { pPal = &pPalette[(s[0] & 0xf) * 3]; s++; }
                    else       { pPal = &pPalette[(s[0] >> 4) * 3]; }
                    r = pPal[0]; g = pPal[1]; b = pPal[2];
                } else {
                    if (x & 1) { r = g = b = (s[0] & 0xf) | (s[0] << 4); s++; }
                    else       { r = g = b = (s[0] >> 4) | (s[0] & 0xf0); }
                }
                break;
            case 2:
                if (pPalette) { pPal = &pPalette[((s[0] >> ((3 - (x & 3)) * 2)) & 3) * 3]; r = pPal[0]; g = pPal[1]; b = pPal[2]; }
                else { r = g = b = (s[0] << ((x & 3) * 2)) & 0xc0; }
                if ((x & 3) == 3) s++;
                break;
            case 1:
                if (pPalette) { pPal = &pPalette[((s[0] >> (7 - (x & 7))) & 1) * 3]; r = pPal[0]; g = pPal[1]; b = pPal[2]; }
                else { r = g = b = ((s[0] << (x & 7)) & 0x80) ? 0xff : 0x00; }
                if ((x & 7) == 7) s++;
                break;
        }
        if (x < maxX) {
            uint16_t gx = spectra_nearest(r, g, b);
            row_set_pixel(x + s_img_x_offset, gx_color7(gx));
        }
    }

    // Push this single row into the driver framebuffer (no refresh yet).
    display.epd2.writeNative(s_rowbuf, nullptr, 0, y, panelW, 1);
    return 1;
}

// ---------------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------------

void display_init(void)
{
    Log_info("E1004 display init start (T133A01 1200x1600 Spectra 6)");
    hspi.begin(EPD_SCK_PIN, EPD_MISO_PIN, EPD_MOSI_PIN, -1);
    display.epd2.selectSPI(hspi, SPISettings(10000000, MSBFIRST, SPI_MODE0));
    display.init(115200);     // bring up panel, enable driver diagnostics
    display.setRotation(0);   // native portrait 1200x1600
    display.setTextWrap(false);
    Log_info("E1004 display init end (%dx%d)", display.width(), display.height());
}

uint16_t display_height()
{
    return display.height();
}

uint16_t display_width()
{
    return display.width();
}

void display_set_light_sleep(uint8_t enabled)
{
    g_light_sleep_enabled = (enabled != 0);
}

void display_sleep(void)
{
    // Put the panel into its lowest-power state until the next full refresh.
    display.hibernate();
}

void display_reset(void)
{
    Log_info("E1004 e-Paper clear start");
    display.setFullWindow();
    display.firstPage();
    do {
        display.fillScreen(GxEPD_WHITE);
    } while (display.nextPage());
    Log_info("E1004 e-Paper clear end");
}

uint8_t *display_read_file(const char *filename, int *file_size)
{
    File f = SPIFFS.open(filename, "r");
    uint8_t *buffer;

    if (!f) {
        Log_error("E1004: failed to open file %s", filename);
        *file_size = 0;
        return nullptr;
    }
    *file_size = f.size();
    if (*file_size == 0) {
        Log_error("E1004: file %s is empty", filename);
        f.close();
        return nullptr;
    }
    Log_info("E1004: allocating %d bytes for %s", *file_size, filename);
    buffer = (uint8_t *)ps_malloc(*file_size);
    if (!buffer) {
        buffer = (uint8_t *)malloc(*file_size);
    }
    if (!buffer) {
        Log_error("E1004: file buffer allocation failed");
        *file_size = 0;
        f.close();
        return nullptr;
    }
    f.read(buffer, *file_size);
    f.close();
    return buffer;
}

void display_show_image(uint8_t *image_buffer, int data_size, bool bWait)
{
    (void)bWait; // a Spectra 6 full refresh always blocks until complete

    if (!image_buffer || data_size < 4) {
        Log_error("E1004: display_show_image called with no data");
        return;
    }

    bool isPNG = (image_buffer[0] == 0x89 && image_buffer[1] == 'P' &&
                  image_buffer[2] == 'N' && image_buffer[3] == 'G');
    if (!isPNG) {
        // Logos / loading screens are G5-compressed bb_epaper bitmaps which the
        // GxEPD2 stack cannot decode.  Show a neutral white screen instead of
        // garbage so the device stays in a defined visual state.
        Log_error("E1004: non-PNG image (G5/BMP) not supported, clearing screen");
        display_reset();
        return;
    }

    PNG *png = new PNG();
    if (!png) {
        Log_error("E1004: PNG decoder allocation failed");
        return;
    }

    int rc = png->openRAM((uint8_t *)image_buffer, data_size, png_draw_e1004);
    if (rc != PNG_SUCCESS) {
        Log_error("E1004: png openRAM failed (%d)", rc);
        delete png;
        return;
    }

    int w = png->getWidth();
    int h = png->getHeight();
    Log_info("E1004: decoding %dx%d PNG (%d-bpp)", w, h, png->getBpp());

    // Center the image on the panel; areas outside stay white.
    s_img_x_offset = (w < (int)display.width())  ? (display.width()  - w) / 2 : 0;
    s_img_y_offset = (h < (int)display.height()) ? (display.height() - h) / 2 : 0;

    // Start from a clean white framebuffer so areas outside the image stay
    // white.  writeScreenBuffer only fills the framebuffer (no panel refresh).
    display.epd2.writeScreenBuffer(0xFF);

    rc = png->decode(NULL, 0);
    png->close();
    if (rc != PNG_SUCCESS) {
        Log_error("E1004: png decode failed (%d)", rc);
    }
    delete png;

    // Push the completed framebuffer to the panel (blocking full refresh).
    display.epd2.refresh(false);
    display.epd2.powerOff();
    Log_info("E1004: image refresh complete");
}

// ---------------------------------------------------------------------------
// Text / message helpers (Adafruit_GFX via GxEPD2_7C)
// ---------------------------------------------------------------------------

// Draw a single horizontally-centered line of text at baseline y.
static void draw_centered(const char *text, int16_t y, const GFXfont *font, uint16_t color)
{
    display.setFont(font);
    display.setTextColor(color);
    int16_t x1, y1; uint16_t tw, th;
    display.getTextBounds(text, 0, 0, &x1, &y1, &tw, &th);
    int16_t x = ((int)display.width() - (int)tw) / 2 - x1;
    display.setCursor(x, y);
    display.print(text);
}

// Render up to a few centered lines as a full-screen message and refresh.
static void show_message_lines(const char *title,
                               const char *line1 = nullptr,
                               const char *line2 = nullptr,
                               const char *line3 = nullptr)
{
    const int cx = (int)display.height() / 2; // vertical center (portrait)
    display.setFullWindow();
    display.firstPage();
    do {
        display.fillScreen(GxEPD_WHITE);
        if (title) draw_centered(title, cx - 120, &FreeSansBold24pt7b, GxEPD_BLACK);
        if (line1) draw_centered(line1, cx - 30,  &FreeSans12pt7b,    GxEPD_BLACK);
        if (line2) draw_centered(line2, cx + 20,  &FreeSans12pt7b,    GxEPD_BLACK);
        if (line3) draw_centered(line3, cx + 70,  &FreeSans12pt7b,    GxEPD_BLACK);
    } while (display.nextPage());
    display.powerOff();
}

// Map a MSG enum to title + up to three text lines.
static void message_text_for(MSG message_type, const char *message_text,
                             const char **title,
                             const char **l1, const char **l2, const char **l3)
{
    *title = "TRMNL";
    *l1 = *l2 = *l3 = nullptr;

    switch (message_type) {
        case WIFI_CONNECT:
            *title = "Set up Wi-Fi";
            *l1 = "Connect to the \"TRMNL\" Wi-Fi network";
            *l2 = "on your phone or computer.";
            break;
        case WIFI_FAILED:
            *title = "Wi-Fi failed";
            *l1 = "Can't establish a Wi-Fi connection.";
            *l2 = "Hold the button to reset Wi-Fi.";
            break;
        case WIFI_WEAK:
            *title = "Weak Wi-Fi";
            *l1 = "Connected, but the signal is weak.";
            break;
        case WIFI_INTERNAL_ERROR:
            *title = "Connection error";
            *l1 = "Wi-Fi connected, but the API";
            *l2 = "connection could not be established.";
            break;
        case API_ERROR:
        case API_REQUEST_FAILED:
        case API_UNABLE_TO_CONNECT:
            *title = "Server error";
            *l1 = "Wi-Fi connected, request to API failed.";
            *l2 = "Check your internet connection.";
            break;
        case API_SETUP_FAILED:
            *title = "Setup failed";
            *l1 = "The /api/setup request returned an error.";
            break;
        case API_SIZE_ERROR:
        case MSG_FORMAT_ERROR:
            *title = "Content error";
            *l1 = "The TRMNL content was malformed.";
            break;
        case MSG_TOO_BIG:
            *title = "Image too large";
            *l1 = "The downloaded image is too big.";
            break;
        case MAC_NOT_REGISTERED:
            *title = "Not registered";
            *l1 = "This device is not registered.";
            break;
        case FW_UPDATE:
            *title = "Updating";
            *l1 = "Installing a firmware update...";
            break;
        case FW_UPDATE_FAILED:
            *title = "Update failed";
            *l1 = "The firmware update did not complete.";
            break;
        case FW_UPDATE_SUCCESS:
            *title = "Update complete";
            *l1 = "Firmware updated successfully.";
            break;
        case WIFI_RESET_CONFIRM:
            *title = "Reset Wi-Fi?";
            *l1 = "Hold the button to confirm,";
            *l2 = "tap to cancel.";
            break;
        case POWER_OFF_CONFIRM:
            *title = "Turn off device?";
            *l1 = "Hold the button to confirm,";
            *l2 = "tap to cancel.";
            break;
        case FILL_WHITE:
            *title = nullptr;
            break;
        case TEST:
            *title = "TEST";
            break;
        default:
            *title = DEVICE_MODEL;
            if (message_text && message_text[0]) *l1 = message_text;
            break;
    }
}

void display_show_msg(uint8_t *image_buffer, MSG message_type, const char *message_text)
{
    (void)image_buffer;

    if (message_type == FILL_WHITE) {
        display_reset();
        return;
    }

    const char *title, *l1, *l2, *l3;
    message_text_for(message_type, message_text, &title, &l1, &l2, &l3);
    show_message_lines(title, l1, l2, l3);
}

void display_show_msg(uint8_t *image_buffer, MSG message_type, String friendly_id,
                      bool id, const char *fw_version, String message)
{
    (void)image_buffer;
    (void)fw_version;

    if (message_type == FRIENDLY_ID) {
        String l1 = String("Device ID: ") + friendly_id;
        show_message_lines("Welcome to TRMNL",
                           id ? l1.c_str() : "Setting up your device...",
                           message.length() ? message.c_str() : nullptr);
        return;
    }

    const char *title, *l1, *l2, *l3;
    const char *extra = message.length() ? message.c_str() : nullptr;
    message_text_for(message_type, extra, &title, &l1, &l2, &l3);
    // Prefer an explicit API message as the first body line when present.
    if (extra) l1 = extra;
    show_message_lines(title, l1, l2, l3);
}

void display_show_msg_api(uint8_t *image_buffer, String message)
{
    (void)image_buffer;
    show_message_lines("TRMNL", message.length() ? message.c_str() : nullptr);
}

void display_show_msg_qa(uint8_t *image_buffer, const float *voltage,
                         const float *temperature, bool qa_result)
{
    (void)image_buffer;

    char vline[96];
    char tline[96];
    snprintf(vline, sizeof(vline), "V: %.3f -> %.3f (d %.3f)",
             voltage[0], voltage[1], voltage[2]);
    snprintf(tline, sizeof(tline), "T: %.2f -> %.2f (d %.2f)",
             temperature[0], temperature[1], temperature[2]);

    const int cx = (int)display.height() / 2;
    display.setFullWindow();
    display.firstPage();
    do {
        display.fillScreen(GxEPD_WHITE);
        draw_centered(qa_result ? "PASS" : "FAIL", cx - 140,
                      &FreeSansBold24pt7b, qa_result ? GxEPD_GREEN : GxEPD_RED);
        draw_centered(vline, cx - 20, &FreeSans9pt7b, GxEPD_BLACK);
        draw_centered(tline, cx + 20, &FreeSans9pt7b, GxEPD_BLACK);
        draw_centered(qa_result ? "Press button to clear screen"
                                : "QA failed - set this board aside",
                      cx + 70, &FreeSans9pt7b, GxEPD_BLACK);
    } while (display.nextPage());
    display.powerOff();
}

void display_show_battery(float f)
{
    char line[48];
    snprintf(line, sizeof(line), "Battery: %.3f V", f);
    show_message_lines("Battery", line);
}

void Paint_DrawMultilineText(UWORD x_start, UWORD y_start, const char *message,
                             uint16_t max_width, uint16_t font_width,
                             UWORD color_fg, UWORD color_bg, void *font,
                             bool is_center_aligned)
{
    (void)font;
    (void)font_width;
    (void)color_bg;
    (void)is_center_aligned;

    // GxEPD2 colors are RGB565; the callers pass bb_epaper indices that are not
    // meaningful here, so render in black for legibility.
    (void)color_fg;

    display.setFont(&FreeSans12pt7b);
    display.setTextColor(GxEPD_BLACK);
    display.setCursor(x_start, y_start);

    // Simple word wrap within max_width.
    const char *p = message;
    String line;
    int16_t x1, y1; uint16_t tw, th;
    int16_t cy = (int16_t)y_start;
    const int16_t lineStep = 36;

    while (*p) {
        // gather a word
        String word;
        while (*p == ' ') { word += ' '; p++; }
        while (*p && *p != ' ') { word += *p; p++; }

        String candidate = line + word;
        display.getTextBounds(candidate.c_str(), 0, 0, &x1, &y1, &tw, &th);
        if (tw > max_width && line.length() > 0) {
            display.setCursor(x_start, cy);
            display.print(line);
            cy += lineStep;
            line = word;
            // trim leading spaces on a new line
            line.trim();
        } else {
            line = candidate;
        }
    }
    if (line.length() > 0) {
        display.setCursor(x_start, cy);
        display.print(line);
    }
}

#endif // BOARD_SEEED_RETERMINAL_E1004

