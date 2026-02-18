//esp32-video player with audio by over-dev
// Features:
//   - MJPEG playback dari SD card  (24 fps, frame-deadline sync)
//   - Audio sync via DAC GPIO26 (WAV 8000Hz mono 8-bit)
//   - Low-pass filter software (EQ) untuk noise DAC ★ NEW
//   - Info screen overlay (nama file, FPS, drift A-V, sisa SD) ★ NEW
//   - ESP-NOW remote control (ESP32-C3)
//   - Short press BOOT → next
//   - Long press BOOT  → restart video
//   - Pause / Resume, Shuffle, Prev, Mute, Vol
//
// Konversi video dengan FFmpeg:
//   ffmpeg -y -i input.mp4 -pix_fmt yuvj420p -q:v 7 -vf "transpose=1,fps=15,scale=-1:320:flags=lanczos" output.mjpeg
//   ffmpeg -y -i input.mp4 -ar 8000 -ac 1 -acodec pcm_u8 output.wav
//
// Struktur SD: /mjpeg/output.mjpeg + /mjpeg/output.wav  (nama sama)
// Board: ESP32 Dev Module

#include <Arduino_GFX_Library.h>
#include "MjpegClass.h"
#include "SD.h"
#include <WiFi.h>
#include <esp_now.h>
#include "driver/dac.h"
#include "freertos/semphr.h"

// ─── Pin ──────────────────────────────────────────────────────────────────────
#define BL_PIN            21
#define SD_CS              5
#define SD_MISO           19
#define SD_MOSI           23
#define SD_SCK            18

// ─── Button ───────────────────────────────────────────────────────────────────
#define BOOT_PIN           0
#define DEBOUNCE_MS      400
#define LONG_PRESS_MS   1000

// ─── SPI Speed ────────────────────────────────────────────────────────────────
#define DISPLAY_SPI_SPEED  40000000L
#define SD_SPI_SPEED       80000000L

// ─── Folder ───────────────────────────────────────────────────────────────────
const char *MJPEG_FOLDER = "/mjpeg";

// ─── Audio WAV ────────────────────────────────────────────────────────────────
#define AUDIO_SAMPLE_RATE   8000    // Hz — harus sama dengan FFmpeg -ar
#define AUDIO_CHUNK_BYTES   256

// ─── Low-pass filter (EQ) ────────────────────────────────────────────────────
// ★ NEW: IIR single-pole low-pass filter untuk halus-kan output DAC
// Formula: y[n] = alpha * x[n] + (1 - alpha) * y[n-1]
// alpha = 0.0 → sangat smooth (bass only)
// alpha = 1.0 → tidak ada filtering (original)
// Nilai 0.4 bagus untuk noise DAC 8000Hz → potong ~2kHz ke atas
#define LPF_ALPHA_DEFAULT   0.4f

// Nilai alpha per level EQ:
// Level 0 = OFF (alpha=1.0, bypass), Level 1-5 = makin smooth
const float LPF_ALPHA_TABLE[6] = {
    1.0f,   // 0 = EQ OFF
    0.7f,   // 1 = ringan
    0.5f,   // 2 = sedang
    0.35f,  // 3 = kuat (default)
    0.2f,   // 4 = sangat kuat
    0.1f,   // 5 = bass only
};
volatile int eqLevel = 3;   // default level 3

// ─── Info Screen ──────────────────────────────────────────────────────────────
// ★ NEW: Overlay info screen saat CMD_INFO dikirim dari remote
// Tampil selama INFO_DISPLAY_MS ms lalu hilang otomatis
#define INFO_DISPLAY_MS   4000
volatile bool showInfoReq    = false;
volatile bool infoVisible    = false;
uint32_t      infoShownAt    = 0;

// ─── Video frame timing ───────────────────────────────────────────────────────
#define VIDEO_FPS           24
#define BENCHMARK_FRAMES    20
#define FRAME_INTERVAL_US   (1000000UL / VIDEO_FPS)
#define SYNC_LOG_EVERY      48
#define FPS_OK_PERCENT      90

// ─── ESP-NOW Commands ─────────────────────────────────────────────────────────
#define CMD_PREV     1
#define CMD_NEXT     2
#define CMD_RESTART  3
#define CMD_PAUSE    4
#define CMD_SHUFFLE  5
#define CMD_MUTE     6
#define CMD_VOL_UP   7
#define CMD_VOL_DOWN 8
#define CMD_INFO     9   // ★ NEW: tampilkan info screen
#define CMD_EQ_UP   10   // ★ NEW: naikkan EQ level (makin smooth)
#define CMD_EQ_DOWN 11   // ★ NEW: turunkan EQ level (makin raw)

typedef struct { uint8_t cmd; } RemotePacket;

// ─── File list ────────────────────────────────────────────────────────────────
#define MAX_FILES 20
String   mjpegFileList[MAX_FILES];
uint32_t mjpegFileSizes[MAX_FILES] = {0};
int      mjpegCount        = 0;
int      currentMjpegIndex = 0;

// ─── Playback flags ───────────────────────────────────────────────────────────
volatile bool btnShortReq   = false;
volatile bool remNextReq    = false;
volatile bool remPrevReq    = false;
volatile bool remRestartReq = false;
volatile bool remPauseReq   = false;
volatile bool shuffleOn     = false;
volatile bool isPaused      = false;

// ─── Audio state ──────────────────────────────────────────────────────────────
volatile bool audioRunning  = false;
volatile bool audioStopReq  = false;
volatile bool audioPauseReq = false;
volatile bool audioMuted    = false;
volatile int  audioVolume   = 8;    // 0=silent, 10=full

// Mutex SD card — video dan audio tidak baca SD bersamaan
SemaphoreHandle_t sdMutex = nullptr;

static unsigned long  *videoStartMsPtr = nullptr;
static uint32_t        audioPausedMs   = 0;
static char            activeWavPath[128] = {0};
TaskHandle_t audioTaskHandle = nullptr;

// ─── Video frame timing state ─────────────────────────────────────────────────
static int64_t  video_start_us   = 0;
static int64_t  video_paused_us  = 0;
static int64_t  pause_started_us = 0;

// ─── Benchmark state ──────────────────────────────────────────────────────────
static int64_t  bench_start_us   = 0;
static bool     benchDone        = false;
static uint32_t frameIntervalUs  = FRAME_INTERVAL_US;

// ─── MJPEG globals ────────────────────────────────────────────────────────────
MjpegClass mjpeg;
int            total_frames;
unsigned long  total_read_video;
unsigned long  total_decode_video;
unsigned long  total_show_video;
unsigned long  start_ms, curr_ms;
long           output_buf_size, estimateBufferSize;
uint8_t       *mjpeg_buf  = nullptr;
uint16_t      *output_buf = nullptr;

// ─── Info screen state ────────────────────────────────────────────────────────
// ★ NEW: simpan snapshot stats saat info diminta
static char   infoFileName[64]  = {0};
static float  infoFpsActual     = 0.0f;
static float  infoFpsTarget     = 0.0f;
static float  infoDriftMs       = 0.0f;
static float  infoSDFreeGB      = 0.0f;
static int    infoFrameCount    = 0;

// ─── Display ──────────────────────────────────────────────────────────────────
Arduino_DataBus *bus = new Arduino_HWSPI(2, 15, 14, 13, 12);
Arduino_GFX     *gfx = new Arduino_ILI9341(bus);

// ─── SD SPI ───────────────────────────────────────────────────────────────────
SPIClass sd_spi(VSPI);

// ─── ISR ──────────────────────────────────────────────────────────────────────
volatile uint32_t buttonPressedAt = 0;
uint32_t          lastPress       = 0;

void IRAM_ATTR onButtonPress()
{
    buttonPressedAt = millis();
    btnShortReq     = true;
}

// ─────────────────────────────────────────────────────────────────────────────
// Flag helpers
// ─────────────────────────────────────────────────────────────────────────────
void clearPhysicalFlags()
{
    btnShortReq = false;
    isPaused    = false;
}

void clearAllFlags()
{
    btnShortReq   = false;
    remNextReq    = false;
    remPrevReq    = false;
    remRestartReq = false;
    remPauseReq   = false;
    isPaused      = false;
}

// ─────────────────────────────────────────────────────────────────────────────
// ★ NEW: Low-pass filter apply
// Input  : raw sample 0-255 (PCM-U8)
// Output : filtered sample 0-255
// State  : lpfPrev (static, reset saat audio start)
// ─────────────────────────────────────────────────────────────────────────────
static float lpfPrev = 128.0f;

inline uint8_t applyLPF(uint8_t rawSample)
{
    float alpha = LPF_ALPHA_TABLE[eqLevel];
    if (alpha >= 1.0f) return rawSample;   // bypass jika EQ OFF

    lpfPrev = alpha * (float)rawSample + (1.0f - alpha) * lpfPrev;
    int out = (int)(lpfPrev + 0.5f);
    if (out < 0)   out = 0;
    if (out > 255) out = 255;
    return (uint8_t)out;
}

// ─────────────────────────────────────────────────────────────────────────────
// ★ NEW: Gambar overlay info screen di atas video yang sedang jalan
// ─────────────────────────────────────────────────────────────────────────────
void drawInfoOverlay()
{
    // Box lebih kecil, tetap di tengah
    const int OW = 180;
    const int OH = 88;
    const int OX = (gfx->width()  - OW) / 2;
    const int OY = (gfx->height() - OH) / 2;
    const int LINE_H = 11;   // jarak antar baris
    const int TX = OX + 5;   // indent teks

    // Background hitam + border cyan
    gfx->fillRoundRect(OX, OY, OW, OH, 4, RGB565_BLACK);
    gfx->drawRoundRect(OX, OY, OW, OH, 4, 0x07FF);

    // ── Judul (header kecil) ──────────────────────────────────────────────────
    gfx->setTextSize(1);
    gfx->setTextColor(0x07FF);
    gfx->setCursor(TX, OY + 4);
    gfx->print("INFO");
    gfx->drawFastHLine(OX + 2, OY + 14, OW - 4, 0x07FF);

    // ── Baris info (textSize 1 = 6x8 px per char) ────────────────────────────
    gfx->setTextColor(RGB565_WHITE);
    int y = OY + 17;

    // File
    gfx->setCursor(TX, y);
    gfx->printf("%-18s", infoFileName);   // max 18 char, rata kiri
    y += LINE_H;

    // FPS
    gfx->setCursor(TX, y);
    gfx->printf("FPS  %.1f/%.0f", infoFpsActual, infoFpsTarget);
    y += LINE_H;

    // Drift
    gfx->setCursor(TX, y);
    float absD = fabsf(infoDriftMs);
    gfx->printf("Drift%s%.0fms", (infoDriftMs >= 0) ? " A+" : " V+", absD);
    y += LINE_H;

    // Frame + SD Free (satu baris, dua kolom)
    gfx->setCursor(TX, y);
    gfx->printf("Frm %-5d SD %.1fG", infoFrameCount, infoSDFreeGB);
    y += LINE_H;

    // EQ
    gfx->setCursor(TX, y);
    if (eqLevel == 0)
        gfx->print("EQ   OFF");
    else
        gfx->printf("EQ   Lv%d (a=%.2f)", eqLevel, LPF_ALPHA_TABLE[eqLevel]);
}

// ─────────────────────────────────────────────────────────────────────────────
// ★ NEW: Snapshot stats untuk info screen
// Dipanggil tepat saat CMD_INFO diterima
// ─────────────────────────────────────────────────────────────────────────────
void snapshotInfoStats(const char *mjpegPath)
{
    // Nama file saja (strip folder)
    const char *slash = strrchr(mjpegPath, '/');
    strncpy(infoFileName, slash ? slash + 1 : mjpegPath, sizeof(infoFileName) - 1);
    // Potong jika terlalu panjang untuk layar
    if (strlen(infoFileName) > 20) infoFileName[20] = '\0';

    // FPS aktual
    int64_t elapsed_us = esp_timer_get_time() - video_start_us - video_paused_us;
    infoFpsActual = (elapsed_us > 0) ? (total_frames * 1e6f / elapsed_us) : 0.0f;
    infoFpsTarget = (frameIntervalUs > 0) ? (1e6f / frameIntervalUs) : VIDEO_FPS;

    // Drift A-V (positif = audio leading)
    float aud_ms = (float)(millis() - start_ms - audioPausedMs);
    float vid_ms = (float)(elapsed_us / 1000);
    infoDriftMs  = aud_ms - vid_ms;

    infoFrameCount = total_frames;

    // SD Free
    xSemaphoreTake(sdMutex, portMAX_DELAY);
    uint64_t freeBytes = SD.totalBytes() - SD.usedBytes();
    xSemaphoreGive(sdMutex);
    infoSDFreeGB = (float)freeBytes / (1024.0f * 1024.0f * 1024.0f);
}

// ─────────────────────────────────────────────────────────────────────────────
// Audio Task — Core 0
// Streaming WAV dari SD, sinkron ke waktu video
// ★ MODIFIED: tambah low-pass filter per sample
// ─────────────────────────────────────────────────────────────────────────────
void audioTask(void *param)
{
    if (!videoStartMsPtr) { audioRunning = false; vTaskDelete(NULL); return; }

    xSemaphoreTake(sdMutex, portMAX_DELAY);
    File wavFile = SD.open(activeWavPath, "r");
    xSemaphoreGive(sdMutex);

    if (!wavFile)
    {
        Serial.printf("[Audio] Cannot open: %s\n", activeWavPath);
        audioRunning = false; vTaskDelete(NULL); return;
    }

    xSemaphoreTake(sdMutex, portMAX_DELAY);
    uint8_t hdr[44];
    wavFile.read(hdr, 44);
    xSemaphoreGive(sdMutex);

    if (hdr[0]!='R' || hdr[1]!='I' || hdr[2]!='F' || hdr[3]!='F')
    {
        Serial.println("[Audio] Invalid WAV header");
        xSemaphoreTake(sdMutex, portMAX_DELAY); wavFile.close(); xSemaphoreGive(sdMutex);
        audioRunning = false; vTaskDelete(NULL); return;
    }

    // ★ Reset LPF state di awal setiap track
    lpfPrev = 128.0f;

    audioRunning = true;
    Serial.println("[Audio] Streaming WAV...");

    uint8_t  chunk[AUDIO_CHUNK_BYTES];
    uint32_t sampleIdx  = 0;
    uint32_t pauseStart = 0;

    while (!audioStopReq)
    {
        // ── Pause ─────────────────────────────────────────────────────────────
        if (audioPauseReq)
        {
            pauseStart = millis();
            dac_output_voltage(DAC_CHANNEL_2, 128);
            while (audioPauseReq && !audioStopReq)
                vTaskDelay(pdMS_TO_TICKS(10));
            audioPausedMs += millis() - pauseStart;
        }
        if (audioStopReq) break;

        // ── Baca chunk dari SD ────────────────────────────────────────────────
        int bytesRead = 0;
        if (xSemaphoreTake(sdMutex, pdMS_TO_TICKS(5)) == pdTRUE)
        {
            if (wavFile.available())
                bytesRead = wavFile.read(chunk, AUDIO_CHUNK_BYTES);
            xSemaphoreGive(sdMutex);
        }
        if (bytesRead <= 0)
        {
            if (!wavFile.available()) break;
            vTaskDelay(pdMS_TO_TICKS(2)); continue;
        }

        // ── Output sample per sample dengan time-sync ─────────────────────────
        for (int i = 0; i < bytesRead && !audioStopReq; i++)
        {
            if (audioPauseReq) break;

            uint32_t elapsedMs = millis() - (*videoStartMsPtr) - audioPausedMs;
            uint32_t targetIdx = (uint32_t)((uint64_t)elapsedMs * AUDIO_SAMPLE_RATE / 1000UL);

            if (sampleIdx < targetIdx)
            {
                uint32_t gap = targetIdx - sampleIdx;
                if (gap > AUDIO_SAMPLE_RATE / 2) {
                    Serial.printf("[Audio] Hard resync +%ums\n", gap * 1000 / AUDIO_SAMPLE_RATE);
                    // ★ Reset LPF saat hard resync agar tidak artefak
                    lpfPrev = 128.0f;
                    sampleIdx = targetIdx;
                } else {
                    sampleIdx++;
                }
                continue;
            }
            else if (sampleIdx > targetIdx + 16)
            {
                delayMicroseconds(125);
                i--;
                continue;
            }

            // ── Output ke DAC ─────────────────────────────────────────────────
            if (audioMuted || audioVolume == 0)
            {
                dac_output_voltage(DAC_CHANNEL_2, 128);
            }
            else
            {
                int s      = (int)chunk[i];

                // ★ NEW: Terapkan low-pass filter SEBELUM volume scaling
                // Ini mengurangi aliasing noise dari DAC 8-bit
                uint8_t filtered = applyLPF((uint8_t)s);

                int scaled = ((int)(filtered - 128) * audioVolume / 10) + 128;
                if (scaled < 0)   scaled = 0;
                if (scaled > 255) scaled = 255;
                dac_output_voltage(DAC_CHANNEL_2, (uint8_t)scaled);
            }
            sampleIdx++;
            delayMicroseconds(125);   // 1/8000Hz = 125µs
        }
    }

    dac_output_voltage(DAC_CHANNEL_2, 128);
    xSemaphoreTake(sdMutex, portMAX_DELAY); wavFile.close(); xSemaphoreGive(sdMutex);
    audioRunning = false;
    Serial.printf("[Audio] Done, %u samples\n", sampleIdx);
    vTaskDelete(NULL);
}

// ─────────────────────────────────────────────────────────────────────────────
// Start / stop audio
// ─────────────────────────────────────────────────────────────────────────────
void startAudio(const char *mjpegPath, unsigned long *startMsPtr)
{
    strncpy(activeWavPath, mjpegPath, sizeof(activeWavPath) - 1);
    char *ext = strstr(activeWavPath, ".mjpeg");
    if (!ext) { Serial.println("[Audio] Cannot build WAV path"); return; }
    strcpy(ext, ".wav");

    xSemaphoreTake(sdMutex, portMAX_DELAY);
    bool exists = SD.exists(activeWavPath);
    xSemaphoreGive(sdMutex);
    if (!exists) { Serial.printf("[Audio] No WAV: %s\n", activeWavPath); return; }

    videoStartMsPtr = startMsPtr;
    audioPausedMs   = 0;
    audioStopReq    = false;
    audioPauseReq   = false;
    audioRunning    = false;
    xTaskCreatePinnedToCore(audioTask, "audioTask", 4096, NULL, 2, &audioTaskHandle, 0);
}

void stopAudio()
{
    if (audioTaskHandle)
    {
        audioStopReq = true;
        uint32_t t = millis();
        while (audioRunning && millis() - t < 600) delay(10);
        audioTaskHandle = nullptr;
    }
    audioStopReq  = false;
    audioPauseReq = false;
    dac_output_voltage(DAC_CHANNEL_2, 128);
}

// ─────────────────────────────────────────────────────────────────────────────
// ESP-NOW callback
// ★ MODIFIED: tambah CMD_INFO, CMD_EQ_UP, CMD_EQ_DOWN
// ─────────────────────────────────────────────────────────────────────────────
void onDataReceive(const esp_now_recv_info_t *info, const uint8_t *data, int len)
{
    if (len < (int)sizeof(RemotePacket)) return;
    RemotePacket pkt;
    memcpy(&pkt, data, sizeof(pkt));
    Serial.printf("[ESP-NOW] CMD: %d\n", pkt.cmd);
    switch (pkt.cmd)
    {
        case CMD_PREV:    remPrevReq    = true; break;
        case CMD_NEXT:    remNextReq    = true; break;
        case CMD_RESTART: remRestartReq = true; break;
        case CMD_PAUSE:   remPauseReq   = true; break;
        case CMD_SHUFFLE:
            shuffleOn = !shuffleOn;
            Serial.printf("[ESP-NOW] Shuffle: %s\n", shuffleOn ? "ON" : "OFF");
            break;
        case CMD_MUTE:
            audioMuted = !audioMuted;
            if (audioMuted) dac_output_voltage(DAC_CHANNEL_2, 128);
            Serial.printf("[ESP-NOW] Audio: %s\n", audioMuted ? "MUTED" : "ON");
            break;
        case CMD_VOL_UP:
            if (audioVolume < 10) audioVolume++;
            audioMuted = false;
            Serial.printf("[ESP-NOW] Volume: %d/10\n", (int)audioVolume);
            break;
        case CMD_VOL_DOWN:
            if (audioVolume > 0) audioVolume--;
            Serial.printf("[ESP-NOW] Volume: %d/10\n", (int)audioVolume);
            break;

        // ★ NEW ───────────────────────────────────────────────────────────────
        case CMD_INFO:
            showInfoReq = true;
            Serial.println("[ESP-NOW] Info screen requested");
            break;
        case CMD_EQ_UP:
            if (eqLevel < 5) eqLevel++;
            Serial.printf("[ESP-NOW] EQ Level: %d (alpha=%.2f)\n",
                          eqLevel, LPF_ALPHA_TABLE[eqLevel]);
            break;
        case CMD_EQ_DOWN:
            if (eqLevel > 0) eqLevel--;
            Serial.printf("[ESP-NOW] EQ Level: %d%s\n",
                          eqLevel, eqLevel == 0 ? " (OFF)" : "");
            break;
        // ─────────────────────────────────────────────────────────────────────

        default:
            Serial.printf("[ESP-NOW] Unknown: %d\n", pkt.cmd);
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Show error
// ─────────────────────────────────────────────────────────────────────────────
void showError(const char *msg)
{
    gfx->fillScreen(RGB565_BLACK);
    gfx->setTextColor(RGB565_RED);
    gfx->setTextSize(2);
    int cx = max(0, (gfx->width() - (int)strlen(msg) * 12) / 2);
    int cy = (gfx->height() - 16) / 2;
    gfx->setCursor(cx, cy);
    gfx->println(msg);
    Serial.printf("ERROR: %s\n", msg);
}

// ─────────────────────────────────────────────────────────────────────────────
// SETUP
// ─────────────────────────────────────────────────────────────────────────────
void setup()
{
    Serial.begin(115200);

    pinMode(BL_PIN, OUTPUT);
    digitalWrite(BL_PIN, HIGH);

    dac_output_enable(DAC_CHANNEL_2);
    dac_output_voltage(DAC_CHANNEL_2, 128);

    Serial.println("Display init");
    if (!gfx->begin(DISPLAY_SPI_SPEED)) { Serial.println("Display FAILED"); while (1) {} }
    gfx->setRotation(0);
    gfx->fillScreen(RGB565_BLACK);
    Serial.printf("Screen: %dx%d\n", gfx->width(), gfx->height());

    sdMutex = xSemaphoreCreateMutex();

    Serial.println("SD init");
    sd_spi.begin(SD_SCK, SD_MISO, SD_MOSI, SD_CS);
    if (!SD.begin(SD_CS, sd_spi, SD_SPI_SPEED, "/sd"))
    {
        showError("SD Card Mount Failed!"); while (1) {}
    }

    output_buf_size = gfx->width() * 4 * 2;
    output_buf = (uint16_t *)heap_caps_aligned_alloc(16, output_buf_size * sizeof(uint16_t), MALLOC_CAP_DMA);
    if (!output_buf) { showError("output_buf alloc failed!"); while (1) {} }

    estimateBufferSize = gfx->width() * gfx->height() * 2 / 5;
    mjpeg_buf = (uint8_t *)heap_caps_malloc(estimateBufferSize, MALLOC_CAP_8BIT);
    if (!mjpeg_buf) { showError("mjpeg_buf alloc failed!"); while (1) {} }

    loadMjpegFilesList();

    pinMode(BOOT_PIN, INPUT);
    attachInterrupt(digitalPinToInterrupt(BOOT_PIN), onButtonPress, FALLING);

    WiFi.mode(WIFI_STA);
    WiFi.disconnect();
    Serial.print("MAC: ");
    Serial.println(WiFi.macAddress());

    if (esp_now_init() != ESP_OK)
        Serial.println("[ESP-NOW] Init failed, manual only");
    else
    {
        esp_now_register_recv_cb(onDataReceive);
        Serial.println("[ESP-NOW] Ready");
    }

    Serial.printf("[EQ] Default level: %d (alpha=%.2f)\n",
                  eqLevel, LPF_ALPHA_TABLE[eqLevel]);
}

// ─────────────────────────────────────────────────────────────────────────────
// LOOP
// ─────────────────────────────────────────────────────────────────────────────
void loop()
{
    if (mjpegCount == 0) { showError("No .mjpeg files!"); delay(3000); return; }
    playSelectedMjpeg(currentMjpegIndex);
}

// ─────────────────────────────────────────────────────────────────────────────
// Advance index
// ─────────────────────────────────────────────────────────────────────────────
void advanceIndex()
{
    if (shuffleOn && mjpegCount > 1)
    {
        int n;
        do { n = random(0, mjpegCount); } while (n == currentMjpegIndex);
        currentMjpegIndex = n;
    }
    else
        currentMjpegIndex = (currentMjpegIndex + 1) % mjpegCount;
}

// ─────────────────────────────────────────────────────────────────────────────
// Play wrapper
// ─────────────────────────────────────────────────────────────────────────────
void playSelectedMjpeg(int idx)
{
    String fullPath = String(MJPEG_FOLDER) + "/" + mjpegFileList[idx];
    char fname[128];
    fullPath.toCharArray(fname, sizeof(fname));
    Serial.printf("Playing: %s\n", fname);
    mjpegPlayFromSDCard(fname, mjpegFileSizes[idx]);
}

// ─────────────────────────────────────────────────────────────────────────────
// JPEG draw callback
// ─────────────────────────────────────────────────────────────────────────────
int jpegDrawCallback(JPEGDRAW *pDraw)
{
    unsigned long s = millis();
    gfx->draw16bitBeRGBBitmap(pDraw->x, pDraw->y, pDraw->pPixels, pDraw->iWidth, pDraw->iHeight);
    total_show_video += millis() - s;
    return 1;
}

bool readMjpegBufSafe(File &f)
{
    xSemaphoreTake(sdMutex, portMAX_DELAY);
    bool ok = mjpeg.readMjpegBuf();
    xSemaphoreGive(sdMutex);
    return ok;
}

// ─────────────────────────────────────────────────────────────────────────────
// Main playback function
// ★ MODIFIED: tambah handling showInfoReq dan info overlay
// ─────────────────────────────────────────────────────────────────────────────
void mjpegPlayFromSDCard(char *mjpegFilename, uint32_t fileSize)
{
RESTART_VIDEO:
    stopAudio();

    xSemaphoreTake(sdMutex, portMAX_DELAY);
    File mjpegFile = SD.open(mjpegFilename, "r");
    xSemaphoreGive(sdMutex);

    if (!mjpegFile || mjpegFile.isDirectory())
    {
        char buf[80]; snprintf(buf, 80, "Cannot open: %.50s", mjpegFilename);
        showError(buf); delay(2000); advanceIndex(); return;
    }

    uint8_t hdr[2] = {0, 0};
    xSemaphoreTake(sdMutex, portMAX_DELAY);
    mjpegFile.read(hdr, 2);
    xSemaphoreGive(sdMutex);

    if (hdr[0] != 0xFF || hdr[1] != 0xD8)
    {
        xSemaphoreTake(sdMutex, portMAX_DELAY); mjpegFile.close(); xSemaphoreGive(sdMutex);
        char buf[80]; snprintf(buf, 80, "Corrupt: %.50s", mjpegFilename);
        showError(buf); delay(2000); advanceIndex(); return;
    }

    xSemaphoreTake(sdMutex, portMAX_DELAY); mjpegFile.seek(0); xSemaphoreGive(sdMutex);

    gfx->fillScreen(RGB565_BLACK);
    start_ms = curr_ms = millis();
    total_frames = total_read_video = total_decode_video = total_show_video = 0;

    clearPhysicalFlags();

    // ★ Reset info screen state
    showInfoReq  = false;
    infoVisible  = false;
    infoShownAt  = 0;

    mjpeg.setup(&mjpegFile, mjpeg_buf, jpegDrawCallback, true,
                0, 0, gfx->width(), gfx->height());

    startAudio(mjpegFilename, &start_ms);

    bool doRestart = false;
    bool doPrev    = false;

    // ── Frame timing init ─────────────────────────────────────────────────────
    bool   firstFrame  = true;
    video_paused_us    = 0;
    pause_started_us   = 0;
    bench_start_us     = 0;
    benchDone          = false;
    frameIntervalUs    = FRAME_INTERVAL_US;

    while (mjpegFile.available() && readMjpegBufSafe(mjpegFile))
    {
        total_read_video += millis() - curr_ms;
        curr_ms = millis();

        // ── Pause ─────────────────────────────────────────────────────────────
        if (remPauseReq)
        {
            isPaused      = !isPaused;
            audioPauseReq = isPaused;
            remPauseReq   = false;
            if (isPaused) {
                pause_started_us = esp_timer_get_time();
                Serial.println("PAUSED");
            } else {
                video_paused_us += esp_timer_get_time() - pause_started_us;
                Serial.println("RESUMED");
            }
        }
        while (isPaused)
        {
            delay(20);
            if (remPauseReq)   { isPaused=false; audioPauseReq=false; remPauseReq=false;   video_paused_us += esp_timer_get_time()-pause_started_us; }
            if (btnShortReq)   { isPaused=false; audioPauseReq=false;                       video_paused_us += esp_timer_get_time()-pause_started_us; }
            if (remNextReq)    { isPaused=false; audioPauseReq=false;                       video_paused_us += esp_timer_get_time()-pause_started_us; }
            if (remPrevReq)    { isPaused=false; audioPauseReq=false;                       video_paused_us += esp_timer_get_time()-pause_started_us; }
            if (remRestartReq) { isPaused=false; audioPauseReq=false;                       video_paused_us += esp_timer_get_time()-pause_started_us; }
        }

        // ── Render frame ──────────────────────────────────────────────────────
        mjpeg.drawJpg();
        total_decode_video += millis() - curr_ms;
        curr_ms = millis();
        total_frames++;

        // ── Set anchor waktu setelah frame pertama selesai render ─────────────
        if (firstFrame)
        {
            firstFrame     = false;
            video_start_us = esp_timer_get_time();
            bench_start_us = video_start_us;
            start_ms       = millis();
        }

        // ── Benchmark ─────────────────────────────────────────────────────────
        if (!benchDone && total_frames == BENCHMARK_FRAMES)
        {
            benchDone = true;
            int64_t bench_elapsed_us = esp_timer_get_time() - bench_start_us;
            float   esp_fps = (bench_elapsed_us > 0)
                              ? (BENCHMARK_FRAMES * 1e6f / bench_elapsed_us)
                              : 0.0f;
            float   pct     = (esp_fps / VIDEO_FPS) * 100.0f;

            Serial.println("╔══════════════════════════════════════╗");
            Serial.println("║        BENCHMARK HASIL               ║");
            Serial.printf( "║  Target FPS  : %2d fps               ║\n", VIDEO_FPS);
            Serial.printf( "║  ESP32 mampu : %.1f fps             ║\n", esp_fps);
            Serial.printf( "║  Kemampuan   : %.0f%% dari target    ║\n", pct);

            if (pct >= FPS_OK_PERCENT)
            {
                Serial.println("║  Status: OK ✓ — tidak perlu konversi║");
                Serial.println("╚══════════════════════════════════════╝");
                frameIntervalUs = (uint32_t)(1e6f / esp_fps);
            }
            else
            {
                int recommended_fps = max(1, (int)(esp_fps * 0.80f));
                const int nice[] = {20, 15, 12, 10, 8, 6, 5};
                for (int i = 0; i < 7; i++) {
                    if (recommended_fps >= nice[i]) { recommended_fps = nice[i]; break; }
                    if (i == 6) recommended_fps = nice[6];
                }

                Serial.println("║  Status: ⚠ TERLALU CEPAT            ║");
                Serial.println("╠══════════════════════════════════════╣");
                Serial.printf( "║  Rekomendasi: fps=%d                ║\n", recommended_fps);
                Serial.println("╚══════════════════════════════════════╝");
                Serial.printf("[Sync] Fallback ke %d fps untuk sesi ini\n", recommended_fps);
                frameIntervalUs = (uint32_t)(1e6f / recommended_fps);
            }
        }

        // ── Frame deadline ────────────────────────────────────────────────────
        int64_t now_us    = esp_timer_get_time();
        int64_t target_us = video_start_us
                            + (int64_t)total_frames * frameIntervalUs
                            - video_paused_us;
        int64_t delta_us  = target_us - now_us;

        if (delta_us > 500)
        {
            while (delta_us > 1000)
            {
                delayMicroseconds(1000);
                delta_us = target_us - esp_timer_get_time();
                if (remNextReq || remPrevReq || remRestartReq ||
                    btnShortReq || remPauseReq || showInfoReq) break;
            }
            if (delta_us > 0) delayMicroseconds((uint32_t)delta_us);
        }

        // ── Log sinkronisasi berkala ──────────────────────────────────────────
#if SYNC_LOG_EVERY > 0
        if (total_frames % SYNC_LOG_EVERY == 0)
        {
            int64_t elapsed_us = esp_timer_get_time() - video_start_us - video_paused_us;
            float   actual_fps = (elapsed_us > 0) ?
                                 (total_frames * 1e6f / elapsed_us) : 0.0f;
            float   target_fps_f = (frameIntervalUs > 0) ? (1e6f / frameIntervalUs) : 0.0f;
            float   aud_ms     = (float)(millis() - start_ms - audioPausedMs);
            float   vid_ms     = (float)(elapsed_us / 1000);
            Serial.printf("[Sync] Frame:%4d | FPS:%.1f/%.0f | Drift A-V:%.1fms\n",
                          total_frames, actual_fps, target_fps_f, aud_ms - vid_ms);
        }
#endif

        // ★ Handle info screen request — tekan lagi saat visible = dismiss
        if (showInfoReq)
        {
            showInfoReq = false;
            if (infoVisible)
            {
                // Sudah tampil → dismiss langsung
                infoVisible = false;
                Serial.println("[Info] Overlay dismissed");
            }
            else
            {
                // Belum tampil → tampilkan
                infoVisible = true;
                infoShownAt = millis();
                snapshotInfoStats(mjpegFilename);
                Serial.println("[Info] Overlay shown");
            }
        }

        // ★ NEW: Gambar info overlay jika aktif ────────────────────────────────
        if (infoVisible)
        {
            drawInfoOverlay();
            if (millis() - infoShownAt >= INFO_DISPLAY_MS)
            {
                infoVisible = false;
                // Tidak perlu gfx->fillScreen karena frame berikutnya
                // akan overwrite area yang sama secara otomatis
                Serial.println("[Info] Overlay hidden");
            }
        }

        // ── Remote / button ───────────────────────────────────────────────────
        if (remNextReq)
        {
            remNextReq = false; btnShortReq = false;
            Serial.println("Remote: NEXT"); break;
        }
        if (remPrevReq)
        {
            remPrevReq = false; btnShortReq = false;
            currentMjpegIndex = (currentMjpegIndex - 1 + mjpegCount) % mjpegCount;
            doPrev = true;
            Serial.println("Remote: PREV"); break;
        }
        if (remRestartReq)
        {
            remRestartReq = false; btnShortReq = false;
            doRestart = true;
            Serial.println("Remote: RESTART"); break;
        }
        if (btnShortReq)
        {
            uint32_t now  = millis();
            uint32_t hold = now - buttonPressedAt;
            if (now - lastPress < DEBOUNCE_MS)
            {
                btnShortReq = false;
            }
            else if (hold >= LONG_PRESS_MS)
            {
                lastPress = now; btnShortReq = false;
                doRestart = true;
                Serial.println("Button LONG: restart"); break;
            }
            else
            {
                lastPress = now; btnShortReq = false;
                Serial.println("Button SHORT: next"); break;
            }
        }
    }

    // ── Stats ─────────────────────────────────────────────────────────────────
    int   tu  = millis() - start_ms;
    float fps = (tu > 0) ? (1000.0f * total_frames / tu) : 0.0f;
    float target_fps = (frameIntervalUs > 0) ? (1e6f / frameIntervalUs) : VIDEO_FPS;
    total_decode_video -= total_show_video;
    Serial.printf("=== Selesai: %d frame | %dms | %.1f fps (target %.0f fps) ===\n",
                  total_frames, tu, fps, target_fps);

    xSemaphoreTake(sdMutex, portMAX_DELAY); mjpegFile.close(); xSemaphoreGive(sdMutex);
    stopAudio();
    clearAllFlags();

    if (doRestart)    goto RESTART_VIDEO;
    else if (!doPrev) advanceIndex();
}

// ─────────────────────────────────────────────────────────────────────────────
// Load + sort file list A→Z
// ─────────────────────────────────────────────────────────────────────────────
void loadMjpegFilesList()
{
    xSemaphoreTake(sdMutex, portMAX_DELAY);
    File dir = SD.open(MJPEG_FOLDER);
    xSemaphoreGive(sdMutex);

    if (!dir) { Serial.printf("Cannot open %s\n", MJPEG_FOLDER); while (1) {} }

    mjpegCount = 0;
    while (true)
    {
        xSemaphoreTake(sdMutex, portMAX_DELAY);
        File f = dir.openNextFile();
        xSemaphoreGive(sdMutex);
        if (!f) break;
        if (!f.isDirectory())
        {
            String name = f.name();
            if (name.endsWith(".mjpeg") && mjpegCount < MAX_FILES)
            {
                mjpegFileList[mjpegCount]  = name;
                mjpegFileSizes[mjpegCount] = f.size();
                mjpegCount++;
            }
        }
        f.close();
    }
    dir.close();

    for (int i = 0; i < mjpegCount - 1; i++)
        for (int j = 0; j < mjpegCount - i - 1; j++)
            if (mjpegFileList[j] > mjpegFileList[j + 1])
            {
                String   tn = mjpegFileList[j];  mjpegFileList[j]  = mjpegFileList[j+1];  mjpegFileList[j+1]  = tn;
                uint32_t ts = mjpegFileSizes[j]; mjpegFileSizes[j] = mjpegFileSizes[j+1]; mjpegFileSizes[j+1] = ts;
            }

    Serial.printf("%d files found:\n", mjpegCount);
    for (int i = 0; i < mjpegCount; i++)
        Serial.printf("  [%d] %s\n", i, mjpegFileList[i].c_str());
}

// ─────────────────────────────────────────────────────────────────────────────
// Helper
// ─────────────────────────────────────────────────────────────────────────────
String formatBytes(size_t b)
{
    if (b < 1024)      return String(b) + " B";
    if (b < 1048576)   return String(b/1024.0, 2) + " KB";
    return                    String(b/1048576.0, 2) + " MB";
}
