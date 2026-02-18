// ─────────────────────────────────────────────────────────────────────────────
// ESP32-C3 Remote Controller for MJPEG Player
// ★ Update: tambah CMD_INFO (long press BTN_OK) dan CMD_EQ_UP/DOWN (BTN_ACT)
// ─────────────────────────────────────────────────────────────────────────────
// Libraries needed:
//   - "Adafruit SSD1306"        (Library Manager)
//   - "Adafruit GFX Library"    (Library Manager)
//   - esp_now.h / WiFi.h        (built-in ESP32 Arduino Core)
//
// Board: "ESP32C3 Dev Module"
// ─────────────────────────────────────────────────────────────────────────────

#include <WiFi.h>
#include <esp_now.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// ─── Display ──────────────────────────────────────────────────────────────────
#define SCREEN_WIDTH  128
#define SCREEN_HEIGHT  64
#define OLED_RESET     -1
#define OLED_ADDR    0x3C

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// ─── I2C Pins ─────────────────────────────────────────────────────────────────
#define SDA_PIN 9
#define SCL_PIN 10

// ─── LED ──────────────────────────────────────────────────────────────────────
#define LED_PIN        8
#define LED_ON         LOW
#define LED_OFF        HIGH

// ─── Battery ──────────────────────────────────────────────────────────────────
#define BATTERY_PIN       0
#define BATTERY_SAMPLES   10
#define VREF           3300.0    // ★ FIX: ESP32-C3 ADC reference = 3.3V
#define ADC_MAX        4095.0
#define VOLTAGE_DIVIDER   2.0
#define BATTERY_MAX    4200
#define BATTERY_MIN    3000
#define BATTERY_CRITICAL 3300

// ─── Button Pins ──────────────────────────────────────────────────────────────
#define BTN_RIGHT  3
#define BTN_LEFT   2
#define BTN_UP     4
#define BTN_DOWN   5
#define BTN_OK     6
#define BTN_BACK   7
#define BTN_ACT    LOW

// ─── Button Timing ────────────────────────────────────────────────────────────
#define DEBOUNCE_MS    150
#define VOL_LONG_MS    300    // threshold hold untuk VOL / EQ repeat
#define INFO_LONG_MS   600   // ★ NEW: threshold long press BTN_OK → INFO

// ─── ESP-NOW Commands ─────────────────────────────────────────────────────────
#define CMD_PREV      1
#define CMD_NEXT      2
#define CMD_RESTART   3
#define CMD_PAUSE     4
#define CMD_SHUFFLE   5
#define CMD_MUTE      6
#define CMD_VOL_UP    7
#define CMD_VOL_DOWN  8
#define CMD_INFO      9   // ★ NEW
#define CMD_EQ_UP    10   // ★ NEW
#define CMD_EQ_DOWN  11   // ★ NEW

typedef struct {
    uint8_t cmd;
} RemotePacket;

// ─── MAC Address Player ───────────────────────────────────────────────────────
uint8_t playerMAC[] = { 0xEC, 0xE3, 0x34, 0x66, 0xA5, 0xDC };

// ─── State ────────────────────────────────────────────────────────────────────
bool     shuffleState   = false;
bool     muteState      = false;
int      volumeLevel    = 8;
bool     pauseState     = false;
bool     sendSuccess    = false;
bool     sendFailed     = false;
uint32_t lastFeedbackMs = 0;
uint32_t lastBatteryMs  = 0;
uint32_t lastDrawMs     = 0;
int      batteryPct     = 100;
bool     battCritical   = false;
String   lastAction     = "Ready";
int      eqLevelLocal   = 3;   // ★ NEW: mirror EQ level dari player

// Button state tracking
uint32_t lastPressTime[8] = {0};
uint32_t btnHoldStart[8]  = {0};
bool     btnHolding[8]    = {false};

// ─────────────────────────────────────────────────────────────────────────────
// ESP-NOW send callback
// ─────────────────────────────────────────────────────────────────────────────
void onDataSent(const uint8_t *mac_addr, esp_now_send_status_t status)
{
    if (status == ESP_NOW_SEND_SUCCESS)
    {
        sendSuccess = true;
        sendFailed  = false;
    }
    else
    {
        sendFailed  = true;
        sendSuccess = false;
        Serial.println("[ESP-NOW] Send FAILED");
    }
    lastFeedbackMs = millis();
    drawUI();
}

// ─────────────────────────────────────────────────────────────────────────────
// Send command to player
// ─────────────────────────────────────────────────────────────────────────────
void sendCommand(uint8_t cmd)
{
    RemotePacket packet;
    packet.cmd = cmd;
    esp_now_send(playerMAC, (uint8_t *)&packet, sizeof(packet));

    digitalWrite(LED_PIN, LED_ON);
    delay(80);
    digitalWrite(LED_PIN, LED_OFF);

    Serial.printf("[Remote] Sending CMD: %d\n", cmd);
}

// ─────────────────────────────────────────────────────────────────────────────
// Read battery percentage
// ─────────────────────────────────────────────────────────────────────────────
int readBattery()
{
    long sum = 0;
    for (int i = 0; i < BATTERY_SAMPLES; i++)
    {
        sum += analogRead(BATTERY_PIN);
        delay(2);
    }
    float adcAvg     = sum / (float)BATTERY_SAMPLES;
    float voltage_mv = (adcAvg / ADC_MAX) * VREF * VOLTAGE_DIVIDER;
    int   pct        = (int)((voltage_mv - BATTERY_MIN) * 100.0f / (BATTERY_MAX - BATTERY_MIN));
    pct = constrain(pct, 0, 100);
    battCritical = (voltage_mv < BATTERY_CRITICAL);
    return pct;
}

// ─────────────────────────────────────────────────────────────────────────────
// Draw battery icon
// ─────────────────────────────────────────────────────────────────────────────
void drawBattery(int x, int y, int pct)
{
    display.drawRect(x, y, 20, 10, SSD1306_WHITE);
    display.fillRect(x + 20, y + 3, 2, 4, SSD1306_WHITE);
    int fillW = (int)(16.0f * pct / 100.0f);
    if (fillW > 0)
        display.fillRect(x + 2, y + 2, fillW, 6, SSD1306_WHITE);
}

// ─────────────────────────────────────────────────────────────────────────────
// Draw the full UI
// ★ MODIFIED: tambah EQ level indicator
// ─────────────────────────────────────────────────────────────────────────────
void drawUI()
{
    display.clearDisplay();

    // ── Header bar ────────────────────────────────────────────────────────────
    display.fillRect(0, 0, SCREEN_WIDTH, 13, SSD1306_WHITE);
    display.setTextColor(SSD1306_BLACK);
    display.setTextSize(1);
    display.setCursor(3, 3);
    display.print("MJPEG Remote");
    drawBattery(SCREEN_WIDTH - 25, 2, batteryPct);

    // ── Status area ───────────────────────────────────────────────────────────
    display.setTextColor(SSD1306_WHITE);
    display.setTextSize(1);
    display.setCursor(2, 17);
    display.print("Action:");

    display.fillRoundRect(0, 25, SCREEN_WIDTH, 13, 2, SSD1306_WHITE);
    display.setTextColor(SSD1306_BLACK);
    display.setCursor(4, 28);
    display.print(lastAction);
    display.setTextColor(SSD1306_WHITE);

    // Send status
    display.setCursor(2, 41);
    if (millis() - lastFeedbackMs < 1500)
    {
        if (sendSuccess) display.print(">> Sent OK");
        else if (sendFailed) display.print(">> FAILED! Check MAC");
    }
    else
    {
        display.print("Waiting...");
    }

    // ── State indicators row ──────────────────────────────────────────────────
    display.setCursor(2, 53);

    // Pause/Play
    if (pauseState)
    {
        display.fillRoundRect(0, 52, 30, 11, 2, SSD1306_WHITE);
        display.setTextColor(SSD1306_BLACK);
        display.setCursor(4, 54);
        display.print("PAUSE");
        display.setTextColor(SSD1306_WHITE);
    }
    else
    {
        display.drawRoundRect(0, 52, 30, 11, 2, SSD1306_WHITE);
        display.setCursor(4, 54);
        display.print("PLAY");
    }

    // Shuffle
    if (shuffleState)
    {
        display.fillRoundRect(34, 52, 40, 11, 2, SSD1306_WHITE);
        display.setTextColor(SSD1306_BLACK);
        display.setCursor(38, 54);
        display.print("SHUFFLE");
        display.setTextColor(SSD1306_WHITE);
    }
    else
    {
        display.drawRoundRect(34, 52, 40, 11, 2, SSD1306_WHITE);
        display.setCursor(38, 54);
        display.print("ORDER");
    }

    // ★ NEW: EQ indicator (ganti volume bar kalau mute)
    if (muteState)
    {
        display.setCursor(78, 54);
        display.print("MUTE");
    }
    else
    {
        // Volume bar (5 kotak)
        for (int v = 0; v < 5; v++)
        {
            int bx = 78 + v * 5;
            int threshold = (volumeLevel * 5) / 10;   // skala 0-5
            if (v < threshold)
                display.fillRect(bx, 54, 4, 9, SSD1306_WHITE);
            else
                display.drawRect(bx, 54, 4, 9, SSD1306_WHITE);
        }

        // EQ mini indicator
        display.setCursor(107, 54);
        if (eqLevelLocal == 0)
            display.print("EQ:OFF");
        else
        {
            display.print("EQ:");
            display.print(eqLevelLocal);
        }
    }

    display.display();
}

// ─────────────────────────────────────────────────────────────────────────────
// Check button action
// Return: BTN_NONE, BTN_SHORT, atau BTN_HOLD
// ─────────────────────────────────────────────────────────────────────────────
#define BTN_NONE  0
#define BTN_SHORT 1
#define BTN_HOLD  2

// ★ NEW: BTN_LONG untuk long-press (INFO screen)
#define BTN_LONG  3

int buttonAction(int pin, int pinIndex, uint32_t longThresholdMs = VOL_LONG_MS)
{
    if (digitalRead(pin) == BTN_ACT)
    {
        uint32_t now = millis();
        if (!btnHolding[pinIndex])
        {
            btnHolding[pinIndex]    = true;
            btnHoldStart[pinIndex]  = now;
            lastPressTime[pinIndex] = now;
            return BTN_NONE;
        }

        uint32_t held = now - btnHoldStart[pinIndex];

        if (held >= longThresholdMs)
        {
            if (now - lastPressTime[pinIndex] >= 200)
            {
                lastPressTime[pinIndex] = now;
                return BTN_HOLD;
            }
        }
    }
    else
    {
        if (btnHolding[pinIndex])
        {
            uint32_t held = millis() - btnHoldStart[pinIndex];
            btnHolding[pinIndex] = false;

            // ★ Bedakan LONG vs SHORT berdasarkan threshold yang lebih besar
            if (held >= INFO_LONG_MS && longThresholdMs == INFO_LONG_MS)
                return BTN_LONG;
            else if (held < longThresholdMs)
                return BTN_SHORT;
            // Kalau held >= longThresholdMs tapi < INFO → sudah dipakai sebagai HOLD
        }
    }
    return BTN_NONE;
}

// ─────────────────────────────────────────────────────────────────────────────
// SETUP
// ─────────────────────────────────────────────────────────────────────────────
void setup()
{
    Serial.begin(115200);

    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, LED_OFF);

    pinMode(BATTERY_PIN, INPUT);   // ★ FIX: tambah pinMode battery

    pinMode(BTN_RIGHT, INPUT_PULLUP);
    pinMode(BTN_LEFT,  INPUT_PULLUP);
    pinMode(BTN_UP,    INPUT_PULLUP);
    pinMode(BTN_DOWN,  INPUT_PULLUP);
    pinMode(BTN_OK,    INPUT_PULLUP);
    pinMode(BTN_BACK,  INPUT_PULLUP);

    Wire.begin(SDA_PIN, SCL_PIN);
    if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR))
    {
        Serial.println("SSD1306 init failed!");
    }
    display.clearDisplay();
    display.display();

    // Splash
    display.setTextColor(SSD1306_WHITE);
    display.setTextSize(1);
    display.setCursor(20, 10);
    display.print("MJPEG Remote");
    display.setCursor(10, 25);
    display.print("Initializing...");
    display.display();
    delay(1000);

    WiFi.mode(WIFI_STA);
    WiFi.disconnect();
    Serial.print("Remote MAC: ");
    Serial.println(WiFi.macAddress());

    if (esp_now_init() != ESP_OK)
    {
        Serial.println("ESP-NOW init failed!");
        display.clearDisplay();
        display.setCursor(10, 25);
        display.print("ESP-NOW FAILED!");
        display.display();
        while (true) { delay(1000); }
    }

    esp_now_register_send_cb(onDataSent);

    esp_now_peer_info_t peerInfo = {};
    memcpy(peerInfo.peer_addr, playerMAC, 6);
    peerInfo.channel = 0;
    peerInfo.encrypt = false;

    if (esp_now_add_peer(&peerInfo) != ESP_OK)
        Serial.println("Failed to add peer!");
    else
        Serial.println("Peer added OK");

    batteryPct = readBattery();
    drawUI();
    Serial.println("Remote ready!");
}

// ─────────────────────────────────────────────────────────────────────────────
// LOOP
// ★ MODIFIED: BTN_OK long press → CMD_INFO
//             BTN_UP hold → CMD_EQ_UP, BTN_DOWN hold → CMD_EQ_DOWN
//             BTN_UP short → CMD_RESTART (tetap)
// Mapping tombol:
//   BTN_RIGHT  short → NEXT
//   BTN_LEFT   short → PREV      hold → VOL DOWN
//   BTN_UP     short → RESTART   hold → EQ UP
//   BTN_DOWN   short → SHUFFLE   hold → EQ DOWN
//   BTN_OK     short → PAUSE     long → INFO
//   BTN_BACK   short → MUTE      hold → VOL UP  ★ FIX
// ─────────────────────────────────────────────────────────────────────────────
void loop()
{
    bool needRedraw = false;
    int  pendingCmd = 0;

    // BTN_RIGHT → NEXT
    if (buttonAction(BTN_RIGHT, 0) == BTN_SHORT)
    {
        pendingCmd = CMD_NEXT;
        lastAction = ">> Next";
        needRedraw = true;
    }

    // BTN_LEFT → short=PREV, hold=VOL DOWN
    {
        int act = buttonAction(BTN_LEFT, 1);
        if (act == BTN_SHORT)
        {
            pendingCmd = CMD_PREV;
            lastAction = "<< Prev";
            needRedraw = true;
        }
        else if (act == BTN_HOLD)
        {
            pendingCmd = CMD_VOL_DOWN;
            if (volumeLevel > 0) volumeLevel--;
            lastAction = "Vol - " + String(volumeLevel) + "/10";
            needRedraw = true;
        }
    }

    // BTN_UP → short=RESTART, hold=EQ UP ★
    {
        int act = buttonAction(BTN_UP, 2);
        if (act == BTN_SHORT)
        {
            pendingCmd = CMD_RESTART;
            lastAction = "Restart";
            needRedraw = true;
        }
        else if (act == BTN_HOLD)
        {
            pendingCmd = CMD_EQ_UP;   // ★ ganti dari VOL_UP
            if (eqLevelLocal < 5) eqLevelLocal++;
            lastAction = "EQ + Lv" + String(eqLevelLocal);
            needRedraw = true;
        }
    }

    // BTN_OK → short=PAUSE, long=INFO ★
    {
        int act = buttonAction(BTN_OK, 3, INFO_LONG_MS);
        if (act == BTN_SHORT)
        {
            pendingCmd = CMD_PAUSE;
            pauseState = !pauseState;
            lastAction = pauseState ? "|| Pause" : "|> Resume";
            needRedraw = true;
        }
        else if (act == BTN_LONG || act == BTN_HOLD)
        {
            // ★ Long press BTN_OK → tampilkan info screen di player
            pendingCmd = CMD_INFO;
            lastAction = "Info Screen";
            needRedraw = true;
        }
    }

    // BTN_DOWN → short=SHUFFLE, hold=EQ DOWN ★
    {
        int act = buttonAction(BTN_DOWN, 4);
        if (act == BTN_SHORT)
        {
            pendingCmd = CMD_SHUFFLE;
            shuffleState = !shuffleState;
            lastAction = shuffleState ? "Shuffle ON" : "Shuffle OFF";
            needRedraw = true;
        }
        else if (act == BTN_HOLD)
        {
            pendingCmd = CMD_EQ_DOWN;   // ★ tambah
            if (eqLevelLocal > 0) eqLevelLocal--;
            lastAction = (eqLevelLocal == 0) ? "EQ OFF" : ("EQ - Lv" + String(eqLevelLocal));
            needRedraw = true;
        }
    }

    // BTN_BACK → short=MUTE, hold=VOL UP ★ FIX
    {
        int act = buttonAction(BTN_BACK, 5);
        if (act == BTN_SHORT)
        {
            pendingCmd = CMD_MUTE;
            muteState  = !muteState;
            lastAction = muteState ? "Mute ON" : "Mute OFF";
            needRedraw = true;
        }
        else if (act == BTN_HOLD)
        {
            pendingCmd = CMD_VOL_UP;
            if (volumeLevel < 10) volumeLevel++;
            muteState  = false;
            lastAction = "Vol + " + String(volumeLevel) + "/10";
            needRedraw = true;
        }
    }

    // Kirim command
    if (pendingCmd > 0)
        sendCommand(pendingCmd);

    // Update battery setiap 10 detik
    if (millis() - lastBatteryMs > 10000)
    {
        batteryPct    = readBattery();
        lastBatteryMs = millis();
        needRedraw    = true;

        if (battCritical)
        {
            Serial.println("WARNING: Battery critical!");
            for (int i = 0; i < 5; i++)
            {
                digitalWrite(LED_PIN, LED_ON);  delay(100);
                digitalWrite(LED_PIN, LED_OFF); delay(100);
            }
        }
    }

    if (battCritical && millis() - lastDrawMs > 500)
    {
        needRedraw  = true;
        lastDrawMs  = millis();
    }

    if (needRedraw)
    {
        lastDrawMs = millis();
        drawUI();
    }

    delay(5);
}
