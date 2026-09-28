#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <time.h>
#include <M5EchoBase.h>
#include <M5UnitOLED.h>
#include <arduinoFFT.h>
#include <PubSubClient.h>
#include "wifi_config.h" // WIFI_SSID, WIFI_PASS, MQTT_SERVER, MQTT_PORT, MQTT_TOPIC, DEVICE_ID, NTP_SERVER_PRIMARY, NTP_SERVER_SECONDARY
#include <esp_task_wdt.h>

// ============================================================
// NTP & 時刻同期設定
// ============================================================
static const long GMT_OFFSET_SEC = 9 * 3600; // JST (UTC+9)
static const int DAYLIGHT_OFFSET_SEC = 0;

// 現在日時フォーマット取得ヘルパー (YYYY-MM-DD HH:MM:SS)
String getFormattedDateTime() {
    struct tm timeinfo;
    if (getLocalTime(&timeinfo, 100)) {
        char buf[32];
        strftime(buf, sizeof(buf), "%Y-%m-%d %H:%M:%S", &timeinfo);
        return String(buf);
    }
    // NTP同期前の場合、Uptimeで代替
    unsigned long s = millis() / 1000;
    char buf[32];
    snprintf(buf, sizeof(buf), "Uptime %02lu:%02lu:%02lu", (s / 3600), (s % 3600) / 60, s % 60);
    return String(buf);
}

// ============================================================
// ハードウェアピン定義
// ============================================================
static constexpr uint8_t PIN_CODEC_SDA = 45;
static constexpr uint8_t PIN_CODEC_SCL = 0;
static constexpr uint8_t PIN_I2S_BCLK = 17;
static constexpr uint8_t PIN_I2S_WS = 3;
static constexpr uint8_t PIN_I2S_DIN = 4;
static constexpr uint8_t PIN_I2S_DOUT = 48;
static constexpr uint8_t PIN_AMP_EN = 18;

// Grove (OLED用) I2C
static constexpr uint8_t PIN_GROVE_SDA = 2;
static constexpr uint8_t PIN_GROVE_SCL = 1;

// 本体ボタン
static constexpr uint8_t PIN_BTN = 41;

// ============================================================
// パラメータ
// ============================================================
static constexpr uint32_t SAMPLE_RATE = 16000;
static constexpr size_t FFT_SAMPLES = 1024;
static constexpr size_t I2S_READ_CHUNK = 512;
static constexpr size_t RING_BUFFER_SAMPLES = 8192; // 解析用の一時バッファ(小さめでOK)

// ============================================================
// チューニング用検知ログ構造体 (RAM上リングバッファ)
// ============================================================
struct ChimeLogEntry {
    uint32_t id;
    uint32_t uptimeSec;
    char datetimeStr[32];       // "YYYY-MM-DD HH:MM:SS" (NTP未同期時は "Uptime HH:MM:SS")
    char timeStr[16];           // "HH:MM:SS" (Uptime)
    char result[12];            // "Entrance" または "Genkan"
    char reason[64];            // 判定理由詳細
    uint32_t totalDurationMs;   // 1打目Ding開始から判定確定までの時間
    uint16_t ding1Chunks;       // 1打目Dingの継続チャンク数 (1チャンク=64ms)
    uint16_t dong1Chunks;       // 1打目Dongの継続チャンク数
    float ding1MaxAmp;          // 1打目Ding最大振幅
    uint32_t ding2IntervalMs;   // 1打目Dingから2打目Ding開始までの間隔 (ms, 0=未検知)
    uint16_t ding2Chunks;       // 2打目Ding継続チャンク数
    float ding2MaxAmp;          // 2打目Ding最大振幅
    bool dong2Detected;         // 2打目Dongが確認されたか
    float maxAmpOverall;        // シーケンス全体の最大振幅
    char timeline[160];         // イベントタイムライン文字列
};

static constexpr size_t MAX_LOG_ENTRIES = 20;
static ChimeLogEntry g_logEntries[MAX_LOG_ENTRIES];
static size_t g_logCount = 0;       // 合計記録数 (リングバッファ用)
static uint32_t g_logNextId = 1;    // 連番ID
static SemaphoreHandle_t g_logMutex = nullptr;

// Webサーバー
static WebServer webServer(80);

// ============================================================
// グローバル変数
// ============================================================
static int16_t g_ringBuffer[RING_BUFFER_SAMPLES];
static volatile size_t g_writePos = 0;
static volatile size_t g_samplesAvailable = 0;
static SemaphoreHandle_t g_bufferMutex = nullptr;

static M5EchoBase echobase(I2S_NUM_0);
// OLEDはI2Cポート1 (Wire1) を使用し衝突を回避
static M5UnitOLED oled(PIN_GROVE_SDA, PIN_GROVE_SCL, 400000, 1, 0x3C);
static M5Canvas canvas(&oled);
static bool g_oledEnabled = false;

WiFiClient espClient;
PubSubClient mqttClient(espClient);
static String g_ipAddress = "Offline";

// 検知ステータス用
static volatile float g_currentFreq = 0.0f;
static volatile float g_currentAmp = 0.0f;
static String g_statusText = "IDLE";
static unsigned long g_lastDetectTime = 0;
static int g_chimeCount = 0;
static String g_lastChime = "None";
static float g_lastDetectMaxAmp = 0.0f;
static float g_currentDetectMaxAmp = 0.0f;

// MQTT送信用フラグ
static volatile bool g_pendingPublish = false;
static String g_publishType = "";

// オフライン（ネットワーク切断）タイムアウト用
static unsigned long g_offlineStartTime = 0;
static constexpr unsigned long OFFLINE_TIMEOUT = 5 * 60 * 1000; // 5 minutes

// OLED表示スリープ用
static unsigned long g_lastBtnPressTime = 0;

// ============================================================
// チャイム検知 ステートマシン定義
// ============================================================
enum DetectState {
    STATE_IDLE,
    STATE_DETECTING_DING_1,       // 1打目 Ding 検知中
    STATE_DETECTING_DONG_1,       // 1打目 Dong 検知中
    STATE_WAIT_ENTRANCE_DING,     // エントランス 2打目 Ding 待ち (時間窓: 650ms 〜 1350ms)
    STATE_CONFIRMING_DING_2,      // 2打目 Ding の継続確認 (2チャンク以上)
    STATE_CONFIRMING_DONG_2       // 2打目 Dong への移行確認 (最大400ms待機)
};

static DetectState g_detectState = STATE_IDLE;
static int g_consecutiveDing = 0;
static int g_consecutiveDong = 0;
static int g_consecutiveDing2 = 0;
static int g_missCount = 0;
static unsigned long g_firstDingTime = 0;
static unsigned long g_ding2StartTime = 0;
static unsigned long g_dong2WaitStart = 0;

static float g_ding1MaxAmp = 0.0f;
static float g_ding2MaxAmp = 0.0f;
static String g_currentTimeline = "";

static constexpr float AMP_THRESHOLD = 15000.0f; // ユーザー指定 (5000 -> 15000)
const float AMP_THRESHOLD_START = 15000.0f;
const float AMP_THRESHOLD_CONTINUE = 8000.0f;
const float AMP_THRESHOLD_DING2 = 12000.0f;

const int MIN_CHUNKS = 4;             // 1打目判定用 (4チャンク = 約256ms)
const int MIN_DING2_CHUNKS = 2;       // 2打目Ding継続判定用 (2チャンク = 約130ms)
const int MAX_MISS_TOLERANCE = 4;
const int MAX_STATE_CHUNKS = 40;      // 約2.5秒のタイムアウト

// ハイブリッド判定用 時間パラメータ (1打目Ding開始からの経過時間ms)
const unsigned long ENTRANCE_WINDOW_START_MS = 650;      // エントランス第2音の受け付け開始 (1打目Dong余韻スパイク排除)
const unsigned long ENTRANCE_DECISION_LIMIT_MS = 1350;   // 早期玄関判定リミット (エントランス第2音が来なければ玄関確定)

// ============================================================
// I2S 録音タスク
// ============================================================
void i2sRecordTask(void *pvParameters) {
    esp_task_wdt_add(NULL);

    const size_t stereoChunkBytes = I2S_READ_CHUNK * 4;
    int16_t *stereoBuffer = (int16_t *)malloc(stereoChunkBytes);
    if (!stereoBuffer) {
        Serial.println("[ERROR] i2sRecordTask malloc failed. Rebooting...");
        delay(1000);
        ESP.restart();
    }

    while (true) {
        esp_task_wdt_reset();

        bool success = echobase.record((uint8_t*)stereoBuffer, stereoChunkBytes);
        if (!success) {
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }

        if (xSemaphoreTake(g_bufferMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            for (size_t i = 0; i < I2S_READ_CHUNK; i++) {
                g_ringBuffer[g_writePos] = stereoBuffer[i * 2]; // Lch
                g_writePos = (g_writePos + 1) % RING_BUFFER_SAMPLES;
                if (g_samplesAvailable < RING_BUFFER_SAMPLES) {
                    g_samplesAvailable++;
                }
            }
            xSemaphoreGive(g_bufferMutex);
        }
    }
}

// ============================================================
// チューニング用ログ保存ヘルパー
// ============================================================
void addChimeLog(const ChimeLogEntry &entry) {
    if (g_logMutex && xSemaphoreTake(g_logMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        size_t idx = g_logCount % MAX_LOG_ENTRIES;
        g_logEntries[idx] = entry;
        g_logCount++;
        xSemaphoreGive(g_logMutex);
    }
}

// ============================================================
// 周波数解析・検知タスク
// ============================================================
void monitorTask(void *pvParameters) {
    esp_task_wdt_add(NULL);

    double *vReal = (double *)malloc(FFT_SAMPLES * sizeof(double));
    double *vImag = (double *)malloc(FFT_SAMPLES * sizeof(double));
    if (!vReal || !vImag) {
        Serial.println("[ERROR] monitorTask malloc failed. Rebooting...");
        delay(1000);
        ESP.restart();
    }

    ArduinoFFT<double> fft(vReal, vImag, FFT_SAMPLES, SAMPLE_RATE);

    while (true) {
        esp_task_wdt_reset();

        // 1チャンク(1024サンプル = 約64ms)ごとに解析
        vTaskDelay(pdMS_TO_TICKS(64));

        if (xSemaphoreTake(g_bufferMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            if (g_samplesAvailable < FFT_SAMPLES) {
                xSemaphoreGive(g_bufferMutex);
                continue;
            }

            size_t startPos;
            if (g_writePos >= FFT_SAMPLES) {
                startPos = g_writePos - FFT_SAMPLES;
            } else {
                startPos = RING_BUFFER_SAMPLES - (FFT_SAMPLES - g_writePos);
            }

            double sum = 0;
            for (size_t i = 0; i < FFT_SAMPLES; i++) {
                int16_t sample = g_ringBuffer[(startPos + i) % RING_BUFFER_SAMPLES];
                vReal[i] = sample;
                sum += sample;
            }
            xSemaphoreGive(g_bufferMutex);

            double mean = sum / FFT_SAMPLES;
            double maxAmp = 0;
            for (size_t i = 0; i < FFT_SAMPLES; i++) {
                vReal[i] -= mean;
                vImag[i] = 0.0;
                if (abs(vReal[i]) > maxAmp) {
                    maxAmp = abs(vReal[i]);
                }
            }

            g_currentAmp = maxAmp;

            bool isDing = false;
            bool isDong = false;

            if (maxAmp >= AMP_THRESHOLD) {
                fft.windowing(FFTWindow::Hamming, FFTDirection::Forward);
                fft.compute(FFTDirection::Forward);
                fft.complexToMagnitude();
                
                // 画面表示用に一番強い周波数（従来のmajorPeak）を取得
                double peakFreq = 0, peakMag = 0;
                fft.majorPeak(&peakFreq, &peakMag);
                if (isnan(peakFreq) || isinf(peakFreq)) peakFreq = 0;
                g_currentFreq = (float)peakFreq;

                // --- 複数音（和音・残響）対応の特定帯域ピーク判定 ---
                // FFT bin = 16000 / 1024 = 15.625 Hz
                double dingMax = 0;
                for (int i = 40; i <= 44; i++) { // 625Hz 〜 687Hz
                    if (vReal[i] > dingMax) dingMax = vReal[i];
                }
                
                double dongMax = 0;
                for (int i = 31; i <= 35; i++) { // 484Hz 〜 546Hz
                    if (vReal[i] > dongMax) dongMax = vReal[i];
                }

                // --- 局所的なノイズフロアの算出 ---
                // Ding周辺: 562〜609Hz(36-39) & 703〜796Hz(45-51)
                double dingNoiseSum = 0;
                for (int i = 36; i <= 39; i++) dingNoiseSum += vReal[i];
                for (int i = 45; i <= 51; i++) dingNoiseSum += vReal[i];
                double dingNoiseAvg = dingNoiseSum / 11.0;

                // Dong周辺: 406〜453Hz(26-29) & 562〜609Hz(36-39)
                double dongNoiseSum = 0;
                for (int i = 26; i <= 29; i++) dongNoiseSum += vReal[i];
                for (int i = 36; i <= 39; i++) dongNoiseSum += vReal[i];
                double dongNoiseAvg = dongNoiseSum / 8.0;

                // --- 判定条件 ---
                // 1. 絶対閾値 (ユーザー指定: 15000.0)
                double magThreshold = 15000.0;
                
                // 2. 局所S/N比 & 排他性
                // Ding: 絶対値クリア AND 周辺ノイズの2倍以上 AND Dong帯域よりも1.5倍以上大きい
                if (dingMax > magThreshold && dingMax > dingNoiseAvg * 2.0 && dingMax > dongMax * 1.5) {
                    isDing = true;
                }
                // Dong: 絶対値クリア AND 周辺ノイズの2倍以上 AND Ding帯域よりも1.5倍以上大きい
                if (dongMax > magThreshold && dongMax > dongNoiseAvg * 2.0 && dongMax > dingMax * 1.5) {
                    isDong = true;
                }
            } else {
                g_currentFreq = 0.0f;
            }
            
            if (g_detectState != STATE_IDLE && g_currentAmp > g_currentDetectMaxAmp) {
                g_currentDetectMaxAmp = g_currentAmp;
            }
            
            auto triggerDetection = [&](const char* chimeType, const char* reason, bool dong2Ok = false) {
                g_chimeCount++;
                g_lastChime = chimeType;
                g_lastDetectTime = millis();
                g_lastBtnPressTime = millis();
                g_lastDetectMaxAmp = g_currentDetectMaxAmp;
                g_statusText = "IDLE";

                unsigned long totalDuration = millis() - g_firstDingTime;
                unsigned long intervalDing2 = (g_ding2StartTime > 0) ? (g_ding2StartTime - g_firstDingTime) : 0;

                // チューニング用ログ保存
                ChimeLogEntry entry;
                memset(&entry, 0, sizeof(entry));
                entry.id = g_logNextId++;
                entry.uptimeSec = millis() / 1000;
                uint32_t s = entry.uptimeSec;
                snprintf(entry.timeStr, sizeof(entry.timeStr), "%02lu:%02lu:%02lu", (s / 3600), (s % 3600) / 60, s % 60);
                String dtStr = getFormattedDateTime();
                strncpy(entry.datetimeStr, dtStr.c_str(), sizeof(entry.datetimeStr) - 1);

                Serial.printf("=== CHIME DETECTED: %s ===\n", g_lastChime.c_str());
                Serial.printf("  Time: %s (Uptime: %s)\n", entry.datetimeStr, entry.timeStr);
                Serial.printf("  Reason: %s\n", reason);
                Serial.printf("  Duration: %lu ms, Ding2 Interval: %lu ms, MaxAmp: %.0f\n", totalDuration, intervalDing2, g_lastDetectMaxAmp);

                strncpy(entry.result, chimeType, sizeof(entry.result) - 1);
                strncpy(entry.reason, reason, sizeof(entry.reason) - 1);
                entry.totalDurationMs = totalDuration;
                entry.ding1Chunks = g_consecutiveDing;
                entry.dong1Chunks = g_consecutiveDong;
                entry.ding1MaxAmp = g_ding1MaxAmp;
                entry.ding2IntervalMs = intervalDing2;
                entry.ding2Chunks = g_consecutiveDing2;
                entry.ding2MaxAmp = g_ding2MaxAmp;
                entry.dong2Detected = dong2Ok;
                entry.maxAmpOverall = g_lastDetectMaxAmp;

                g_currentTimeline += " -> " + String(totalDuration) + "ms:" + chimeType;
                strncpy(entry.timeline, g_currentTimeline.c_str(), sizeof(entry.timeline) - 1);
                addChimeLog(entry);

                g_publishType = g_lastChime;
                g_pendingPublish = true;
                g_detectState = STATE_IDLE;
            };

            switch (g_detectState) {
                case STATE_IDLE:
                    if (millis() - g_lastDetectTime < 3000) {
                        break; // 3秒間は再検知しない (反響や多重発火防止)
                    }
                    if (isDing && g_currentAmp >= AMP_THRESHOLD_START) {
                        g_detectState = STATE_DETECTING_DING_1;
                        g_firstDingTime = millis();
                        g_ding2StartTime = 0;
                        g_dong2WaitStart = 0;
                        g_consecutiveDing = 1;
                        g_consecutiveDong = 0;
                        g_consecutiveDing2 = 0;
                        g_missCount = 0;
                        g_ding1MaxAmp = g_currentAmp;
                        g_ding2MaxAmp = 0.0f;
                        g_currentDetectMaxAmp = g_currentAmp;
                        g_statusText = "Ding 1";
                        g_currentTimeline = "0ms:Ding1(A:" + String((int)g_currentAmp) + ")";
                    }
                    break;

                case STATE_DETECTING_DING_1:
                    if (isDong) {
                        if (g_consecutiveDing >= MIN_CHUNKS) {
                            g_detectState = STATE_DETECTING_DONG_1;
                            g_consecutiveDong = 1;
                            g_missCount = 0;
                            g_statusText = "Dong 1";
                            unsigned long d1Time = millis() - g_firstDingTime;
                            g_currentTimeline += " -> " + String(d1Time) + "ms:Dong1";
                        } else {
                            // Dingが短すぎた（ノイズ）
                            g_detectState = STATE_IDLE;
                            g_statusText = "IDLE";
                        }
                    } else if (isDing) {
                        g_consecutiveDing++;
                        g_missCount = 0;
                        if (g_currentAmp > g_ding1MaxAmp) g_ding1MaxAmp = g_currentAmp;
                        if (g_consecutiveDing > MAX_STATE_CHUNKS) { // タイムアウト
                            g_detectState = STATE_IDLE;
                            g_statusText = "IDLE (Timeout)";
                        }
                    } else {
                        g_missCount++;
                        if (g_missCount > MAX_MISS_TOLERANCE || g_currentAmp < AMP_THRESHOLD_CONTINUE) {
                            g_detectState = STATE_IDLE;
                            g_statusText = "IDLE";
                        }
                    }
                    break;

                case STATE_DETECTING_DONG_1: {
                    unsigned long elapsed = millis() - g_firstDingTime;
                    if (isDong) {
                        g_consecutiveDong++;
                        g_missCount = 0;
                        if (g_consecutiveDong > MAX_STATE_CHUNKS) {
                            if (elapsed >= ENTRANCE_WINDOW_START_MS && g_consecutiveDong >= MIN_CHUNKS) {
                                g_detectState = STATE_WAIT_ENTRANCE_DING;
                                g_statusText = "Wait Ding2";
                            } else {
                                g_detectState = STATE_IDLE;
                                g_statusText = "IDLE (Timeout)";
                            }
                            break;
                        }
                    } else {
                        g_missCount++;
                    }

                    if (g_consecutiveDong >= MIN_CHUNKS) {
                        if (elapsed >= ENTRANCE_WINDOW_START_MS) {
                            g_detectState = STATE_WAIT_ENTRANCE_DING;
                            g_statusText = "Wait Ding2";
                            g_missCount = 0;
                        } else if (g_missCount > MAX_MISS_TOLERANCE || g_currentAmp < AMP_THRESHOLD_CONTINUE) {
                            g_detectState = STATE_WAIT_ENTRANCE_DING;
                            g_statusText = "Wait Ding2";
                            g_missCount = 0;
                        }
                    } else {
                        if (g_missCount > MAX_MISS_TOLERANCE || g_currentAmp < AMP_THRESHOLD_CONTINUE) {
                            g_detectState = STATE_IDLE;
                            g_statusText = "IDLE";
                        }
                    }
                    break;
                }

                case STATE_WAIT_ENTRANCE_DING: {
                    unsigned long elapsed = millis() - g_firstDingTime;

                    // 【早期判定】1350ms経過してもエントランス第2音が来なければ玄関確定！
                    if (elapsed >= ENTRANCE_DECISION_LIMIT_MS) {
                        triggerDetection("Genkan", "Timeout: No Ding2 within 1350ms");
                        break;
                    }

                    // 【エントランス候補】時間窓内(>=650ms)でDing2候補を検知
                    if (elapsed >= ENTRANCE_WINDOW_START_MS && isDing && g_currentAmp >= AMP_THRESHOLD_DING2) {
                        g_detectState = STATE_CONFIRMING_DING_2;
                        g_ding2StartTime = millis();
                        g_consecutiveDing2 = 1;
                        g_ding2MaxAmp = g_currentAmp;
                        g_statusText = "Ding 2 Check";
                        g_currentTimeline += " -> " + String(elapsed) + "ms:Ding2?(A:" + String((int)g_currentAmp) + ")";
                    }
                    break;
                }

                case STATE_CONFIRMING_DING_2: {
                    unsigned long elapsed = millis() - g_firstDingTime;

                    if (isDing && g_currentAmp >= AMP_THRESHOLD_CONTINUE) {
                        g_consecutiveDing2++;
                        if (g_currentAmp > g_ding2MaxAmp) g_ding2MaxAmp = g_currentAmp;

                        // 2チャンク以上継続で本物のDing2と確認
                        if (g_consecutiveDing2 >= MIN_DING2_CHUNKS) {
                            g_detectState = STATE_CONFIRMING_DONG_2;
                            g_dong2WaitStart = millis();
                            g_statusText = "Dong 2 Check";
                            g_currentTimeline += " -> Ding2_OK(" + String(g_consecutiveDing2) + "ch)";
                        }
                    } else {
                        // 単発スパイクノイズだった
                        g_currentTimeline += " -> Ding2_Drop";
                        g_detectState = STATE_WAIT_ENTRANCE_DING;
                        g_statusText = "Wait Ding2";

                        if (elapsed >= ENTRANCE_DECISION_LIMIT_MS) {
                            triggerDetection("Genkan", "Timeout: Ding2 was noise spike");
                        }
                    }
                    break;
                }

                case STATE_CONFIRMING_DONG_2: {
                    unsigned long elapsed = millis() - g_firstDingTime;
                    unsigned long waitDongTime = millis() - g_dong2WaitStart;

                    if (isDong) {
                        unsigned long d2DongTime = millis() - g_firstDingTime;
                        g_currentTimeline += " -> " + String(d2DongTime) + "ms:Dong2";
                        triggerDetection("Entrance", "Ding2 confirmed + Dong detected", true);
                        break;
                    }

                    if (isDing) {
                        g_consecutiveDing2++;
                        if (g_currentAmp > g_ding2MaxAmp) g_ding2MaxAmp = g_currentAmp;
                        if (g_consecutiveDing2 >= 6) { // 約380ms以上Ding継続
                            triggerDetection("Entrance", "Ding2 long duration confirmed", false);
                            break;
                        }
                    }

                    // Dong待機タイムアウト (約400ms待ってもDongが現れなかった場合)
                    // エントランス見逃し防止のため、Ding2が確認できていればEntranceとする
                    if (waitDongTime > 400) {
                        triggerDetection("Entrance", "Ding2 confirmed (Dong weak)", false);
                        break;
                    }
                    break;
                }
            }
        }
    }
}

// ============================================================
// OLED 描画タスク
// ============================================================
void oledDisplayTask(void *pvParameters) {
    esp_task_wdt_add(NULL);

    bool isSleeping = false;
    while (true) {
        esp_task_wdt_reset();
        vTaskDelay(pdMS_TO_TICKS(200));

        // 30秒経過したら画面を消灯して処理をスキップ
        if (millis() - g_lastBtnPressTime > 30000) {
            if (!isSleeping) {
                canvas.fillScreen(TFT_BLACK);
                canvas.pushSprite(0, 0);
                isSleeping = true;
            }
            continue;
        }
        isSleeping = false;

        canvas.fillScreen(TFT_BLACK);
        canvas.setTextColor(TFT_WHITE);
        
        canvas.setTextSize(1);
        canvas.setCursor(0, 0);
        canvas.print("Chime Detect");
        
        // カウンタを右上に表示
        canvas.setCursor(canvas.width() - 45, 0);
        canvas.printf("Cnt:%d", g_chimeCount);

        canvas.drawFastHLine(0, 10, canvas.width(), TFT_WHITE);
        
        canvas.setCursor(0, 14);
        canvas.printf("IP: %s", g_ipAddress.c_str());

        // 検知ステータス (直近の検知結果を5秒間大きく表示)
        if (millis() - g_lastDetectTime < 5000 && g_lastDetectTime > 0) {
            canvas.setTextColor(TFT_WHITE);
            canvas.setTextSize(2);
            canvas.setCursor(0, 24);
            canvas.print(g_lastChime);
        } else {
            canvas.setTextColor(TFT_WHITE);
            canvas.setTextSize(1);
            canvas.setCursor(0, 24);
            canvas.printf("State: %s", g_statusText.c_str());
            canvas.setCursor(0, 34);
            canvas.printf("Last : %s", g_lastChime.c_str());
        }

        // リアルタイム情報
        canvas.setTextColor(TFT_WHITE);
        canvas.setTextSize(1);
        canvas.setCursor(0, 44);
        canvas.printf("Freq: %4.0f Hz", g_currentFreq);
        canvas.setCursor(0, 54);
        canvas.printf("Amp: %.0f M:%.0f", g_currentAmp, g_lastDetectMaxAmp);

        canvas.pushSprite(0, 0);
    }
}

// ============================================================
// Web サーバー初期化
// ============================================================
void setupWebServer() {
    webServer.on("/", HTTP_GET, []() {
        String html;
        html.reserve(8192);
        html += "<!DOCTYPE html><html lang=\"ja\"><head><meta charset=\"UTF-8\">";
        html += "<meta name=\"viewport\" content=\"width=device-width, initial-scale=1.0\">";
        html += "<title>Chime Detect Dashboard</title>";
        html += "<style>";
        html += ":root { --bg: #0f172a; --card: #1e293b; --text: #f8fafc; --sub: #94a3b8; --border: #334155; --accent: #38bdf8; }";
        html += "* { box-sizing: border-box; margin: 0; padding: 0; }";
        html += "body { font-family: -apple-system, BlinkMacSystemFont, 'Segoe UI', Roboto, sans-serif; background: var(--bg); color: var(--text); padding: 20px; line-height: 1.5; }";
        html += ".container { max-width: 1100px; margin: 0 auto; }";
        html += "header { display: flex; justify-content: space-between; align-items: center; margin-bottom: 20px; flex-wrap: wrap; gap: 12px; }";
        html += "h1 { font-size: 1.5rem; color: var(--accent); }";
        html += ".grid { display: grid; grid-template-columns: repeat(auto-fit, minmax(180px, 1fr)); gap: 14px; margin-bottom: 20px; }";
        html += ".card { background: var(--card); border: 1px solid var(--border); border-radius: 10px; padding: 14px; }";
        html += ".card-title { font-size: 0.75rem; color: var(--sub); text-transform: uppercase; margin-bottom: 4px; }";
        html += ".card-val { font-size: 1.2rem; font-weight: bold; }";
        html += ".actions { display: flex; gap: 10px; margin-bottom: 20px; flex-wrap: wrap; }";
        html += ".btn { display: inline-flex; align-items: center; padding: 8px 16px; border-radius: 6px; font-size: 0.85rem; font-weight: 600; text-decoration: none; cursor: pointer; border: none; }";
        html += ".btn-pri { background: #0284c7; color: white; } .btn-pri:hover { background: #0369a1; }";
        html += ".btn-sec { background: #334155; color: white; } .btn-sec:hover { background: #475569; }";
        html += ".btn-dan { background: #dc2626; color: white; } .btn-dan:hover { background: #b91c1c; }";
        html += "table { width: 100%; border-collapse: collapse; margin-top: 8px; font-size: 0.82rem; }";
        html += "th, td { padding: 9px 10px; text-align: left; border-bottom: 1px solid var(--border); }";
        html += "th { background: #162032; color: var(--sub); font-weight: 600; }";
        html += "tr:hover { background: rgba(255,255,255,0.02); }";
        html += ".badge { display: inline-block; padding: 2px 7px; border-radius: 4px; font-weight: bold; font-size: 0.75rem; }";
        html += ".badge-ent { background: rgba(16,185,129,0.2); color: #34d399; border: 1px solid rgba(16,185,129,0.4); }";
        html += ".badge-gen { background: rgba(245,158,11,0.2); color: #fbbf24; border: 1px solid rgba(245,158,11,0.4); }";
        html += ".tl-code { font-family: monospace; font-size: 0.75rem; color: #a5b4fc; background: #0b1120; padding: 3px 6px; border-radius: 4px; display: block; overflow-x: auto; white-space: nowrap; max-width: 280px; }";
        html += "</style></head><body><div class=\"container\">";
        
        html += "<header><div><h1>🔔 Chime Detect Dashboard</h1>";
        html += "<p style=\"color:var(--sub);font-size:0.85rem;\">Atom VoiceS3R Hybrid Recognition System</p></div>";
        html += "<div><a href=\"/\" class=\"btn btn-sec\">🔄 Refresh</a></div></header>";
        
        unsigned long upSec = millis() / 1000;
        char upStr[32];
        snprintf(upStr, sizeof(upStr), "%02luh %02lum %02lus", upSec / 3600, (upSec % 3600) / 60, upSec % 60);
        String curTimeStr = getFormattedDateTime();
        
        html += "<div class=\"grid\">";
        html += "<div class=\"card\"><div class=\"card-title\">Current Time (JST)</div><div class=\"card-val\" style=\"font-size:1.05rem;\">" + curTimeStr + "</div></div>";
        html += "<div class=\"card\"><div class=\"card-title\">IP Address</div><div class=\"card-val\">" + g_ipAddress + "</div></div>";
        html += "<div class=\"card\"><div class=\"card-title\">Uptime</div><div class=\"card-val\">" + String(upStr) + "</div></div>";
        html += "<div class=\"card\"><div class=\"card-title\">Detections (Last)</div><div class=\"card-val\">" + String(g_chimeCount) + " (" + g_lastChime + ")</div></div>";
        html += "<div class=\"card\"><div class=\"card-title\">Heap / RSSI</div><div class=\"card-val\">" + String(ESP.getFreeHeap() / 1024) + " KB / " + String(WiFi.RSSI()) + " dBm</div></div>";
        html += "</div>";
        
        html += "<div class=\"actions\">";
        html += "<a href=\"/logs.csv\" class=\"btn btn-pri\" download>📥 Download CSV</a>";
        html += "<a href=\"/logs.json\" class=\"btn btn-pri\" download>📥 Download JSON</a>";
        html += "<a href=\"/clear\" onclick=\"return confirm('ログを消去しますか？');\" class=\"btn btn-dan\">🗑️ Clear Logs</a>";
        html += "</div>";
        
        html += "<div class=\"card\" style=\"overflow-x:auto;\">";
        html += "<h2 style=\"font-size:1.05rem;margin-bottom:8px;\">Recent Detections (Max " + String(MAX_LOG_ENTRIES) + " in RAM)</h2>";
        
        if (g_logCount == 0) {
            html += "<p style=\"color:var(--sub);padding:16px 0;\">No chime events recorded yet. Waiting for chime sound...</p>";
        } else {
            html += "<table><thead><tr>";
            html += "<th>#</th><th>Date / Time</th><th>Result</th><th>Reason</th><th>Total</th><th>Ding2 Int</th><th>Ding1 Ch/Amp</th><th>Ding2 Ch/Amp</th><th>Timeline</th>";
            html += "</tr></thead><tbody>";
            
            if (g_logMutex && xSemaphoreTake(g_logMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
                size_t displayCount = (g_logCount < MAX_LOG_ENTRIES) ? g_logCount : MAX_LOG_ENTRIES;
                for (size_t i = 0; i < displayCount; i++) {
                    size_t idx = (g_logCount - 1 - i) % MAX_LOG_ENTRIES;
                    const ChimeLogEntry &e = g_logEntries[idx];
                    
                    html += "<tr>";
                    html += "<td>" + String(e.id) + "</td>";
                    html += "<td style=\"white-space:nowrap;\">" + String(e.datetimeStr) + "</td>";
                    if (strcmp(e.result, "Entrance") == 0) {
                        html += "<td><span class=\"badge badge-ent\">Entrance</span></td>";
                    } else {
                        html += "<td><span class=\"badge badge-gen\">Genkan</span></td>";
                    }
                    html += "<td>" + String(e.reason) + "</td>";
                    html += "<td>" + String(e.totalDurationMs) + "ms</td>";
                    if (e.ding2IntervalMs > 0) {
                        html += "<td>" + String(e.ding2IntervalMs) + "ms</td>";
                    } else {
                        html += "<td style=\"color:var(--sub);\">-</td>";
                    }
                    html += "<td>" + String(e.ding1Chunks) + "ch/" + String((int)e.ding1MaxAmp) + "</td>";
                    if (e.ding2Chunks > 0) {
                        html += "<td>" + String(e.ding2Chunks) + "ch/" + String((int)e.ding2MaxAmp) + " (" + (e.dong2Detected ? "Dong OK" : "Dong -") + ")</td>";
                    } else {
                        html += "<td style=\"color:var(--sub);\">-</td>";
                    }
                    html += "<td><code class=\"tl-code\">" + String(e.timeline) + "</code></td>";
                    html += "</tr>";
                }
                xSemaphoreGive(g_logMutex);
            }
            html += "</tbody></table>";
        }
        
        html += "</div></div></body></html>";
        webServer.send(200, "text/html", html);
    });

    webServer.on("/logs.csv", HTTP_GET, []() {
        String csv;
        csv.reserve(4096);
        csv = "id,datetime,uptime_sec,result,reason,total_duration_ms,ding1_chunks,dong1_chunks,ding1_max_amp,ding2_interval_ms,ding2_chunks,ding2_max_amp,dong2_detected,max_amp_overall,timeline\r\n";
        
        if (g_logMutex && xSemaphoreTake(g_logMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
            size_t count = (g_logCount < MAX_LOG_ENTRIES) ? g_logCount : MAX_LOG_ENTRIES;
            size_t startIdx = (g_logCount > MAX_LOG_ENTRIES) ? (g_logCount % MAX_LOG_ENTRIES) : 0;
            for (size_t i = 0; i < count; i++) {
                size_t idx = (startIdx + i) % MAX_LOG_ENTRIES;
                const ChimeLogEntry &e = g_logEntries[idx];
                csv += String(e.id) + ",";
                csv += "\"" + String(e.datetimeStr) + "\",";
                csv += String(e.uptimeSec) + ",";
                csv += "\"" + String(e.result) + "\",";
                csv += "\"" + String(e.reason) + "\",";
                csv += String(e.totalDurationMs) + ",";
                csv += String(e.ding1Chunks) + ",";
                csv += String(e.dong1Chunks) + ",";
                csv += String((int)e.ding1MaxAmp) + ",";
                csv += String(e.ding2IntervalMs) + ",";
                csv += String(e.ding2Chunks) + ",";
                csv += String((int)e.ding2MaxAmp) + ",";
                csv += String(e.dong2Detected ? "true" : "false") + ",";
                csv += String((int)e.maxAmpOverall) + ",";
                csv += "\"" + String(e.timeline) + "\"\r\n";
            }
            xSemaphoreGive(g_logMutex);
        }
        
        webServer.sendHeader("Content-Disposition", "attachment; filename=\"chime_logs.csv\"");
        webServer.send(200, "text/csv; charset=utf-8", csv);
    });

    webServer.on("/logs.json", HTTP_GET, []() {
        String json;
        json.reserve(4096);
        json = "[\r\n";
        if (g_logMutex && xSemaphoreTake(g_logMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
            size_t count = (g_logCount < MAX_LOG_ENTRIES) ? g_logCount : MAX_LOG_ENTRIES;
            size_t startIdx = (g_logCount > MAX_LOG_ENTRIES) ? (g_logCount % MAX_LOG_ENTRIES) : 0;
            for (size_t i = 0; i < count; i++) {
                size_t idx = (startIdx + i) % MAX_LOG_ENTRIES;
                const ChimeLogEntry &e = g_logEntries[idx];
                json += "  {\r\n";
                json += "    \"id\": " + String(e.id) + ",\r\n";
                json += "    \"datetime\": \"" + String(e.datetimeStr) + "\",\r\n";
                json += "    \"uptime_sec\": " + String(e.uptimeSec) + ",\r\n";
                json += "    \"time\": \"" + String(e.timeStr) + "\",\r\n";
                json += "    \"result\": \"" + String(e.result) + "\",\r\n";
                json += "    \"reason\": \"" + String(e.reason) + "\",\r\n";
                json += "    \"total_duration_ms\": " + String(e.totalDurationMs) + ",\r\n";
                json += "    \"ding1_chunks\": " + String(e.ding1Chunks) + ",\r\n";
                json += "    \"dong1_chunks\": " + String(e.dong1Chunks) + ",\r\n";
                json += "    \"ding1_max_amp\": " + String(e.ding1MaxAmp, 1) + ",\r\n";
                json += "    \"ding2_interval_ms\": " + String(e.ding2IntervalMs) + ",\r\n";
                json += "    \"ding2_chunks\": " + String(e.ding2Chunks) + ",\r\n";
                json += "    \"ding2_max_amp\": " + String(e.ding2MaxAmp, 1) + ",\r\n";
                json += "    \"dong2_detected\": " + String(e.dong2Detected ? "true" : "false") + ",\r\n";
                json += "    \"max_amp_overall\": " + String(e.maxAmpOverall, 1) + ",\r\n";
                json += "    \"timeline\": \"" + String(e.timeline) + "\"\r\n";
                json += "  }" + String(i + 1 < count ? "," : "") + "\r\n";
            }
            xSemaphoreGive(g_logMutex);
        }
        json += "]\r\n";
        
        webServer.sendHeader("Content-Disposition", "attachment; filename=\"chime_logs.json\"");
        webServer.send(200, "application/json", json);
    });

    webServer.on("/clear", []() {
        if (g_logMutex && xSemaphoreTake(g_logMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
            g_logCount = 0;
            xSemaphoreGive(g_logMutex);
        }
        webServer.sendHeader("Location", "/");
        webServer.send(302, "text/plain", "Cleared");
    });

    webServer.begin();
    Serial.println("[Web] HTTP Server started on port 80.");
}

// ============================================================
// setup()
// ============================================================
void setup() {
    Serial.begin(115200);
    delay(500);
    Serial.println("=== Chime Detect System ===");

    // --- ボタン初期化 ---
    pinMode(PIN_BTN, INPUT_PULLUP);
    g_lastBtnPressTime = millis(); // 起動直後は30秒表示する

    // --- OLED接続確認と初期化 (Grove I2C) ---
    Wire1.begin(PIN_GROVE_SDA, PIN_GROVE_SCL);
    Wire1.setTimeOut(50);
    Wire1.beginTransmission(0x3C);
    if (Wire1.endTransmission() == 0) {
        g_oledEnabled = true;
        Serial.println("[OLED] Module detected at 0x3C.");
        oled.init();
        oled.setRotation(1);
        canvas.setColorDepth(1);
        canvas.createSprite(oled.width(), oled.height());
        canvas.setTextWrap(true);
        canvas.fillScreen(TFT_BLACK);
        canvas.setTextColor(TFT_WHITE);
        canvas.setCursor(0, 0);
        canvas.print("Initializing...");
        canvas.pushSprite(0, 0);
    } else {
        g_oledEnabled = false;
        Serial.println("[OLED] Module not detected. Display skipped.");
    }

    // --- ミューテックス作成 ---
    g_bufferMutex = xSemaphoreCreateMutex();
    g_logMutex = xSemaphoreCreateMutex();

    // --- I2S / マイク初期化 ---
    Wire.begin(PIN_CODEC_SDA, PIN_CODEC_SCL);
    bool i2s_ok = echobase.init(SAMPLE_RATE, PIN_CODEC_SDA, PIN_CODEC_SCL, PIN_I2S_DIN,
                                PIN_I2S_WS, PIN_I2S_DOUT, PIN_I2S_BCLK, Wire);
    if (!i2s_ok) {
        Serial.println("[I2S] Init Failed.");
    } else {
        // 大音量によるクリッピングを防ぐためゲインを12dBに変更 (従来は24dB)
        echobase.setMicGain(ES8311_MIC_GAIN_12DB);
    }
    pinMode(PIN_AMP_EN, OUTPUT);
    digitalWrite(PIN_AMP_EN, LOW); 
    echobase.setMute(false);

    // --- Wi-Fi接続 ---
    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_SSID, WIFI_PASS);
    
    int retryCount = 0;
    while (WiFi.status() != WL_CONNECTED && retryCount < 20) {
        delay(500);
        Serial.print(".");
        retryCount++;
    }
    if (WiFi.status() == WL_CONNECTED) {
        Serial.println("\n[WiFi] Connected.");
        g_ipAddress = WiFi.localIP().toString();

        // --- NTP時刻同期設定 (JST UTC+9) ---
        configTime(GMT_OFFSET_SEC, DAYLIGHT_OFFSET_SEC, NTP_SERVER_PRIMARY, NTP_SERVER_SECONDARY);
        Serial.printf("[NTP] Time synchronization configured (Primary: %s, Secondary: %s)\n", NTP_SERVER_PRIMARY, NTP_SERVER_SECONDARY);

        setupWebServer();
    } else {
        Serial.println("\n[WiFi] Timeout.");
    }

    // --- MQTT初期設定 ---
    mqttClient.setServer(MQTT_SERVER, MQTT_PORT);

    // --- Watchdog Timer 初期化 (10秒) ---
    esp_task_wdt_init(10, true);
    esp_task_wdt_add(NULL); // loop() タスクを登録

    // --- タスク起動 ---
    xTaskCreatePinnedToCore(i2sRecordTask, "I2S_REC", 8192, NULL, 3, NULL, 1);
    xTaskCreatePinnedToCore(monitorTask, "MONITOR", 8192, NULL, 2, NULL, 0);
    if (g_oledEnabled) {
        xTaskCreatePinnedToCore(oledDisplayTask, "OLED", 4096, NULL, 1, NULL, 0);
    }

    Serial.println("System Ready.");
}

// 非ブロッキングのMQTT再接続処理
void reconnectMQTT() {
    if (WiFi.status() != WL_CONNECTED) return;
    
    static unsigned long lastReconnectAttempt = 0;
    if (!mqttClient.connected()) {
        if (g_offlineStartTime == 0) {
            g_offlineStartTime = millis();
        } else if (millis() - g_offlineStartTime > OFFLINE_TIMEOUT) {
            Serial.println("[ERROR] MQTT Offline timeout. Rebooting...");
            ESP.restart();
        }

        if (millis() - lastReconnectAttempt > 5000) {
            lastReconnectAttempt = millis();
            Serial.print("[MQTT] Attempting connection...");
            // LWT設定: QoS 0, Retain true, Payload "offline"
            if (mqttClient.connect(DEVICE_ID, MQTT_TOPIC_STATUS, 0, true, "offline")) {
                Serial.println("connected");
                // 接続成功時に Birth Message (online) を Retain true で送信
                mqttClient.publish(MQTT_TOPIC_STATUS, "online", true);
            } else {
                Serial.print("failed, rc=");
                Serial.println(mqttClient.state());
            }
        }
    }
}

void loop() {
    if (WiFi.status() == WL_CONNECTED) {
        // WebServer クライアントリクエスト処理
        webServer.handleClient();

        if (!mqttClient.connected()) {
            reconnectMQTT();
        } else {
            g_offlineStartTime = 0; // 正常時はリセット
            mqttClient.loop();
            
            // 検知イベントがあればパブリッシュ
            if (g_pendingPublish) {
                g_pendingPublish = false;
                String payload = "{\"event\":\"chime_detected\",\"type\":\"" + g_publishType + "\",\"device_id\":\"" + String(DEVICE_ID) + "\",\"value\":1}";
                
                if (mqttClient.publish(MQTT_TOPIC, payload.c_str())) {
                    Serial.println("[MQTT] Message Published: " + payload);
                } else {
                    Serial.println("[MQTT] Message Publish Failed!");
                }
            }

            // テレメトリー (ハートビート) の定期送信 (5分=300000ms毎)
            static unsigned long lastTelemetryTime = 0;
            if (millis() - lastTelemetryTime > 300000) {
                lastTelemetryTime = millis();
                unsigned long uptimeSec = millis() / 1000;
                long rssi = WiFi.RSSI();
                uint32_t freeHeap = ESP.getFreeHeap();

                String telemetryPayload = "{\"uptime\":" + String(uptimeSec) + ",\"rssi\":" + String(rssi) + ",\"free_heap\":" + String(freeHeap) + "}";
                if (mqttClient.publish(MQTT_TOPIC_TELEMETRY, telemetryPayload.c_str())) {
                    Serial.println("[MQTT] Telemetry Published: " + telemetryPayload);
                } else {
                    Serial.println("[MQTT] Telemetry Publish Failed!");
                }
            }
        }
    } else {
        // Wi-Fi 切断時
        if (g_offlineStartTime == 0) {
            g_offlineStartTime = millis();
        } else if (millis() - g_offlineStartTime > OFFLINE_TIMEOUT) {
            Serial.println("[ERROR] WiFi Offline timeout. Rebooting...");
            ESP.restart();
        }
        
        static unsigned long lastWifiAttempt = 0;
        if (millis() - lastWifiAttempt > 5000) {
            lastWifiAttempt = millis();
            Serial.println("[WiFi] Reconnecting...");
            WiFi.disconnect();
            WiFi.begin(WIFI_SSID, WIFI_PASS);
        }
    }

    // ボタンが押されたらOLED表示を30秒延長 (AtomS3のボタンはLOWアクティブ)
    if (digitalRead(PIN_BTN) == LOW) {
        g_lastBtnPressTime = millis();
    }

    esp_task_wdt_reset();
    delay(10);
}
