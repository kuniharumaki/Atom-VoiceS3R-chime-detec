#include <Arduino.h>
#include <WiFi.h>
#include <M5EchoBase.h>
#include <M5UnitOLED.h>
#include <arduinoFFT.h>
#include <PubSubClient.h>
#include "wifi_config.h" // WIFI_SSID, WIFI_PASS, MQTT_SERVER, MQTT_PORT, MQTT_TOPIC, DEVICE_ID
#include <esp_task_wdt.h>

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
    STATE_DETECTING_DING,
    STATE_DETECTING_DONG,
    STATE_WAIT_DING_2
};

static DetectState g_detectState = STATE_IDLE;
static int g_consecutiveDing = 0;
static int g_consecutiveDong = 0;
static int g_missCount = 0;
static unsigned long g_firstDingTime = 0;

static constexpr float AMP_THRESHOLD = 15000.0f; // ユーザー指定 (5000 -> 15000)
const float AMP_THRESHOLD_START = 15000.0f;
const float AMP_THRESHOLD_CONTINUE = 8000.0f;
const float AMP_THRESHOLD_DING2 = 12000.0f;
const int MIN_CHUNKS = 4;
const int MAX_MISS_TOLERANCE = 4;
const int MAX_STATE_CHUNKS = 40; // 約1.5秒のタイムアウト

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
            
            auto triggerDetection = [&]() {
                g_chimeCount++;
                unsigned long interval = millis() - g_firstDingTime;
                if (interval > 2000) {
                    g_lastChime = "Genkan";
                } else {
                    g_lastChime = "Entrance";
                }
                g_lastDetectTime = millis();
                g_lastBtnPressTime = millis();
                g_lastDetectMaxAmp = g_currentDetectMaxAmp;
                g_statusText = "IDLE";
                Serial.printf("=== CHIME DETECTED: %s (Interval:%lu ms, MaxAmp:%.0f) ===\n", g_lastChime.c_str(), interval, g_lastDetectMaxAmp);
                g_publishType = g_lastChime;
                g_pendingPublish = true;
                g_detectState = STATE_IDLE;
            };

            switch (g_detectState) {
                case STATE_IDLE:
                    if (millis() - g_lastDetectTime < 3000) {
                        break; // 3秒間は再検知しない (反響や2回目の鳴動による上書き防止)
                    }
                    if (isDing && g_currentAmp >= AMP_THRESHOLD_START) {
                        g_detectState = STATE_DETECTING_DING;
                        g_firstDingTime = millis();
                        g_consecutiveDing = 1;
                        g_missCount = 0;
                        g_currentDetectMaxAmp = g_currentAmp;
                        g_statusText = "Detecting DING";
                    }
                    break;

                case STATE_DETECTING_DING:
                    // Dongの成分が見えたら、Dingが残響していてもDongフェーズへ優先して移行する
                    if (isDong) {
                        if (g_consecutiveDing >= MIN_CHUNKS) {
                            g_detectState = STATE_DETECTING_DONG;
                            g_consecutiveDong = 1;
                            g_missCount = 0;
                            g_statusText = "Detecting DONG";
                        } else {
                            // Dingが短すぎた場合はリセット
                            g_detectState = STATE_IDLE;
                            g_statusText = "IDLE";
                        }
                    } else if (isDing) {
                        g_consecutiveDing++;
                        g_missCount = 0;
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

                case STATE_DETECTING_DONG:
                    if (isDing && g_consecutiveDong >= MIN_CHUNKS && g_currentAmp >= AMP_THRESHOLD_DING2) {
                        if (millis() - g_firstDingTime > 500) {
                            triggerDetection();
                            break;
                        }
                    }
                    if (isDong) {
                        g_consecutiveDong++;
                        g_missCount = 0;
                        if (g_consecutiveDong > MAX_STATE_CHUNKS) { // タイムアウト
                            g_detectState = STATE_IDLE;
                            g_statusText = "IDLE (Timeout)";
                        }
                    } else {
                        g_missCount++;
                        if (g_missCount > MAX_MISS_TOLERANCE || g_currentAmp < AMP_THRESHOLD_CONTINUE) {
                            if (g_consecutiveDong >= MIN_CHUNKS) {
                                g_detectState = STATE_WAIT_DING_2;
                                g_statusText = "Waiting DING 2";
                            } else {
                                g_detectState = STATE_IDLE;
                                g_statusText = "IDLE";
                            }
                        }
                    }
                    break;

                case STATE_WAIT_DING_2:
                    if (millis() - g_firstDingTime > 6000) { // 6秒待っても来ない場合はタイムアウト
                        g_detectState = STATE_IDLE;
                        g_statusText = "IDLE (Timeout)";
                    } else if (isDing && g_currentAmp >= AMP_THRESHOLD_DING2) {
                        if (millis() - g_firstDingTime > 500) {
                            triggerDetection();
                        }
                    }
                    break;
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
