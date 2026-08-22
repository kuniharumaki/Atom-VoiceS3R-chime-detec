# Atom-VoiceS3R-chime-detect

M5Stack Atom VoiceS3R を使用して、インターホンのチャイム音（玄関・エントランス）をマイクで集音し、周波数解析（FFT）によって検知・分類するスマートホーム向けデバイスのファームウェアです。
検知した結果は OLED に表示されるほか、ローカルネットワーク内の MQTT ブローカーに通知されます。

## 機能
- **チャイム音検知**: 
  - I2Sマイクからの入力をFFTで解析。
  - 652Hz（Ding）と513Hz（Dong）の周波数を監視。倍音やノイズの影響を排除するため、周波数のピーク強度に対する相対割合で検知。
  - 第1音と第2音の合計鳴動時間（チャンク数）から「玄関（長め）」と「エントランス（短め）」を自動判別。
- **OLED表示**:
  - 検知回数、直近の検知結果、リアルタイムの周波数・振幅を表示。
  - 起動時の OLED 自動検出に対応。未接続の場合は表示タスクをスキップして起動可能。
- **MQTT通知**:
  - 検知時に指定した MQTT トピックへ JSON 形式でデータをパブリッシュ。
  - ネットワーク切断時の自動再接続機能を実装。

## ハードウェア要件
- **メインボード**: M5Stack Atom VoiceS3R (ESP32-S3)
- **マイクベース**: M5Atomic-EchoBase (I2S接続)
- **ディスプレイ (オプション)**: M5Stack 1.3インチ OLEDモジュール (SH1107) - Grove (I2C) 接続

## 開発環境
- [PlatformIO](https://platformio.org/) (VSCode 拡張機能推奨)
- **フレームワーク**: Arduino

## 依存ライブラリ
- `m5stack/M5GFX`
- `m5stack/M5Atomic-EchoBase`
- `kosme/arduinoFFT`
- `knolleary/PubSubClient`

## セットアップ手順

1. **リポジトリのクローン**:
   ```bash
   git clone <repository_url>
   cd Atom-VoiceS3R-chime-detect
   ```

2. **設定ファイルの作成**:
   `src/wifi_config.h` を作成し、Wi-Fi および MQTT の設定を記述してください。（このファイルは Git の管理対象から除外されています）

   ```cpp
   // src/wifi_config.h の例
   #pragma once

   const char *WIFI_SSID = "YOUR_WIFI_SSID";
   const char *WIFI_PASS = "YOUR_WIFI_PASSWORD";

   const char *MQTT_SERVER = "192.168.x.x"; // ブローカーのIPアドレス
   const int MQTT_PORT = 1883;
   const char *MQTT_TOPIC = "home/sensors/intercom/state";
   const char *DEVICE_ID = "atom_voice_intercom";
   ```

3. **ビルドと書き込み**:
   PlatformIO を使用してビルドおよびデバイスへの書き込みを行ってください。
   ```bash
   pio run -t upload
   ```

## MQTT ペイロード仕様
チャイムを検知すると、以下のような JSON が送信されます。
```json
{
  "event": "chime_detected",
  "type": "Genkan",
  "device_id": "atom_voice_intercom",
  "value": 1
}
```
※ `type` は `"Genkan"` または `"Entrance"` になります。
