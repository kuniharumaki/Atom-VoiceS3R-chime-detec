# Atom-VoiceS3R-chime-detect

M5Stack Atom VoiceS3R を使用して、インターホンのチャイム音（玄関・エントランス）をマイクで集音し、周波数解析（FFT）によって検知・分類するスマートホーム向けデバイスのファームウェアです。
検知した結果は OLED に表示されるほか、ローカルネットワーク内の MQTT ブローカーに通知されます。

## 機能
- **チャイム音検知**: 
  - I2Sマイクからの入力をFFTで解析。
  - 652Hz（Ding）と513Hz（Dong）の周波数を監視。
  - **ノイズ対策・誤検知防止**: 洗濯機などの機械音（ブロードバンドノイズ）による誤検知を防ぐため、以下のロジックを採用しています。
    - **絶対音量閾値の引き上げ**: 至近距離のチャイムなど、物理的に大きな音にのみ反応させ、生活環境音をカット。
    - **局所的S/N比の評価**: 対象周波数の強度が「隣接する周波数帯のノイズレベル」に対して突出しているかを判定（純音の検知）。
    - **排他性チェック**: DingとDongの帯域に同時にノイズが乗るケースを弾くため、互いの成分の強さを比較。
    - **タイムアウト処理**: ノイズにより片方のステートに長時間留まった場合は強制リセットし、その直後の本物のチャイムを逃さない設計。
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
   const char *MQTT_TOPIC_STATUS = "home/sensors/intercom/status";
   const char *MQTT_TOPIC_TELEMETRY = "home/sensors/intercom/telemetry";
   const char *DEVICE_ID = "atom_voice_intercom";
   ```

3. **ビルドと書き込み**:
   PlatformIO を使用してビルドおよびデバイスへの書き込みを行ってください。
   ```bash
   pio run -t upload
   ```

## MQTT ペイロード仕様

### チャイム検知 (state)
チャイムを検知すると、指定した `MQTT_TOPIC` へ以下の JSON が送信されます。
```json
{
  "event": "chime_detected",
  "type": "Genkan",
  "device_id": "atom_voice_intercom",
  "value": 1
}
```
※ `type` は `"Genkan"` または `"Entrance"` になります。

### 接続ステータス (status)
デバイスの接続状態を `MQTT_TOPIC_STATUS` へ送信します。
- 接続時 (Birth Message): `online` (Retain: true)
- 異常切断時 (LWT): `offline` (Retain: true)

### テレメトリー / ハートビート (telemetry)
5分に1回、`MQTT_TOPIC_TELEMETRY` へデバイス状態の JSON が送信されます。
```json
{
  "uptime": 300,
  "rssi": -55,
  "free_heap": 204800
}
```
