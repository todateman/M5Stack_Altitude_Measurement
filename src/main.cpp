/*
 * M5Stack Altitude Measurement with EKF Sensor Fusion
 * =====================================================
 * Hardware:
 *   - M5Stack Basic v2.7 (ESP32) ※ 内蔵 IMU なし
 *       - 320x240 LCD
 *   - M5Stack GNSS Module (底面 M-BUS 接続)
 *       - GNSS (u-blox): Serial2 GPIO16(RX)/17(TX), 38400 baud
 *       - 気圧センサ: BMP280, I2C addr 0x76
 *       - IMU: BMI270, I2C addr 0x68 ← こちらを使用
 *
 * EKF 状態ベクトル: x = [高度(m), 垂直速度(m/s)]
 *   予測ステップ: IMU 垂直加速度を入力として状態遷移
 *   更新ステップ: BMP280 高度 (高頻度) & GNSS 高度 (低頻度) で逐次更新
 *
 * 表示:
 *   - GNSS 単体高度
 *   - 気圧計単体高度 (BMP280)
 *   - EKF 融合高度 (GNSS + BMP280 + BMI270)
 *
 * 参考:
 *   https://github.com/m5stack/M5Module-GNSS/tree/main/examples
 */

#include <Arduino.h>
#include <M5Unified.h>
#include <TinyGPSPlus.h>
#include <SD.h>
#include <SPI.h>
#include "Capture.h"

// ============================================================
// ハードウェア設定
// ============================================================

// GNSS Module: GPS UART (Serial2)
static constexpr uint32_t GPS_BAUD = 38400;
static constexpr int      GPS_RX   = 16;   // M-BUS RXD
static constexpr int      GPS_TX   = 17;   // M-BUS TXD

// GNSS Module: BMP280 気圧センサ (I2C)
static constexpr float    SEA_LEVEL_HPA = 1013.25f; // 標準大気圧 [hPa]


// ============================================================
// EKF チューニングパラメータ
// ============================================================

// BMP280 は分解能 ~0.12m だが気温变化でドリフトあり
static constexpr float R_BARO = 0.09f;   // std ≈ 0.3 m

// プロセスノイズ: IMU 加速度計ノイズ標準偏差 [m/s²]
// BMI270 のノイズ特性に合わせて調整 (大きいほど IMU を信頼しない)
static constexpr float Q_SIGMA_A = 0.3f;

// 観測ノイズ分散 [m²]
// GNSS 高度は精度 ~5m (ビル内/受信状況による)
static constexpr float R_GNSS = 25.0f;   // std ≈ 5.0 m

// ============================================================
// BMP280 ミニドライバ (M5.In_I2C 直接使用 / Adafruit 不要)
// M5GFX が i2c_new_master_bus で I2C_NUM_0 を確保済みのため
// Arduino Wire は同ポートへの二重初期化で失敗する。
// M5.In_I2C は M5GFX のバスハンドルを共有するため競合しない。
// ============================================================
struct BMP280Driver {
    static constexpr uint8_t  ADDR = 0x76;
    static constexpr uint32_t FREQ = 400000UL;

    uint16_t dig_T1; int16_t dig_T2, dig_T3;
    uint16_t dig_P1; int16_t dig_P2, dig_P3, dig_P4, dig_P5, dig_P6, dig_P7, dig_P8, dig_P9;
    int32_t  t_fine = 0;

    bool begin() {
        uint8_t id = M5.In_I2C.readRegister8(ADDR, 0xD0, FREQ);
        if (id != 0x58 && id != 0x56 && id != 0x57) return false;

        M5.In_I2C.writeRegister8(ADDR, 0xE0, 0xB6, FREQ); // reset
        delay(10);

        uint8_t c[24];
        M5.In_I2C.readRegister(ADDR, 0x88, c, 24, FREQ);
        dig_T1=(uint16_t)((c[1]<<8)|c[0]); dig_T2=(int16_t)((c[3]<<8)|c[2]); dig_T3=(int16_t)((c[5]<<8)|c[4]);
        dig_P1=(uint16_t)((c[7]<<8)|c[6]); dig_P2=(int16_t)((c[9]<<8)|c[8]); dig_P3=(int16_t)((c[11]<<8)|c[10]);
        dig_P4=(int16_t)((c[13]<<8)|c[12]); dig_P5=(int16_t)((c[15]<<8)|c[14]); dig_P6=(int16_t)((c[17]<<8)|c[16]);
        dig_P7=(int16_t)((c[19]<<8)|c[18]); dig_P8=(int16_t)((c[21]<<8)|c[20]); dig_P9=(int16_t)((c[23]<<8)|c[22]);

        M5.In_I2C.writeRegister8(ADDR, 0xF4, 0x57, FREQ); // osrs_t=x2, osrs_p=x16, mode=normal
        M5.In_I2C.writeRegister8(ADDR, 0xF5, 0xB0, FREQ); // standby=500ms, filter=x16
        delay(100);
        return true;
    }

    float readAltitude(float seaLevel_hPa) {
        uint8_t buf[6];
        M5.In_I2C.readRegister(ADDR, 0xF7, buf, 6, FREQ);
        int32_t adc_P = (int32_t)((buf[0]<<12)|(buf[1]<<4)|(buf[2]>>4));
        int32_t adc_T = (int32_t)((buf[3]<<12)|(buf[4]<<4)|(buf[5]>>4));

        int32_t v1 = ((((adc_T>>3)-((int32_t)dig_T1<<1)))*((int32_t)dig_T2))>>11;
        int32_t v2 = (((((adc_T>>4)-((int32_t)dig_T1))*((adc_T>>4)-((int32_t)dig_T1)))>>12)*((int32_t)dig_T3))>>14;
        t_fine = v1 + v2;

        double d1=(double)t_fine/2.0-64000.0, d2=d1*d1*(double)dig_P6/32768.0+d1*(double)dig_P5*2.0;
        d2=d2/4.0+(double)dig_P4*65536.0;
        d1=((double)dig_P3*d1*d1/524288.0+(double)dig_P2*d1)/524288.0;
        d1=(1.0+d1/32768.0)*(double)dig_P1;
        if (d1 == 0.0) return 0.0f;
        double p=1048576.0-(double)adc_P;
        p=(p-d2/4096.0)*6250.0/d1;
        d1=(double)dig_P9*p*p/2147483648.0; d2=p*(double)dig_P8/32768.0;
        p+=(d1+d2+(double)dig_P7)/16.0;

        return 44330.0f*(1.0f-powf((float)(p/100.0)/seaLevel_hPa, 0.1903f));
    }
};

// ============================================================
// 2 状態 EKF クラス
// ============================================================
//
//  状態：x = [h (altitude), v (vertical velocity)]^T
//
//  遷移モデル (dt 秒ごとに予測):
//    h_{k+1} = h_k + dt * v_k + 0.5 * dt² * a_z
//    v_{k+1} = v_k + dt * a_z
//      a_z: IMU から取得した重力補正済み垂直加速度 [m/s²]
//
//  観測モデル (高度センサ共通):
//    z = h_k + noise    → H = [1, 0]
//

class AltitudeEKF {
public:
    float x[2];       // 状態ベクトル: [高度, 垂直速度]
    float P[2][2];    // 共分散行列
    bool  initialized = false;

    // 初期化 (初期高度を設定)
    void init(float h0) {
        x[0] = h0;
        x[1] = 0.0f;
        P[0][0] = 100.0f; P[0][1] = 0.0f;
        P[1][0] = 0.0f;   P[1][1] = 10.0f;
        initialized = true;
    }

    // 予測ステップ: IMU 垂直加速度 az [m/s²], 時間刻み dt [s]
    void predict(float az, float dt) {
        // --- 状態遷移 ---
        float new_h = x[0] + dt * x[1] + 0.5f * dt * dt * az;
        float new_v = x[1] + dt * az;
        x[0] = new_h;
        x[1] = new_v;

        // --- 共分散遷移 P = F * P * F^T + Q ---
        // F = [[1, dt], [0, 1]]
        // Q = σ_a² * [[dt⁴/4,  dt³/2],
        //              [dt³/2,  dt²  ]]
        float qa  = Q_SIGMA_A * Q_SIGMA_A;
        float dt2 = dt * dt;
        float dt3 = dt2 * dt;
        float dt4 = dt3 * dt;

        float p00 = P[0][0], p01 = P[0][1];
        float p10 = P[1][0], p11 = P[1][1];

        // F * P の各要素
        float fp00 = p00 + dt * p10;
        float fp01 = p01 + dt * p11;
        float fp10 = p10;
        float fp11 = p11;

        // (F * P) * F^T + Q
        P[0][0] = fp00 + dt * fp01 + 0.25f * dt4 * qa;
        P[0][1] = fp01             + 0.5f  * dt3 * qa;
        P[1][0] = fp10 + dt * fp11 + 0.5f  * dt3 * qa;
        P[1][1] = fp11             +          dt2 * qa;
    }

    // 更新ステップ: 高度観測 z [m], 観測ノイズ分散 R [m²]
    void update(float z, float R) {
        // H = [1, 0] → S = H * P * H^T + R = P[0][0] + R
        float S  = P[0][0] + R;
        float K0 = P[0][0] / S;   // カルマンゲイン (高度)
        float K1 = P[1][0] / S;   // カルマンゲイン (速度)

        float innov = z - x[0];   // イノベーション (残差)
        x[0] += K0 * innov;
        x[1] += K1 * innov;

        // P = (I - K * H) * P (Joseph 形式も可だが簡略版で十分)
        float p00 = P[0][0], p01 = P[0][1];
        float p10 = P[1][0], p11 = P[1][1];
        P[0][0] = (1.0f - K0) * p00;
        P[0][1] = (1.0f - K0) * p01;
        P[1][0] = p10 - K1 * p00;
        P[1][1] = p11 - K1 * p01;
    }

    // 推定高度 [m]
    float altitude() const { return x[0]; }
    // 推定垂直速度 [m/s]
    float velocity() const { return x[1]; }
    // 高度推定分散 (精度の目安) [m²]
    float altVariance() const { return P[0][0]; }
};

// ============================================================
// グローバル変数
// ============================================================

TinyGPSPlus   gps;
BMP280Driver  bmp;
AltitudeEKF   ekf;

// 各センサ高度 [m]
float altGNSS = 0.0f;
float altBaro = 0.0f;
float altEKF  = 0.0f;

// センサ状態フラグ
bool gnssValid    = false;
bool baroValid    = false;
bool baroCalibrated = false;

// 気圧計キャリブレーション: GNSS 初回 Fix 時に offset を補正
float baroOffset = 0.0f;
float baroBootRaw = 0.0f;

// 衛星数
int satCount = 0;

// タイマー
unsigned long lastLoopMs  = 0;
unsigned long lastDispMs  = 0;
unsigned long lastLogMs   = 0;

bool sdReady = false;
static constexpr int SD_CS_PIN = 4;
static constexpr const char* LOG_PATH = "/altitude_log.csv";

int counter;
char fn[100];

static void buildTimestamp(char* out, size_t outSize)
{
    if (gnssValid && gps.date.isValid() && gps.time.isValid()) {
        snprintf(out, outSize, "%04d-%02d-%02d %02d:%02d:%02d",
                 gps.date.year(), gps.date.month(), gps.date.day(),
                 gps.time.hour(), gps.time.minute(), gps.time.second());
        return;
    }
    unsigned long sec = millis() / 1000UL;
    unsigned long hh = sec / 3600UL;
    unsigned long mm = (sec % 3600UL) / 60UL;
    unsigned long ss = sec % 60UL;
    snprintf(out, outSize, "UPTIME %02lu:%02lu:%02lu", hh, mm, ss);
}

static void appendCsvLog()
{
    if (!sdReady) return;

    char ts[24];
    buildTimestamp(ts, sizeof(ts));
    char latStr[20] = "";
    char lonStr[20] = "";
    char gnssAltStr[16] = "";

    if (gnssValid && gps.location.isValid()) {
        snprintf(latStr, sizeof(latStr), "%.7f", gps.location.lat());
        snprintf(lonStr, sizeof(lonStr), "%.7f", gps.location.lng());
    }
    if (gnssValid && gps.altitude.isValid()) {
        snprintf(gnssAltStr, sizeof(gnssAltStr), "%.2f", altGNSS);
    }

    File file = SD.open(LOG_PATH, FILE_APPEND);
    if (!file) return;

    file.printf(
        "%s,%lu,%s,%s,%s,%.2f,%.2f,%.3f,%.4f,%d,%d,%d,%d\n",
        ts,
        millis(),
        latStr,
        lonStr,
        gnssAltStr,
        altBaro,
        altEKF,
        ekf.initialized ? ekf.velocity() : 0.0f,
        ekf.initialized ? ekf.altVariance() : 0.0f,
        satCount,
        gnssValid ? 1 : 0,
        baroValid ? 1 : 0,
        ekf.initialized ? 1 : 0
    );
    file.close();
}


// ============================================================
// 表示定数 (320x240 横向き)
// ============================================================
static constexpr int W = 320;
static constexpr int H = 240;

// 行 Y 座標
static constexpr int ROW_HEADER   = 0;
static constexpr int ROW_GNSS     = 19;
static constexpr int ROW_BARO     = 92;
static constexpr int ROW_EKF      = 165;
static constexpr int ROW_STATUS   = 216;
static constexpr int ROW_HEIGHT   = 72;  // 各センサ行の高さ

// 色
static constexpr uint16_t COL_GNSS   = 0x07E0; // TFT_GREEN
static constexpr uint16_t COL_BARO   = 0xFFE0; // TFT_YELLOW
static constexpr uint16_t COL_EKF    = 0xFD20; // TFT_ORANGE
static constexpr uint16_t COL_DIM    = 0x4208; // 暗いグレー (仕切り線)
static constexpr uint16_t COL_STATUS = 0xAD75; // 薄いグレー

// ============================================================
// 1 センサ行を描画するヘルパー
// ============================================================
static void drawSensorRow(int yTop, uint16_t color,
                          const char* label, const char* sublabel,
                          float value, bool valid,
                          const char* extra = nullptr)
{
    // ラベル
    M5.Display.setTextSize(2);
    M5.Display.setTextColor(color, TFT_BLACK);
    M5.Display.drawString(label, 6, yTop + 4);

    // サブラベル (右端)
    if (sublabel) {
        M5.Display.setTextSize(1);
        M5.Display.setTextColor(COL_STATUS, TFT_BLACK);
        int sw = strlen(sublabel) * 6;  // textSize(1) = 6px/char
        M5.Display.drawString(sublabel, W - sw - 4, yTop + 4);
    }

    // 値
    M5.Display.setTextSize(3);
    if (valid) {
        char buf[24];
        snprintf(buf, sizeof(buf), "%.2f m", value);
        M5.Display.setTextColor(color, TFT_BLACK);
        M5.Display.drawString(buf, 6, yTop + 24);
    } else {
        M5.Display.setTextColor(COL_DIM, TFT_BLACK);
        M5.Display.drawString("---.-- m", 6, yTop + 24);
    }

    // 追加情報 (EKF 行: 推定速度など)
    if (extra) {
        M5.Display.setTextSize(1);
        M5.Display.setTextColor(COL_STATUS, TFT_BLACK);
        M5.Display.drawString(extra, 6, yTop + 56);
    }

    // 下仕切り線
    M5.Display.drawLine(0, yTop + ROW_HEIGHT - 1, W - 1, yTop + ROW_HEIGHT - 1, COL_DIM);
}

// ============================================================
// 画面全体を描画 (スプライト経由でフリッカーなし)
// ============================================================
static void drawDisplay()
{
    M5.Display.startWrite();
    M5.Display.fillScreen(TFT_BLACK);

    // --- ヘッダー ---
    M5.Display.setTextSize(1);
    M5.Display.setTextColor(0x07FF, TFT_BLACK);  // シアン
    M5.Display.drawString("M5Stack EKF Altitude Fusion", 6, 4);
    M5.Display.drawLine(0, ROW_HEADER + 18, W - 1, ROW_HEADER + 18, COL_DIM);

    // --- GNSS 行 ---
    char gnssExtra[32] = "";
    if (gnssValid) {
        snprintf(gnssExtra, sizeof(gnssExtra), "Sats:%d", satCount);
    }
    drawSensorRow(ROW_GNSS, COL_GNSS,
                  "GNSS Only", gnssValid ? gnssExtra : "No Fix",
                  altGNSS, gnssValid);

    // --- 気圧計行 ---
    char baroLbl[32] = "";
    if (baroCalibrated) {
        snprintf(baroLbl, sizeof(baroLbl), "BMP280 (cal)");
    } else {
        snprintf(baroLbl, sizeof(baroLbl), "BMP280 (rel)");
    }
    drawSensorRow(ROW_BARO, COL_BARO,
                  "Baro Only", baroLbl,
                  altBaro, baroValid);

    // --- EKF 融合行 ---
    char ekfExtra[48] = "";
    if (ekf.initialized) {
        snprintf(ekfExtra, sizeof(ekfExtra),
                 "vel:%.2f m/s  P:%.3f m²",
                 ekf.velocity(), ekf.altVariance());
    }
    drawSensorRow(ROW_EKF, COL_EKF,
                  "EKF Fusion", "GNSS+BMP280+BMI270",
                  altEKF, ekf.initialized,
                  ekfExtra);

    // --- ステータスバー ---
    char statusBuf[80];
    if (ekf.initialized && gnssValid && baroValid) {
        float diffGNSS = altGNSS - altEKF;
        float diffBaro = altBaro - altEKF;
        snprintf(statusBuf, sizeof(statusBuf),
                 "GNSS-EKF:%+.2fm  Baro-EKF:%+.2fm",
                 diffGNSS, diffBaro);
    } else {
        snprintf(statusBuf, sizeof(statusBuf),
                 "GNSS:%s  BMP280:%s  Calib:%s",
                 gnssValid  ? "OK" : "wait",
                 baroValid  ? "OK" : "err",
                 baroCalibrated ? "OK" : "wait");
    }
    M5.Display.setTextSize(1);
    M5.Display.setTextColor(COL_STATUS, TFT_BLACK);
    M5.Display.drawString(statusBuf, 4, ROW_STATUS + 4);

    M5.Display.endWrite();
}

// ============================================================
// 起動時スプラッシュ
// ============================================================
static void showSplash(const char* msg, uint16_t color = TFT_WHITE)
{
    M5.Display.fillScreen(TFT_BLACK);
    M5.Display.setTextColor(color);
    M5.Display.setTextSize(2);
    int tw = strlen(msg) * 12;
    M5.Display.drawString(msg, (W - tw) / 2, H / 2 - 8);
}

// ============================================================
// setup()
// ============================================================
void setup()
{
    // M5Stack 初期化
    auto cfg = M5.config();
    M5.begin(cfg);
    M5.Display.setRotation(1);  // 横向き
    M5.Display.setBrightness(128);

    counter = 0;

    showSplash("Initializing...");

    // デバッグシリアル
    Serial.begin(115200);
    Serial.println("\n=== M5Stack EKF Altitude Fusion ===");

    sdReady = SD.begin(SD_CS_PIN, SPI, 25000000);
    if (sdReady) {
        bool needHeader = !SD.exists(LOG_PATH);
        File file = SD.open(LOG_PATH, FILE_APPEND);
        if (file) {
            if (needHeader || file.size() == 0) {
                file.println("timestamp_utc,uptime_ms,lat_deg,lon_deg,alt_gnss_m,alt_baro_m,alt_ekf_m,vel_mps,ekf_var_m2,sats,gnss_valid,baro_valid,ekf_initialized");
            }
            file.close();
            Serial.printf("SD: logging to %s\n", LOG_PATH);
        } else {
            sdReady = false;
            Serial.println("SD: open log file failed");
        }
    } else {
        Serial.println("SD: init failed");
    }

    // GPS UART (M5Stack GNSS Module)
    Serial2.begin(GPS_BAUD, SERIAL_8N1, GPS_RX, GPS_TX);
    Serial.printf("GPS: Serial2 RX=%d TX=%d baud=%d\n", GPS_RX, GPS_TX, GPS_BAUD);

    // BMP280 初期化 (M5.In_I2C 経由 / Wire 不使用)
    if (!bmp.begin()) {
        showSplash("BMP280 Error!", TFT_RED);
        Serial.println("ERROR: BMP280 not found at 0x76");
        // BMP280 なしで継続 (GNSS + IMU の 2 センサ融合にフォールバック)
    } else {
        baroValid = true;
        Serial.println("BMP280: OK");

        // 起動直後の気圧高度で EKF を初期化
        float rawAlt = bmp.readAltitude(SEA_LEVEL_HPA);
        baroBootRaw = rawAlt;
        baroOffset = -baroBootRaw;  // GNSS未Fix時は起動地点を 0m とする
        altBaro = rawAlt + baroOffset;
        ekf.init(altBaro);
        Serial.printf("EKF initialized: h0=%.2f m (baro relative, raw=%.2f m)\n", altBaro, rawAlt);
    }

    // IMU 初期化 (M5Unified が I2C バスをスキャンして GNSS Module 内蔵 BMI270 を自動検出)
    M5.Imu.init();
    Serial.println("IMU: BMI270 OK (via M5Unified)");

    lastLoopMs = millis();

    showSplash("Ready!", TFT_GREEN);
    delay(800);
}

// ============================================================
// loop()
// ============================================================
void loop()
{
    M5.update();

    // GPS データをパーサーへ供給
    while (Serial2.available() > 0) {
        gps.encode(Serial2.read());
    }

    unsigned long now = millis();
    float dt = (now - lastLoopMs) * 1e-3f;

    // 最小ループ間隔: 20 ms (50 Hz)
    if (dt < 0.02f) return;
    lastLoopMs = now;

    // ==========================================
    // 1. IMU 読み出し (GNSS Module 内蔵 BMI270 / M5Unified 経由)
    // ==========================================
    float ax = 0.0f, ay = 0.0f, az = 0.0f;
    M5.Imu.getAccel(&ax, &ay, &az);

    // GNSS Module (M5Stack 底面) の BMI270 は Z 軸が下向き: 静止時 az ≈ -1.0 g
    // 重力成分を除去して上向き正の垂直加速度を得る [m/s²]
    float az_inertial = -(az + 1.0f) * 9.80665f;

    // ==========================================
    // 2. BMP280 気圧高度 読み出し
    // ==========================================
    float rawBaro = 0.0f;
    if (baroValid) {
        rawBaro = bmp.readAltitude(SEA_LEVEL_HPA);
        altBaro = rawBaro + baroOffset;
    }

    // ==========================================
    // 3. GNSS 高度 読み出し & キャリブレーション
    // ==========================================
    bool gnssUpdated = false;
    if (gps.location.isValid() && gps.altitude.isValid()) {
        altGNSS = (float)gps.altitude.meters();
        satCount = gps.satellites.isValid() ? (int)gps.satellites.value() : 0;
        static uint32_t lastFixSentenceCount = 0;
        uint32_t fixSentenceCount = gps.sentencesWithFix();
        gnssUpdated = (fixSentenceCount != lastFixSentenceCount);
        if (gnssUpdated) {
            lastFixSentenceCount = fixSentenceCount;
        }

        if (!gnssValid) {
            Serial.printf("GNSS Fix! alt=%.2f m, sats=%d\n", altGNSS, satCount);
        }
        gnssValid = true;

        // 気圧高度オフセット補正: BMP280 の絶対値誤差を GNSS で校正
        // (BMP280 は相対変化は精度高いが絶対値は実際の海面気圧に依存するためオフセット補正)
        if (baroValid && gnssUpdated) {
            if (!baroCalibrated) {
                // 初回: 最初の有効 GNSS 更新で即時に絶対高度へ合わせる
                baroOffset = altGNSS - rawBaro;
                altBaro    = rawBaro + baroOffset;
                ekf.x[0]   = altGNSS;
                ekf.P[0][0] = 1.0f;
                baroCalibrated = true;
                Serial.printf("Baro calibrated: offset=%.2f m  (GNSS=%.2f, rawBaro=%.2f)\n",
                              baroOffset, altGNSS, rawBaro);
            } else {
                // キャリブレーション後: 気圧の長期ドリフトを緩やかに補正 (τ ≈ 100 sec)
                baroOffset += 0.01f * ((altGNSS - rawBaro) - baroOffset);
                altBaro = rawBaro + baroOffset;
            }
        }
    }

    // ==========================================
    // 4. EKF: 予測 → 更新
    // ==========================================
    if (ekf.initialized) {
        // 予測: IMU 加速度で状態を前進
        ekf.predict(az_inertial, dt);

        // 更新: BMP280 (常時、高頻度)
        if (baroValid) {
            ekf.update(altBaro, R_BARO);
        }

        // 更新: GNSS (新データが来たときのみ)
        if (gnssValid && gnssUpdated) {
            ekf.update(altGNSS, R_GNSS);
        }

        altEKF = ekf.altitude();
    } else if (baroValid) {
        // BMP280 が有効になった時点で EKF を初期化
        ekf.init(altBaro);
    }

    // ==========================================
    // 5. 表示更新 (5 Hz = 200 ms 毎)
    // ==========================================
    if (now - lastDispMs >= 200) {
        lastDispMs = now;
        drawDisplay();

        // シリアルデバッグ出力
        Serial.printf(
            "GNSS=%7.2f  Baro=%7.2f  EKF=%7.2f  vel=%+6.3f  P=%.4f  sats=%d\n",
            altGNSS, altBaro, altEKF,
            ekf.initialized ? ekf.velocity() : 0.0f,
            ekf.initialized ? ekf.altVariance() : 0.0f,
            satCount
        );
    }

    // ==========================================
    // 6. ボタン A で スクリーンキャプチャ
    // ==========================================
    if(M5.BtnA.wasPressed()) {
      sprintf(fn, "/Capture%d.bmp", counter);
      Screen_Capture_BMP(fn);
      counter++;
    }
    
    if (now - lastLogMs >= 1000) {
        lastLogMs = now;
        appendCsvLog();
    }
}
