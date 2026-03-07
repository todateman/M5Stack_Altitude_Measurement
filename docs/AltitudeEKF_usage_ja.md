# AltitudeEKF 利用ガイド

このドキュメントでは、`include/AltitudeEKF.h` を他プロジェクトで使う方法を説明します。

## 1. 気圧データから高度を求める

`AltitudeEKF.h` には高度算出の方法が2つあります。

### 1.1 補正済み気圧（Pa）から求める

センサドライバが補正済み気圧を Pa で返す場合に使います。

```cpp
#include "AltitudeEKF.h"

static constexpr float SEA_LEVEL_HPA = 1013.25f;

float altitudeFromPressure(float pressurePa) {
    return AltitudeMath::pressureToAltitudeMeters(pressurePa, SEA_LEVEL_HPA);
}
```

- 入力: 気圧 [Pa]
- 入力: 海面更正気圧 [hPa]
- 出力: 高度 [m]

### 1.2 BMP280 の生ADC値から直接求める

`adcPressure` / `adcTemperature` とキャリブレーション定数がある場合に使います。

```cpp
#include "AltitudeEKF.h"

static constexpr float SEA_LEVEL_HPA = 1013.25f;

AltitudeMath::BMP280Calibration cal {
    dig_T1, dig_T2, dig_T3,
    dig_P1, dig_P2, dig_P3, dig_P4, dig_P5,
    dig_P6, dig_P7, dig_P8, dig_P9
};

int32_t tFine = 0;
float altitudeM = AltitudeMath::readAltitude(
    cal,
    adcPressure,
    adcTemperature,
    SEA_LEVEL_HPA,
    &tFine
);
```

- `adcPressure` と `adcTemperature` は、20-bit組み立て済みのBMP280生データです。
- `tFine` は不要なら `nullptr` を渡せます。

## 2. EKFで高度を補正する

`AltitudeEKF` は2状態フィルタです。

- 状態: 高度 `h` [m]
- 状態: 垂直速度 `v` [m/s]

典型的な1ループの順序:

1. IMUの鉛直加速度で予測
2. 気圧高度で更新（高頻度）
3. GNSS高度で更新（新規データ到着時のみ）

### 2.1 基本初期化

```cpp
#include "AltitudeEKF.h"

AltitudeEKF ekf;

void setupFilter(float initialAltitudeM) {
    ekf.setProcessAccelSigma(0.3f);
    ekf.setInitialCovariance(100.0f, 10.0f);
    ekf.init(initialAltitudeM);
}
```

### 2.2 予測 + 観測更新

```cpp
void updateFilter(float dt,
                  float imuVerticalAccelMps2,
                  float baroAltitudeM,
                  bool baroValid,
                  float gnssAltitudeM,
                  bool gnssUpdated) {
    if (!ekf.isInitialized()) {
        return;
    }

    static constexpr float R_BARO = 0.09f;  // 分散 [m^2]
    static constexpr float R_GNSS = 25.0f;  // 分散 [m^2]

    ekf.predict(imuVerticalAccelMps2, dt);

    if (baroValid) {
        ekf.update(baroAltitudeM, R_BARO);
    }

    if (gnssUpdated) {
        ekf.update(gnssAltitudeM, R_GNSS);
    }

    float fusedAltitudeM = ekf.altitude();
    float fusedVelocityMps = ekf.velocity();
    float altitudeVar = ekf.altVariance();

    (void)fusedAltitudeM;
    (void)fusedVelocityMps;
    (void)altitudeVar;
}
```

### 2.3 絶対高度への再アライン（任意）

気圧オフセットが長期ドリフトした場合、GNSSを基準にEKF高度を合わせられます。

```cpp
void alignToGnss(float gnssAltitudeM) {
    ekf.alignAltitude(gnssAltitudeM, 1.0f);
}
```

## 3. 調整の目安

- 気圧出力のノイズが大きい場合は `R_BARO` を上げる
- GNSS高度のジャンプが大きい場合は `R_GNSS` を上げる
- IMU加速度ノイズが大きい場合は `setProcessAccelSigma()` を上げる
- 動きへの追従が遅い場合は `setProcessAccelSigma()` を下げる

## 4. 単位チェック

- `predict()` に渡すIMU入力: `m/s^2`
- `dt`: `s`
- 高度観測値: `m`
- 観測ノイズ `R`: `m^2`
- 海面更正気圧: `hPa`
