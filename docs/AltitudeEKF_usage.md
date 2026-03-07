# AltitudeEKF Usage Guide

This document explains how to use `include/AltitudeEKF.h` in other projects.

## 1. Convert Barometric Data to Altitude

`AltitudeEKF.h` provides two ways to calculate altitude.

### 1.1 From pressure only

Use this when your sensor driver already provides compensated pressure in Pa.

```cpp
#include "AltitudeEKF.h"

static constexpr float SEA_LEVEL_HPA = 1013.25f;

float altitudeFromPressure(float pressurePa) {
    return AltitudeMath::pressureToAltitudeMeters(pressurePa, SEA_LEVEL_HPA);
}
```

- Input: pressure in Pa
- Input: sea-level pressure in hPa
- Output: altitude in meters

### 1.2 Directly from raw BMP280 ADC values

Use this when you have raw `adcPressure`/`adcTemperature` and calibration constants.

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

- `adcPressure` and `adcTemperature` are 20-bit assembled BMP280 raw values.
- `tFine` is optional and can be `nullptr` if not needed.

## 2. Correct Altitude with EKF

`AltitudeEKF` is a 2-state filter:

- State: altitude `h` [m]
- State: vertical velocity `v` [m/s]

Typical loop order:

1. Predict with IMU vertical acceleration
2. Update with barometric altitude (high rate)
3. Update with GNSS altitude when a new fix arrives (low rate)

### 2.1 Basic setup

```cpp
#include "AltitudeEKF.h"

AltitudeEKF ekf;

void setupFilter(float initialAltitudeM) {
    ekf.setProcessAccelSigma(0.3f);
    ekf.setInitialCovariance(100.0f, 10.0f);
    ekf.init(initialAltitudeM);
}
```

### 2.2 Prediction + measurement updates

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

    static constexpr float R_BARO = 0.09f;  // variance [m^2]
    static constexpr float R_GNSS = 25.0f;  // variance [m^2]

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

### 2.3 Align to absolute reference (optional)

If barometric offset drifts, align EKF altitude to GNSS once fix quality is acceptable.

```cpp
void alignToGnss(float gnssAltitudeM) {
    ekf.alignAltitude(gnssAltitudeM, 1.0f);
}
```

## 3. Practical Tuning Notes

- Increase `R_BARO` if barometric output is noisy.
- Increase `R_GNSS` if GNSS altitude jumps.
- Increase `setProcessAccelSigma()` if IMU acceleration is noisy.
- Decrease `setProcessAccelSigma()` if motion response is too slow.

## 4. Units Checklist

- IMU input to `predict()`: `m/s^2`
- `dt`: `s`
- Altitude observations: `m`
- Measurement noise `R`: `m^2`
- Sea-level reference pressure: `hPa`
