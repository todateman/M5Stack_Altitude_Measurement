#pragma once

#include <stdint.h>
#include <math.h>

namespace AltitudeMath {

struct BMP280Calibration {
    uint16_t digT1;
    int16_t digT2;
    int16_t digT3;
    uint16_t digP1;
    int16_t digP2;
    int16_t digP3;
    int16_t digP4;
    int16_t digP5;
    int16_t digP6;
    int16_t digP7;
    int16_t digP8;
    int16_t digP9;
};

// Convert pressure to altitude using the international barometric formula.
// pressurePa: measured pressure [Pa]
// seaLevelHpa: sea-level reference pressure [hPa]
inline float pressureToAltitudeMeters(float pressurePa, float seaLevelHpa) {
    if (pressurePa <= 0.0f || seaLevelHpa <= 0.0f) {
        return 0.0f;
    }

    const float pressureHpa = pressurePa * 0.01f;
    return 44330.0f * (1.0f - powf(pressureHpa / seaLevelHpa, 0.1903f));
}

// Calculate altitude from raw BMP280 ADC values and calibration constants.
// adcPressure: uncompensated pressure ADC value (20-bit assembled)
// adcTemperature: uncompensated temperature ADC value (20-bit assembled)
// seaLevelHpa: sea-level reference pressure [hPa]
// tFineOut: optional pointer to receive t_fine for temperature compensation chain
inline float readAltitude(const BMP280Calibration& cal,
                          int32_t adcPressure,
                          int32_t adcTemperature,
                          float seaLevelHpa,
                          int32_t* tFineOut = nullptr) {
    int32_t v1 = ((((adcTemperature >> 3) - ((int32_t)cal.digT1 << 1))) * ((int32_t)cal.digT2)) >> 11;
    int32_t v2 = (((((adcTemperature >> 4) - ((int32_t)cal.digT1)) * ((adcTemperature >> 4) - ((int32_t)cal.digT1))) >> 12) * ((int32_t)cal.digT3)) >> 14;
    int32_t tFine = v1 + v2;
    if (tFineOut) {
        *tFineOut = tFine;
    }

    double d1 = (double)tFine / 2.0 - 64000.0;
    double d2 = d1 * d1 * (double)cal.digP6 / 32768.0 + d1 * (double)cal.digP5 * 2.0;
    d2 = d2 / 4.0 + (double)cal.digP4 * 65536.0;
    d1 = ((double)cal.digP3 * d1 * d1 / 524288.0 + (double)cal.digP2 * d1) / 524288.0;
    d1 = (1.0 + d1 / 32768.0) * (double)cal.digP1;
    if (d1 == 0.0) {
        return 0.0f;
    }

    double pressurePa = 1048576.0 - (double)adcPressure;
    pressurePa = (pressurePa - d2 / 4096.0) * 6250.0 / d1;
    d1 = (double)cal.digP9 * pressurePa * pressurePa / 2147483648.0;
    d2 = pressurePa * (double)cal.digP8 / 32768.0;
    pressurePa += (d1 + d2 + (double)cal.digP7) / 16.0;

    return pressureToAltitudeMeters((float)pressurePa, seaLevelHpa);
}

}  // namespace AltitudeMath

// Generic 1D altitude EKF with 2-state model:
// x = [altitude(m), vertical_velocity(m/s)]^T
class AltitudeEKF {
public:
    AltitudeEKF() = default;

    // Process acceleration noise sigma [m/s^2].
    void setProcessAccelSigma(float sigma) {
        processAccelSigma_ = sigma;
    }

    // Initial covariance for altitude and vertical velocity.
    void setInitialCovariance(float altitudeVar, float velocityVar) {
        initP00_ = altitudeVar;
        initP11_ = velocityVar;
    }

    // Initialize filter state.
    void init(float altitude0, float velocity0 = 0.0f) {
        x0_ = altitude0;
        x1_ = velocity0;
        p00_ = initP00_;
        p01_ = 0.0f;
        p10_ = 0.0f;
        p11_ = initP11_;
        initialized_ = true;
    }

    bool isInitialized() const {
        return initialized_;
    }

    // Prediction step.
    // az: gravity-compensated vertical acceleration [m/s^2]
    // dt: elapsed time [s]
    void predict(float az, float dt) {
        if (!initialized_ || dt <= 0.0f) return;

        const float dt2 = dt * dt;
        const float dt3 = dt2 * dt;
        const float dt4 = dt3 * dt;

        // State transition.
        x0_ = x0_ + dt * x1_ + 0.5f * dt2 * az;
        x1_ = x1_ + dt * az;

        // Covariance prediction: P = FPF^T + Q
        const float qa = processAccelSigma_ * processAccelSigma_;
        const float fp00 = p00_ + dt * p10_;
        const float fp01 = p01_ + dt * p11_;
        const float fp10 = p10_;
        const float fp11 = p11_;

        p00_ = fp00 + dt * fp01 + 0.25f * dt4 * qa;
        p01_ = fp01 + 0.5f * dt3 * qa;
        p10_ = fp10 + dt * fp11 + 0.5f * dt3 * qa;
        p11_ = fp11 + dt2 * qa;
    }

    // Measurement update for altitude observation.
    // z: measured altitude [m]
    // r: observation noise variance [m^2]
    void update(float z, float r) {
        if (!initialized_ || r <= 0.0f) return;

        const float s = p00_ + r;
        if (s <= 0.0f) return;

        const float k0 = p00_ / s;
        const float k1 = p10_ / s;

        const float innov = z - x0_;
        x0_ += k0 * innov;
        x1_ += k1 * innov;

        // Simplified covariance update for H = [1, 0].
        const float prevP00 = p00_;
        const float prevP01 = p01_;
        p00_ = (1.0f - k0) * prevP00;
        p01_ = (1.0f - k0) * prevP01;
        p10_ = p10_ - k1 * prevP00;
        p11_ = p11_ - k1 * prevP01;
    }

    // Align altitude state to an external reference without resetting velocity.
    void alignAltitude(float altitude, float altitudeVar) {
        if (!initialized_) {
            init(altitude);
            p00_ = altitudeVar;
            return;
        }
        x0_ = altitude;
        if (altitudeVar > 0.0f) {
            p00_ = altitudeVar;
        }
    }

    float altitude() const { return x0_; }
    float velocity() const { return x1_; }
    float altVariance() const { return p00_; }

private:
    float processAccelSigma_ = 0.3f;
    float initP00_ = 100.0f;
    float initP11_ = 10.0f;

    float x0_ = 0.0f;
    float x1_ = 0.0f;

    float p00_ = 100.0f;
    float p01_ = 0.0f;
    float p10_ = 0.0f;
    float p11_ = 10.0f;

    bool initialized_ = false;
};

