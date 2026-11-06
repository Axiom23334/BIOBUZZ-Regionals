package org.firstinspires.ftc.teamcode.maths;

/**
 * 1-Dimensional Kalman Filter for smoothing noisy sensor measurements
 * such as camera pixel distances and heights while eliminating stationary jitter.
 */
public class KalmanFilter {
    private double q; // Process noise covariance (higher = responds faster to true movement)
    private double r; // Measurement noise covariance (higher = filters out more sensor noise)
    private double x; // Estimated state
    private double p; // Estimation error covariance
    private double k; // Kalman gain
    private boolean initialized = false;

    /**
     * @param processNoise Q: estimate of process variance (e.g. 0.05 to 0.5)
     * @param measurementNoise R: estimate of sensor measurement variance (e.g. 4.0 to 15.0)
     */
    public KalmanFilter(double processNoise, double measurementNoise) {
        this.q = processNoise;
        this.r = measurementNoise;
        this.p = 1.0;
        this.x = 0.0;
    }

    /**
     * Updates the filter with a new raw measurement and returns the filtered value.
     */
    public double update(double measurement) {
        if (!initialized) {
            this.x = measurement;
            this.p = 1.0;
            this.initialized = true;
            return measurement;
        }

        // 1. Time update (Prediction): p = p + q
        p = p + q;

        // 2. Measurement update (Correction)
        // Kalman Gain: K = p / (p + r)
        k = p / (p + r);

        // State update: x = x + K * (measurement - x)
        x = x + k * (measurement - x);

        // Covariance update: p = (1 - K) * p
        p = (1.0 - k) * p;

        return x;
    }

    public void reset() {
        this.initialized = false;
        this.p = 1.0;
    }

    public void setParameters(double processNoise, double measurementNoise) {
        this.q = processNoise;
        this.r = measurementNoise;
    }

    public double getState() {
        return x;
    }

    public boolean isInitialized() {
        return initialized;
    }
}
