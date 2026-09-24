/**
 * @file main.cpp
 * @brief Autonomous Drone Flight Telemetry & Sensor Fusion Simulator
 * @author Ayoub Lahmar (@Redayoub-lang)
 * @version 1.0
 */

#include 
#include 
#include 
#include 
#include 
#include 
#include 

struct RawIMUData {
    double accel_x, accel_y, accel_z;
    double gyro_x, gyro_y, gyro_z;
    double baro_altitude;
};

struct FlightState {
    double pitch;
    double roll;
    double altitude;
};

class SensorNoiseGenerator {
private:
    std::default_random_engine generator;
    std::normal_distribution noise;

public:
    SensorNoiseGenerator(double mean = 0.0, double stddev = 0.05) 
        : noise(mean, stddev) {}

    double getNoise() {
        return noise(generator);
    }
};

class SensorFusionEstimator {
private:
    double alpha;
    double estimated_pitch = 0.0;
    double estimated_roll = 0.0;

public:
    SensorFusionEstimator(double filter_alpha = 0.96) : alpha(filter_alpha) {}

    FlightState update(const RawIMUData& raw, double dt) {
        double accel_pitch = std::atan2(raw.accel_y, std::sqrt(raw.accel_x * raw.accel_x + raw.accel_z * raw.accel_z)) * (180.0 / M_PI);
        double accel_roll  = std::atan2(-raw.accel_x, raw.accel_z) * (180.0 / M_PI);

        double gyro_pitch_rate = raw.gyro_x * (180.0 / M_PI);
        double gyro_roll_rate  = raw.gyro_y * (180.0 / M_PI);

        estimated_pitch = alpha * (estimated_pitch + gyro_pitch_rate * dt) + (1.0 - alpha) * accel_pitch;
        estimated_roll  = alpha * (estimated_roll + gyro_roll_rate * dt) + (1.0 - alpha) * accel_roll;

        return {estimated_pitch, estimated_roll, raw.baro_altitude};
    }
};

class AltitudePIDController {
private:
    double Kp, Ki, Kd;
    double prev_error = 0.0;
    double integral = 0.0;

public:
    AltitudePIDController(double p, double i, double d) : Kp(p), Ki(i), Kd(d) {}

    double computeThrust(double target_alt, double current_alt, double dt) {
        double error = target_alt - current_alt;
        integral += error * dt;
        double derivative = (error - prev_error) / dt;
        prev_error = error;

        return (Kp * error) + (Ki * integral) + (Kd * derivative);
    }
};

int main() {
    std::cout << "=========================================================\n";
    std::cout << "   AUTONOMOUS DRONE FLIGHT TELEMETRY & FUSION SYSTEM     \n";
    std::cout << "   Developer: Ayoub Lahmar (@Redayoub-lang)              \n";
    std::cout << "=========================================================\n\n";

    SensorNoiseGenerator noiseGen(0.0, 0.02);
    SensorFusionEstimator fusion(0.95);
    AltitudePIDController pid(1.5, 0.05, 0.8);

    double dt = 0.1;
    double target_altitude = 10.0;
    double current_altitude = 0.0;

    std::cout << std::fixed << std::setprecision(2);
    std::cout << "Time(s) | Accel_Pitch | Gyro_Pitch | Filtered_Pitch | Alt_Hold(m) | Thrust_Output\n";
    std::cout << "----------------------------------------------------------------------------------\n";

    for (int t = 0; t <= 50; ++t) {
        double time_sec = t * dt;
        double true_pitch = 5.0 * std::sin(time_sec); 
        
        RawIMUData rawData;
        rawData.accel_x = std::sin(true_pitch * M_PI / 180.0) * 9.81 + noiseGen.getNoise();
        rawData.accel_y = noiseGen.getNoise();
        rawData.accel_z = std::cos(true_pitch * M_PI / 180.0) * 9.81 + noiseGen.getNoise();
        rawData.gyro_x  = (0.5 * std::cos(time_sec)) * (M_PI / 180.0) + noiseGen.getNoise();
        rawData.gyro_y  = 0.0;
        rawData.gyro_z  = 0.0;
        
        double thrust = pid.computeThrust(target_altitude, current_altitude, dt);
        current_altitude += (thrust * 0.05) + noiseGen.getNoise();
        rawData.baro_altitude = current_altitude;

        FlightState state = fusion.update(rawData, dt);

        std::cout << std::setw(6) << time_sec << "s | "
                  << std::setw(11) << true_pitch + noiseGen.getNoise() * 10 << " | "
                  << std::setw(10) << state.pitch + noiseGen.getNoise() * 2 << " | "
                  << std::setw(14) << state.pitch << " | "
                  << std::setw(11) << state.altitude << " | "
                  << std::setw(13) << thrust << "\n";

        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    std::cout << "\n[SUCCESS] Telemetry simulation and state estimation completed successfully.\n";
    return 0;
}
