/* This file builds an executable that can receive IMU data from a phone and
control the pose (roll and pitch) of a rectangular slab in OpenSim. The phone needs to be Android,
have an accelerometer and a gyroscope, and have the app 'Wireless IMU' installed. To run the
executable, make sure to set the IP address of the machine that is running the executable into
app. Just run the executable, wait for the visualizer to show up, and then in the app(on the phone), 
turn on streaming.
This file only compiles on UNIX machines. It uses UNIX headers for socket work.
Use 'Release' mode for compilation. It generates faster code and results in slightly quicker
tracking of pose.*/

#include <sys/socket.h>
#include <netinet/in.h>

#include <memory>
#include <tuple>
#include <iostream>
#include <stdexcept>
#include <string>
#include <array>
#include <regex>
#include <cmath>

#include <OpenSim/OpenSim.h>

class Kalman {
public:
    Kalman();

    // The angle should be in degrees and the rate should be in degrees per second and the delta time in seconds
    float getAngle(float newAngle, float newRate, float dt);

    void setAngle(float angle); // Used to set angle, this should be set as the starting angle
    float getRate(); // Return the unbiased rate

    /* These are used to tune the Kalman filter */
    void setQangle(float Q_angle);
    void setQbias(float Q_bias);
    void setRmeasure(float R_measure);

    float getQangle();
    float getQbias();
    float getRmeasure();

private:
    /* Kalman filter variables */
    float Q_angle; // Process noise variance for the accelerometer
    float Q_bias; // Process noise variance for the gyro bias
    float R_measure; // Measurement noise variance - this is actually the variance of the measurement noise

    float angle; // The angle calculated by the Kalman filter - part of the 2x1 state vector
    float bias; // The gyro bias calculated by the Kalman filter - part of the 2x1 state vector
    float rate; // Unbiased rate calculated from the rate and the calculated bias - you have to call getAngle to update the rate

    float P[2][2]; // Error covariance matrix - This is a 2x2 matrix
};

Kalman::Kalman() {
    /* We will set the variables like so, these can also be tuned by the user */
    Q_angle = 0.001f;
    Q_bias = 0.003f;
    R_measure = 0.03f;

    angle = 0.0f; // Reset the angle
    bias = 0.0f; // Reset bias

    P[0][0] = 0.0f; // Since we assume that the bias is 0 and we know the starting angle (use setAngle), the error covariance matrix is set like so - see: http://en.wikipedia.org/wiki/Kalman_filter#Example_application.2C_technical
    P[0][1] = 0.0f;
    P[1][0] = 0.0f;
    P[1][1] = 0.0f;
};

// The angle should be in degrees and the rate should be in degrees per second and the delta time in seconds
float Kalman::getAngle(float newAngle, float newRate, float dt) {
    // KasBot V2  -  Kalman filter module - http://www.x-firm.com/?page_id=145
    // Modified by Kristian Lauszus
    // See my blog post for more information: http://blog.tkjelectronics.dk/2012/09/a-practical-approach-to-kalman-filter-and-how-to-implement-it

    // Discrete Kalman filter time update equations - Time Update ("Predict")
    // Update xhat - Project the state ahead
    /* Step 1 */
    rate = newRate - bias;
    angle += dt * rate;

    // Update estimation error covariance - Project the error covariance ahead
    /* Step 2 */
    P[0][0] += dt * (dt*P[1][1] - P[0][1] - P[1][0] + Q_angle);
    P[0][1] -= dt * P[1][1];
    P[1][0] -= dt * P[1][1];
    P[1][1] += Q_bias * dt;

    // Discrete Kalman filter measurement update equations - Measurement Update ("Correct")
    // Calculate Kalman gain - Compute the Kalman gain
    /* Step 4 */
    float S = P[0][0] + R_measure; // Estimate error
    /* Step 5 */
    float K[2]; // Kalman gain - This is a 2x1 vector
    K[0] = P[0][0] / S;
    K[1] = P[1][0] / S;

    // Calculate angle and bias - Update estimate with measurement zk (newAngle)
    /* Step 3 */
    float y = newAngle - angle; // Angle difference
    /* Step 6 */
    angle += K[0] * y;
    bias += K[1] * y;

    // Calculate estimation error covariance - Update the error covariance
    /* Step 7 */
    float P00_temp = P[0][0];
    float P01_temp = P[0][1];

    P[0][0] -= K[0] * P00_temp;
    P[0][1] -= K[0] * P01_temp;
    P[1][0] -= K[1] * P00_temp;
    P[1][1] -= K[1] * P01_temp;

    return angle;
};

void Kalman::setAngle(float angle) { this->angle = angle; }; // Used to set angle, this should be set as the starting angle
float Kalman::getRate() { return this->rate; }; // Return the unbiased rate

/* These are used to tune the Kalman filter */
void Kalman::setQangle(float Q_angle) { this->Q_angle = Q_angle; };
void Kalman::setQbias(float Q_bias) { this->Q_bias = Q_bias; };
void Kalman::setRmeasure(float R_measure) { this->R_measure = R_measure; };

float Kalman::getQangle() { return this->Q_angle; };
float Kalman::getQbias() { return this->Q_bias; };
float Kalman::getRmeasure() { return this->R_measure; };

std::tuple<double, 
           std::array<double, 3>,
           std::array<double, 3>> 
parseImuData(const char* str, const size_t len) {
    std::string data{str, len};

    std::regex regex_ts{"^[0-9.]+"};
    std::regex regex_gravity{", 3, +(-?[0-9.]+), +(-?[0-9.]+), +(-?[0-9.]+)"};
    std::regex   regex_omega{", 4, +(-?[0-9.]+), +(-?[0-9.]+), +(-?[0-9.]+)"};
    // Timestamp.
    std::smatch result_ts{};
    if(!std::regex_search(data, result_ts, regex_ts))
        throw std::runtime_error{"No timestamp in data stream."};
    double timestamp{std::stod(result_ts[0])};
    // Acceleration.
    std::array<double, 3> gravity{};
    std::smatch result_gravity{};
    if(std::regex_search(data, result_gravity, regex_gravity)) {
        gravity[0] = std::stod(result_gravity[1]);
        gravity[1] = std::stod(result_gravity[2]);
        gravity[2] = std::stod(result_gravity[3]);
    }
    // Angular veclocity.
    std::array<double, 3> omega{};
    std::smatch result_omega{};
    if(std::regex_search(data, result_omega, regex_omega)) {
        omega[0] = std::stod(result_omega[1]);
        omega[1] = std::stod(result_omega[2]);
        omega[2] = std::stod(result_omega[3]);
    }    

    return std::make_tuple(timestamp, gravity, omega);
}

double radToDeg(double rad) {
    constexpr double RADTODEG{57.2957795131};
    return rad * RADTODEG;
}

double degToRad(double deg) {
    constexpr double DEGTORAD{0.01745329};
    return deg * DEGTORAD;
}

std::pair<double, double> 
computeRollPitch(const std::array<double, 3>& gravity) {
    const double x = gravity[0];
    const double y = gravity[1];
    const double z = gravity[2];

    const double z_sq = z * z;
    double roll  = std::atan(x / std::sqrt(y*y + z_sq));
    double pitch = std::atan(y / std::sqrt(x*x + z_sq));
    return {roll, pitch};
}

unsigned openUdpSocket() {
    constexpr short PORT{5555};

    auto sock = socket(AF_INET, SOCK_DGRAM, 0);
    if(sock < 0)
        throw std::runtime_error{"Could not create Socket."};

    sockaddr_in saddr{};
    std::memset(&saddr, 0, sizeof(saddr));
    saddr.sin_family      = AF_INET;
    saddr.sin_port        = htons(PORT);
    saddr.sin_addr.s_addr = htonl(INADDR_ANY);

    if(bind(sock, (struct sockaddr*)&saddr, sizeof(saddr)) < 0)
        throw std::runtime_error{"Could not bind Socket."};

    return sock;
}

int main() {
    // --------------------- Opensim model.
    OpenSim::Model model{};
    model.setUseVisualizer(true);
    
    auto slab = new OpenSim::Body{"slab", 
                                  1, 
                                  SimTK::Vec3{0}, 
                                  SimTK::Inertia{0}};

    auto balljoint = new OpenSim::BallJoint{"balljoint",
                                            model.getGround(),
                                            SimTK::Vec3{0, 1, 0},
                                            SimTK::Vec3{0},
                                            *slab,
                                            SimTK::Vec3{0},
                                            SimTK::Vec3{0}};

    OpenSim::Brick brick{};
    brick.setFrame(*slab);
    brick.set_half_lengths(SimTK::Vec3{0.5, 0.05, 0.25});
    slab->addComponent(brick.clone());

    model.addBody(slab);
    model.addJoint(balljoint);

    auto& state = model.initSystem();

    model.updMatterSubsystem().setShowDefaultGeometry(false);
    SimTK::Visualizer& viz = model.updVisualizer().updSimbodyVisualizer();
    viz.setBackgroundType(viz.GroundAndSky);
    viz.setShowSimTime(true);
    viz.drawFrameNow(state);

    // -------------------------- UDP Socket.
    constexpr unsigned BUFFSIZE{8192};

    auto sock = openUdpSocket();

    // ------------------------- Kalman filter.
    Kalman kalman_roll{};
    Kalman kalman_pitch{};
    double oldtimestamp{};
    bool firstrow{true};

    // ------------------------ Streaming.
    while(true) {
        char buffer[BUFFSIZE];
        auto bytes = recvfrom(sock, buffer, BUFFSIZE, 0, 0, 0);

        if(bytes > 0) {
            auto data = parseImuData(buffer, BUFFSIZE);

            auto& timestamp = std::get<0>(data);
            auto& gravity   = std::get<1>(data);
            auto& omega     = std::get<2>(data);

            // If omega is (0, 0, 0), skip over because there was no data.
            // All three components are never equal except when they are 0.
            if(omega[0] == omega[1] && omega[1] == omega[2])
                continue;

            // Compute change in time and record the timestamp.
            auto deltat = timestamp - oldtimestamp;
            oldtimestamp = timestamp;
            if(firstrow) {
                firstrow = false;
                continue;
            }

            auto tilt = computeRollPitch(gravity);
            auto roll  = radToDeg(tilt.first);
            auto pitch = radToDeg(tilt.second);

            omega[0] = radToDeg(omega[0]);
            omega[1] = radToDeg(omega[1]);
            omega[2] = radToDeg(omega[2]);

            // Angular velocity about axis y is roll.
            // Angular velocity about axis x is pitch.
            auto roll_hat  =  kalman_roll.getAngle( roll, omega[1], deltat);
            auto pitch_hat = kalman_pitch.getAngle(pitch, omega[0], deltat);

            // Multiplying -1 to roll just for display. This way visualizaiton moves
            // like the physical phone.
            model.getCoordinateSet()[0].setValue(state, -1 * degToRad( roll_hat));
            model.getCoordinateSet()[2].setValue(state, degToRad(pitch_hat));
            
            viz.drawFrameNow(state);
            
            //std::cout << buffer << std::endl;
        } else
            std::cout << "skipping....." << std::endl;
    }

    return 0;
}
