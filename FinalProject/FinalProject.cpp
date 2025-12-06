/**
 * Author: Aidan (Ace) Ehrenhalt
 * Last Date Modified: December 2nd, 2025
 * 
 * Description:
 * Simulation of UAV halftime show using multithreading and OpenGL for visualization.
 * 15 UAVs launched from virtual football field and fly in a formation on around a virtual sphere.
 */

#include <cmath>
#include <mutex>
#include <atomic>
#include <chrono>
#include <thread>
#include <vector>
#include <iomanip>
#include <iostream>

// 3D Vector class for position, velocity, and forces
class Vec3 {
public:
    double x, y, z;
    
    // Constructor with default values
    Vec3(double x = 0, double y = 0, double z = 0) : x(x), y(y), z(z) {} // Set Vec3::[components] with values x, y, z respectively or default value 0
    
    // Override basic arithmetic with vector operations for Vec3 class
    Vec3 operator+(const Vec3 &v) const { return Vec3(x + v.x, y + v.y, z + v.z); }
    Vec3 operator-(const Vec3 &v) const { return Vec3(x - v.x, y - v.y, z - v.z); }
    Vec3 operator*(double s) const { return Vec3(x * s, y * s, z * s); }
    Vec3 operator/(double s) const { return Vec3(x / s, y / s, z / s); }

    Vec3 &operator+=(const Vec3 &v) {
        x += v.x;
        y += v.y;
        z += v.z;
        
        return *this;
    }
    
    double magnitude() const { return sqrt(x*x + y*y + z*z); }
    double distance(const Vec3& v) const { return (*this - v).magnitude(); }

    Vec3 normalized() const { 
        double mag = magnitude();

        if (mag > 0) 
        {
            return *this / mag;
        }
        
        return Vec3(0, 0, 0);
    }
};



// PID Controller class with improved control
class PIDController {
private:
    double kp, ki, kd;
    double integral;
    double prev_error;
    double integral_limit;
    double output_limit;
    
public:
    PIDController(double kp = 1.0, double ki = 0.0, double kd = 0.0, 
                  double integral_limit = 100.0, double output_limit = 50.0) 
        : kp(kp), ki(ki), kd(kd), integral(0), prev_error(0), 
          integral_limit(integral_limit), output_limit(output_limit) {}
    
    /**
     * Calculate PID output based on error and time step
     * 
     * @param error current error value
     * @param dt time step
     * 
     * @return PID controller output
     */
    double calculate(double error, double dt) {
        // Proportional term
        double p_term = kp * error;
        
        // Integral term with anti-windup
        integral += error * dt;
        
        if (integral > integral_limit) {
            integral = integral_limit;
        }
        
        if (integral < -integral_limit) {
            integral = -integral_limit;
        }

        double i_term = ki * integral;
        
        // Derivative term (with filter for noise reduction)
        double derivative = 0;

        if (dt > 0) {
            derivative = (error - prev_error) / dt;
        }
        
        double d_term = kd * derivative;
        
        prev_error = error;
        
        // Calculate total output
        double output = p_term + i_term + d_term;
        
        // Limit output - Also limited in ECE_UAV class, but extra safeguard here
        if (output > output_limit) output = output_limit;
        if (output < -output_limit) output = -output_limit;
        
        return output;
    }
    
    void reset() {
        integral = 0;
        prev_error = 0;
    }
    
    void setGains(double p, double i, double d) {
        kp = p;
        ki = i;
        kd = d;
    }
    
    double getIntegral() const { return integral; }
};

// Forward Declaration of ECE_UAV
class ECE_UAV;
void threadFunction(ECE_UAV *pUav);

// ECE_UAV class with improve physics and control
class ECE_UAV
{
    private:
        Vec3 pos;
        Vec3 velocity;
        Vec3 acceleration;

        double mass;
        double max_force_per_axis;
        double drag_coeff;
        double gravity_comp;

        // UAV Thread Management
        std::thread uav_thread;

        // Control / State Variables
        enum FlightPhase { GROUND_IDLE, CLIMBING, ON_SPHERE };
        FlightPhase current_phase;

        Vec3 target_pos;
        double time_on_sphere;
        double local_sim_time; // Track Local Sim Time
        bool end_sim;

        // PID controllers for x, y, z axes
        PIDController pid_x;
        PIDController pid_y;
        PIDController pid_z;
        
        // Thread Safety
        mutable std::mutex data_mutex;

    public:
    /**
     * Constructor to initialize UAV position
     * 
     * @param initial_pos Initial position of the UAV on the field
     */
    ECE_UAV(Vec3 initial_pos) : pos(initial_pos), velocity(0, 0, 0), acceleration(0, 0, 0),
                                mass(1.0), current_phase(GROUND_IDLE), target_pos(0, 0, 50),
                                time_on_sphere(0.0), end_sim(false)
    {
        // Init PID controllers with starting gains -- Generates desired velocity
        pid_x.setGains(4.0, 0.2, 2.0);   // P, I, D gains for X position
        pid_y.setGains(4.0, 0.2, 2.0);   // P, I, D gains for Y position
        pid_z.setGains(5.0, 0.3, 2.5);   // Higher gains for Z (altitude)
    }

    /**
     * Start ECE_UAV thread
     */
    void startThread()
    {
        uav_thread = std::thread(threadFunction, this);
    }

    /**
     * JOin ECE_UAV thread (for cleanup)
     */
    void joinThread()
    {
        if (uav_thread.joinable())
        {
            uav_thread.join();
        }
    }

    /**
     * Stop ECE_UAV thread
     */
    void stopThread()
    {
        end_sim = true;
    }

    bool isSimEnded() const
    {
        return end_sim;
    }

    /**
     * Update kinematics based on physics forces (used by threadFunction)
     * Uses constant acceleration motion equations
     * 
     * @param dt time step (0.01 sec)
     */
    void updateKinematics(double dt)
    {
        std::lock_guard<std::mutex> lock(data_mutex);

        // Incremenet Local Sim Time
        local_sim_time += dt;

        // Groud Idle - Wait for 5 seconds before climbing
        if (current_phase == GROUND_IDLE)
        {
            if (local_sim_time >= 5.0)
            {
                current_phase = CLIMBING;

                // Reset PID Controllers
                pid_x.reset();
                pid_y.reset();
                pid_z.reset();
            }

            // Stay on Ground (No Movement)
            return;
        }

        // Climbing - Move towards target position
        if (current_phase == CLIMBING)
        {
            // Calculate position errors
            Vec3 pos_error = target_pos - pos;

            // Use PID controllers to generate desired velocities
            double desired_vx = pid_x.calculate(pos_error.x, dt);
            double desired_vy = pid_y.calculate(pos_error.y, dt);
            double desired_vz = pid_z.calculate(pos_error.z, dt);

            // Limit desired velocities to reasonable values (2 m/s)
            double max_climb_velocity = 2.0;
            Vec3 desired_velocity(desired_vx, desired_vy, desired_vz);
            double desired_velocity_mag = desired_velocity.magnitude();

            if (desired_velocity_mag > max_climb_velocity)
            {
                desired_velocity = desired_velocity.normalized() * max_climb_velocity;
            }

            // Calculate Velocity Error
            Vec3 velocity_error = desired_velocity - velocity;

            // Simple proportional control on velocity to get force
            double kp_velocity = 3.0;
            Vec3 control_force = velocity_error * kp_velocity;

            // Add gravity (10 N downwards per 1 kg of mass)
            double gravity_force = 10.0; // 10 N in -Z direction
            control_force.z += gravity_force;

            // Apply force limits (20 N total)
            double force_limit = control_force.magnitude();

            if (force_limit > 20.0)
            {
                control_force = control_force.normalized() * 20.0;
            }

            // Calculate Acceleration (F = ma)
            acceleration.z -= 10.0; // Gravity

            // Calculate Velocity: v = v0 + a * dt
            velocity = velocity + acceleration * dt;

            // Calculate Position: x = x0 + v * dt + 0.5 * a * dt^2
            pos = pos + velocity * dt + acceleration * (0.5 * dt * dt);

            // Groud Limits = Can't go below z = 0
            if (pos.z < 0)
            {
                pos.z = 0;
                
                if (velocity.z < 0)
                {
                    velocity.z = 0;
                }
            }

            // Check if reached target position (within 0.5 m) before switching to ON_SPHERE
            double distance_to_target = pos.distance(target_pos);

            if (distance_to_target <= 10.0) // Check within 10m of sphere
            {
                current_phase = ON_SPHERE;
                time_on_sphere = 0.0; // Reset UAV time on sphere

                // Add UAV movement on sphere surface
            }
        }

        // On Sphere - UAV on virtual sphere surface
        if (current_phase == ON_SPHERE)
        {
            // Update time on sphere
            time_on_sphere += dt;

            // Implement sphere surface flight

            // Simple holding pattern near target pos (temp)
            Vec3 pos_error = target_pos - pos;
            double desired_vx = pid_x.calculate(pos_error.x, dt);
            double desired_vy = pid_y.calculate(pos_error.y, dt);
            double desired_vz = pid_z.calculate(pos_error.z, dt);
            
            Vec3 desired_velocity(desired_vx, desired_vy, desired_vz);
            Vec3 velocity_error = desired_velocity - velocity;

            double kp_velocity = 3.0;
            Vec3 control_force = velocity_error * kp_velocity;
            control_force.z += 10.0; // Gravity Compensation

            double force_mag = control_force.magnitude();

            if (force_mag > 20.0)
            {
                control_force = control_force.normalized() * 20.0;
            }

            acceleration = control_force / mass;
            acceleration.z -= 10.0; // Gravity
            
            velocity = velocity + acceleration * dt;
            pos = pos + velocity * dt + acceleration * (0.5 * dt * dt);
        }
    }

    /**
     * Get UAV's current position
     * 
     * @return current position vector
     */
    Vec3 getPos() const
    {
        std::lock_guard<std::mutex> lock(data_mutex);

        return pos;
    }

    /**
     * Get UAV's current velocity
     * 
     * @return current velocity vector
     */
    Vec3 getVelocity() const
    {
        std::lock_guard<std::mutex> lock(data_mutex);

        return velocity;
    }

    /**
     * Set UAV's velocity - Collision handling
     */
    void setVelocity(const Vec3 &newVel)
    {
        std::lock_guard<std::mutex> lock(data_mutex);

        velocity = newVel;
    }

    /**
     * Get UAV's time spent on virtual sphere surface
     * 
     * @return time on sphere in seconds
     */
    double getTimeOnSphere() const
    {
        std::lock_guard<std::mutex> lock(data_mutex);

        return time_on_sphere;
    }

    /**
     * Get UAV's current flight phase
     * 
     * @return current flight phase enum
     */
    FlightPhase getFlightPhase() const
    {
        std::lock_guard<std::mutex> lock(data_mutex);

        return current_phase;
    }
};



/**
 * Thread function for UAV simulation
 */
void threadFunction(ECE_UAV *pUAV)
{
    while(!pUAV->isSimEnded())
    {
        // Update kinematics with fixed time step (0.01 sec)
        pUAV->updateKinematics(0.01);

        // Sleep to maintain real-time simulation (10 ms)
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
}



// Global Variables
std::atomic<double> globalSimTime(0.0); // Global Simulation Time (seconds)
std::vector<ECE_UAV *> uavFleet;        // Vector of pointers to individual UAVs

/**
 * Initialize UAV fleet at starting position on virtual field
 */
void initializeUAVFleet()
{
    // Y-Position - Spread UAV fleet on a grid on the field
    std::vector<double> x_pos = {-40, -20, 0, 20, 40}; // Relative to center field - 5 Columns

    // X-Position - Spread UAV fleet on a grid on the field
    std::vector<double> y_pos = {-10, 0, 10}; // 3 Rows

    // Create Fleet of 15 UAVs
    for (double x : x_pos)
    {
        for (double y : y_pos)
        {
            Vec3 inital_pos(x, y, 0); // Start on ground (z=0)
            uavFleet.push_back(new ECE_UAV(inital_pos));
        }
    }

    std::cout << "Initialized UAV Fleet with " << uavFleet.size() << " UAVs.\n" << std::flush;
}

/**
 * Main function to run UAV simulation
 * 
 * @param argc argument count
 * @param argv argument vector with argument values
 */
int main(int argc, char** argv)
{
    std::cout << "Starting UAV Halftime Show Simulation...\n";

    // Initialize UAV Fleet at starting positions
    initializeUAVFleet();

    // Start UAV Threads
    std::cout << "Starting UAV Threads...\n";
    for (ECE_UAV *uav : uavFleet)
    {
        uav->startThread();
    }


    // TODO: Initialize OpenGL Visualization Here
    
    // Temp: Simple Simulation Loop
    std::cout << "Running Simulation Loop for 5 seconds for testing...\n";
    
    double dt = 0.03; // 30 ms per iteration
    double maxTime = 5.0; // Run for 5 seconds for testing

    while (globalSimTime.load() < maxTime)
    {
        // Update global simulation time
        globalSimTime.store(globalSimTime.load() + dt);

        // TODO: Check Collisions
        // TODO: Rendering
        // TODO: Checking Completion Conditions

        // Sleep to maintain time step (30 ms)
        std::this_thread::sleep_for(std::chrono::milliseconds(30));

        // Print status every second
        static double last_print_time = 0.0;
        if (globalSimTime.load() - last_print_time >= 1.0)
        {
            last_print_time = globalSimTime.load();

            // Print position of first UAV for testing
            std::cout << "Simulation Time: " << globalSimTime.load() << " sec - UAV[0] position: ("
                      << uavFleet[0]->getPos().x << ", "
                      << uavFleet[0]->getPos().y << ", "
                      << uavFleet[0]->getPos().z << ")\n" << std::flush;
        }
    }

    // Stop UAV Threads
    std::cout << "Stopping UAV Threads...\n";
    for (auto *uav : uavFleet)
    {
        uav->stopThread();
        uav->joinThread();

        delete uav; // Clean up UAV objects
    }

    std::cout << "UAV Halftime Show Simulation Ended.\n";

    return 0;
}