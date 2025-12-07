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
#include <ctime>
#include <atomic>
#include <chrono>
#include <thread>
#include <vector>
#include <cstdlib>
#include <iomanip>
#include <iostream>

// Define M_PI (for use if not defined)
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

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


// Forward Declaration of Global Variables
extern std::atomic<double> globalSimTime;

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

        // Sphere Target Position
        Vec3 sphere_center;
        double sphere_radius;
        Vec3 sphere_target;

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
                                sphere_center(0, 0, 50), sphere_radius(10.0), sphere_target(0, 0, 0),
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
     * Join ECE_UAV thread (for cleanup)
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
     * Generate randomized sphere target position for ON_SPHERE phase
     * 
     * @return random point on virtual sphere's surface
     */
    Vec3 generateRandomSphereTarget()
    {
        Vec3 random_point;

        // Generate random spherical coordinates - Random points until they fall on the sphere, once they fall on the sphere's surface translate to actual sphere
        do
        {
            random_point.x = ((double)rand() / RAND_MAX) * 2.0 - 1.0; // [-1, 1]
            random_point.y = ((double)rand() / RAND_MAX) * 2.0 - 1.0; // [-1, 1]
            random_point.z = ((double)rand() / RAND_MAX) * 2.0 - 1.0; // [-1, 1]
        } while (random_point.magnitude() > 1.0 || random_point.magnitude() < 0.01);

        // Normalizing (Scaling and Translating) to virtual sphere surface
        random_point = random_point.normalized();
        random_point = sphere_center + random_point * sphere_radius;

        return random_point;
    }

    /**
     * Projecting UAV position onto the virtual sphere
     * 
     * @param current_pos current position of the UAV
     * 
     * @return projected position on the virtual sphere
     */
    Vec3 projectOntoSphere(const Vec3& current_pos)
    {
        Vec3 direction = current_pos - sphere_center;
        double distance = direction.magnitude();

        if (distance > 0)
        {
            return sphere_center + direction.normalized() * sphere_radius;
        }

        return current_pos;
    }

    /**
     * Calculate tangential velocity - Remove velocity component in direction towards (or away from) sphere center
     * 
     * @param velocity current velocity vector
     * @param pos current position
     * 
     * @return tangential velocity component
     */
    Vec3 getTangentialVelocity(const Vec3 &velocity, const Vec3 &pos)
    {
        Vec3 radial = (pos - sphere_center).normalized();
        double radial_component = velocity.x * radial.x + velocity.y * radial.y + velocity.z * radial.z;

        Vec3 radial_velocity = radial * radial_component;
        
        return velocity - radial_velocity; // Remove radial velocity component
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

        // Current Sim Time
        double current_time = globalSimTime.load();

        // Groud Idle - Wait for 5 seconds before climbing
        if (current_phase == GROUND_IDLE)
        {
            // Wait for 5 seconds before climbing
            if (current_time < 5.0)
            {
                // Stay on Ground (GROUD_IDLE)
                velocity = Vec3(0, 0, 0);
                acceleration = Vec3(0, 0, 0);
                
                return;
            }
            else
            {
                current_phase = CLIMBING;

                // Reset PID Controllers
                pid_x.reset();
                pid_y.reset();
                pid_z.reset();
            }
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
            acceleration = control_force / mass; // Acceleration Calculation
            acceleration.z -= 10.0; // Account for Gravity

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
                sphere_target = generateRandomSphereTarget();

                // Reset PID Controllers
                pid_x.reset();
                pid_y.reset();
                pid_z.reset();
            }

            return;
        }

        // On Sphere - UAV on virtual sphere surface
        if (current_phase == ON_SPHERE)
        {
            // Update time on sphere
            time_on_sphere += dt;

            // Calculate distance to sphere target
            double distance_to_sphere_target = pos.distance(sphere_target);

            // Generate new random target when close to arriving at current target
            if (distance_to_sphere_target < 2.0)
            {
                sphere_target = generateRandomSphereTarget();
            }

            // Calculate position error relative to sphere target
            Vec3 pos_error = sphere_target - pos;

            // Use PID Controllers to generate desired velocity
            double desired_vx = pid_x.calculate(pos_error.x, dt);
            double desired_vy = pid_y.calculate(pos_error.y, dt);
            double desired_vz = pid_z.calculate(pos_error.z, dt);

            // Limit velocity to speed between 2 - 10 m/s - Target ~6 m/s for stable flight path
            double target_speed = 6.0;
            Vec3 desired_velocity(desired_vx, desired_vy, desired_vz);
            double desired_speed = desired_velocity.magnitude();

            if (desired_speed > 0)
            {
                desired_velocity = desired_velocity.normalized() * target_speed; // Scale to target velocity
            }

            // Calculate velocity error
            Vec3 velocity_error = desired_velocity - velocity;

            // Control force based on velocity_error
            double kp_velocity = 3.0;
            Vec3 control_force = velocity_error * kp_velocity;

            // Gravity Compensation
            double gravity_force = 10.0;
            control_force.z += gravity_force;

            // Constrain radial force to keep UAV on virtual sphere surface - Calculate radial direction from sphere center
            Vec3 radial = (pos - sphere_center).normalized();
            double distance_from_center = pos.distance(sphere_center);
            double radial_error = sphere_radius - distance_from_center;

            // Apply strong radial force to keep UAV on sphere surface
            double kp_radial = 50.0; // Radial control gain
            Vec3 radial_force = sphere_radius - distance_from_center;

            control_force = control_force + radial_force;

            // Calculation accounting for gravity
            Vec3 gravity(0, 0, -gravity_force);
            Vec3 total_force = control_force + gravity;

            // Apply 20 N force magnitude limit
            double force_mag = total_force.magnitude();

            if (force_mag > 20.0)
            {
                total_force = total_force.normalized() * 20.0;
            }

            // Calculate acceleration: F = ma
            acceleration = total_force / mass;

            // Update velocity
            velocity = velocity + acceleration * dt;

            // Constrain velocity to tangential component - Helps keep UAV moving along virtual sphere surface
            velocity = getTangentialVelocity(velocity, pos);

            // Limit speed to range 2-10 m/s
            double current_speed = velocity.magnitude();

            if (current_speed > 10.0)
            {
                velocity = velocity.normalized() * 10.0;
            }
            else if (current_speed < 2.0 && current_speed > 0)
            {
                velocity = velocity.normalized() * 2.0;
            }

            // Update position
            pos = pos + velocity * dt + acceleration * (0.5 * dt * dt);

            // Project position onto virtual sphere surface
            pos = projectOntoSphere(pos);

            return;
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
    void setVelocity(const Vec3 &newVelocity)
    {
        std::lock_guard<std::mutex> lock(data_mutex);

        velocity = newVelocity;
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
 * Collision handling for individual UAVs
 * If bounding boxes come within 1 cm (0.01 m), handle collision by swapping velocities
 */
void collisionHandling()
{
    const double collision_distance = 0.01; // 0.01 m (1 cm)
    const double bounding_box_size = 0.20; // 0.20 m (20 cm) bounding box around UAV

    // Check for collisions between UAVs
    for (size_t i = 0; i < uavFleet.size(); i++)
    {
        for (size_t j = i + 1; j < uavFleet.size(); j++)
        {
            Vec3 pos_i = uavFleet[i]->getPos();
            Vec3 pos_j = uavFleet[j]->getPos();

            double distance = pos_i.distance(pos_j); // Calculate distance between UAVs

            // Check if bounding boxes are colliding
            if (distance < (bounding_box_size + collision_distance))
            {
                // Swap velocities (like the UAVs are bouncing off and going opposite directions)
                Vec3 velocity_i = uavFleet[i]->getVelocity();
                Vec3 velocity_j = uavFleet[j]->getVelocity();

                uavFleet[i]->setVelocity(velocity_j);
                uavFleet[j]->setVelocity(velocity_i);
            }
        }
    }
}



/**
 * Check if simulation meets completion criteria
 * All UAVs must be within 10m of the virtual sphere center (0, 0, 50) AND have spent 60 seconds flying on the virtual sphere
 * 
 * @return true if simulation should end, false otherwise
 */
bool checkCompletionConditions()
{
    Vec3 target_point(0, 0, 50);
    const double required_sphere_time = 60.0; // 60 sec on sphere
    const double proximity_threshold = 10.0; // 10 m proximity

    bool all_conditions_met = true;

    for (ECE_UAV *uav : uavFleet)
    {
        Vec3 pos = uav->getPos();
        double distance = pos.distance(target_point);
        double time_on_sphere = uav->getTimeOnSphere();

        if (distance > proximity_threshold)
        {
            all_conditions_met = false;

            break;
        }

        // Check if UAV has flown for the 60 sec on sphere
        if (time_on_sphere < required_sphere_time)
        {
            all_conditions_met = false;

            break;
        }
    }

    return all_conditions_met;
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

    // Testing Phase 2 - Drone Movement Control + Phase Control
    for (int i = 0; i < uavFleet.size(); i++)
    {
        Vec3 pos = uavFleet[i]->getPos();
        std::cout << "UAV[" << i << "] starts at: (" << pos.x << ", " << pos.y << ", " << pos.z << ")\n";
    }

    // Start UAV Threads
    std::cout << "Starting UAV Threads...\n";
    for (ECE_UAV *uav : uavFleet)
    {
        uav->startThread();
    }

    // TODO: Initialize OpenGL Visualization Here
    
    // Simulation Loop
    std::cout << "Running Simulation... \n";

    double dt = 0.03; // 30 ms per iteration

    while (true)
    {
        // Update global simulation time
        globalSimTime.store(globalSimTime.load() + dt);

        // Handle UAV collisions
        collisionHandling();

        // Check if simulation meets completion criteria
        if (checkCompletionConditions())
        {
            std::cout << "\n=== SIMULATION COMPLETE ===\n";
            std::cout << "All UAVs have reached target on virtual sphere and remained on sphere surface for 60 seconds.\n";

            break;
        }

        // TODO: Rendering with OpenGL

        // Sleep to maintain correct time stop interval (30 ms)
        std::this_thread::sleep_for(std::chrono::milliseconds(30));

        // Print status every 2 second interval
        static double last_print_time = 0.0;

        if (globalSimTime.load() - last_print_time >= 2.0)
        {
            last_print_time = globalSimTime.load();

            // Count UAVs during each phase
            int ground_idle_count = 0;
            int climbing_count = 0;
            int on_sphere_count = 0;

            double min_sphere_time = 999999;

            for (ECE_UAV *uav : uavFleet)
            {
                auto phase = uav->getFlightPhase();

                if (phase == 0)
                {
                    ground_idle_count++;
                }
                else if (phase == 1)
                {
                    climbing_count++;
                }
                else if (phase == 2)
                {
                    on_sphere_count++;
                    double sphere_time = uav->getTimeOnSphere();
                    if (sphere_time < min_sphere_time)
                    {
                        min_sphere_time = sphere_time;
                    }
                }
            }

            // Print UAV Phase Counts
            std::cout << "\n---Sim Time: " << std::fixed << std::setprecision(1) << globalSimTime.load() << "s ---\n";
            std::cout << " Ground: " << ground_idle_count << " UAVs"
                      << " | " << " Climbing: " << climbing_count << " UAVs"
                      << " | " << " On Sphere: " << on_sphere_count << " UAVs\n";

            if (on_sphere_count > 0)
            {
                std::cout << " Min sphere time: " << std::setprecision(2) << min_sphere_time << " s / 60.0s \n";
            }

            // DEBUGGING: Print UAV[0] details
            Vec3 pos = uavFleet[0]->getPos();
            Vec3 velocity = uavFleet[0]->getVelocity();
                
            std::cout << " UAV[0] Pos: (" << std::setprecision(2) << pos.x << ", " << pos.y << ", " << pos.z << ")"
                      << " | Velocity Magnitude: " << velocity.magnitude() << "m/s\n" << std::flush;
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