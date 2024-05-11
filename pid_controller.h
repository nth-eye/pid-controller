#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

/**
 * @brief Simple Proportional-Integral-Derivative (PID) controller. 
 * Computes the control output based on the input error and controller 
 * gains.
 * 
 * @tparam T The data type for controller parameters and in/out values.
 */
template<class T = double>
struct pid_controller {

    /**
     * @brief Underlying numeric data type.
     * 
     */
    using data_t = T;

    /**
     * @brief Configure the controller proportional, integral, and derivative gains.
     * 
     * @param p The proportional gain.
     * @param i The integral gain.
     * @param d The derivative gain.
     */
    void set_gains(data_t p, data_t i, data_t d)
    {
        kp = p;
        ki = i;
        kd = d;
    }

    /**
     * @brief Configure setpoint for the controller.
     * 
     * @param trg The target value to be achieved by the controller.
     */
    void set_target(data_t trg)
    {
        target = trg;
    }

    /**
     * @brief Set the saturation limits for the control output. The
     * clamping of output value and integral anti-windup mechanism
     * are applied automatically.
     * 
     * @param min The minimum output value allowed.
     * @param max The maximum output value allowed.
     */
    void set_limits(data_t min, data_t max)
    {
        output_min = min;
        output_max = max;
    }

    /**
     * @brief Reset the dynamics of controller.
     * 
     */
    void reset_state()
    {
        integral = 0;
        prev_error = 0;
    }

    /**
     * @brief Get the control output based on the input feedback and time step.
     * 
     * @param input The current input value.
     * @param dt The time step in seconds.
     * @return The computed control output.
     */
    data_t compute(data_t input, data_t dt)
    {
        data_t error = target - input;
        data_t derivative = (error - prev_error) / dt;
        prev_error = error;
        integral += error * dt;
        data_t output = kp * error + ki * integral + kd * derivative;

        if (output > output_max) {
            output = output_max;
            integral = ki ? (output - kp * error - kd * derivative) / ki : 0;
        } else if (output < output_min) {
            output = output_min;
            integral = ki ? (output - kp * error - kd * derivative) / ki : 0;
        }
        return output;
    } 
private:
    data_t kp = 0;
    data_t ki = 0;
    data_t kd = 0;
    data_t target = 0;
    data_t integral = 0;
    data_t prev_error = 0;
    data_t output_min = std::numeric_limits<data_t>::lowest();
    data_t output_max = std::numeric_limits<data_t>::max();
};

#endif
