package Drone;

/**
 * The PID class implements a Proportional-Integral-Derivative (PID) controller.
 * This class allows you to tune the P, I, and D parameters and calculate the control output based on the error and the time elapsed.
 */

public class PID {

    private double P, I, D, max_i, integral, last_error;

    // Flag to handle first update
    private boolean first_run;

    /**
     * Constructor to initialize the PID controller with the specified gains.
     *
     * @param p     The proportional gain (P), controlling the reaction to the current error.
     * @param i     The integral gain (I), controlling the accumulation of past errors.
     * @param d     The derivative gain (D), controlling the reaction to the rate of change of error.
     */
    public PID(double p, double i, double d) {
        this.P = p;
        this.I = i;
        this.D = d;
        integral = 0;
        first_run = true;
    }

    /**
     * Setters for PID params.
     */
    public void Set_P(double p) {
        this.P = p;
    }

    public void Set_I(double i) {
        this.I = i;
    }

    public void Set_D(double d) {
        this.D = d;
    }

    public void Set_Integral(double integral) {
        this.integral = integral;
    }

    public double Get_Integral() {
        return this.integral;
    }

    /**
     * The main update function of the PID controller. This function calculates the control output
     * based on the current error and the time step (dt). It also updates the integral and derivative terms.
     *
     * @param error The current error (difference between the setpoint and the actual value).
     * @param dt    The time difference since the last update (in seconds).
     * @return The control output to be applied to the system.
     */
    public double update(double error, double dt) {

        // Handle the first update to initialize last_error
        if (first_run) {
            last_error = error;
            first_run = false;
        }
        integral += I * error * dt;
        double diff = (error - last_error) / dt;
        double const_integral = integral;
        double control_out = P * error + D * diff + const_integral;
        last_error = error;
        return control_out;
    }
}