package Drone;

import Drone.Moon;

/**
 * This class represents the basic flight controller of the Bereshit space craft.
 *
 * @author ben-moshe
 */
public class Bereshit_101 {
    public static final double WEIGHT_EMP = 165; // kg
    public static final double WEIGHT_FULE = 420; // kg
    public static final double WEIGHT_FULL = WEIGHT_EMP + WEIGHT_FULE; // kg
    // https://davidson.weizmann.ac.il/online/askexpert/%D7%90%D7%99%D7%9A-%D7%9E%D7%98%D7%99%D7%A1%D7%99%D7%9D-%D7%97%D7%9C%D7%9C%D7%99%D7%AA-%D7%9C%D7%99%D7%A8%D7%97
    public static final double MAIN_ENG_F = 430; // N
    public static final double SECOND_ENG_F = 25; // N
    public static final double MAIN_BURN = 0.15; //liter per sec, 12 liter per m'
    public static final double SECOND_BURN = 0.009; //liter per sec 0.6 liter per m'
    public static final double ALL_BURN = MAIN_BURN + 8 * SECOND_BURN;

    public static double accMax(double weight) {
        return acc(weight, true, 8);
    }

    public static double acc(double weight, boolean main, int seconds) {
        double t = 0;
        if (main) {
            t += MAIN_ENG_F;
        }
        t += seconds * SECOND_ENG_F;
        double ans = t / weight;
        return ans;
    }

    // 14095, 955.5, 24.8, 2.0
    public static void main(String[] args) {
        System.out.println("Simulating Bereshit's Landing:");
        // starting point:
        double vs = 24.8;
        double hs = 932;
        double dist = 181 * 1000;
        double ang = 58.3; // zero is vertical (as in landing)
        double alt = 13748; // 2:25:40 (as in the simulation) // https://www.youtube.com/watch?v=JJ0VfRL9AMs
        double time = 0;
        double dt = 1; // sec
        double acc = 0; // Acceleration rate (m/s^2)
        double fuel = 121; //
        double weight = WEIGHT_EMP + fuel;
        double target_vs = 30;
        // PID control variables
        double integral_vs = 0;  // Accumulated integral for vertical speed
        double prev_error_vs = 0; // Previous error for vertical speed (for derivative term)
        System.out.println("time, vs, hs, dist, alt, ang,weight,acc");
        double NN = 0.7; // rate[0,1]
        PID pid = new PID(0.05, 0.00002, 0.2, 0.0);
        // ***** main simulation loop ******
        while (alt > 0) {
            if (time % 10 == 0 || alt < 100) {
                System.out.println(time + "," + vs + "," + hs + "," + dist + "," + alt + "," + ang + "," + weight + "," + acc);
            }
            if (alt > 4000) {
                target_vs = 30;
            }
            // over 2 km above the ground
            if (2000 < alt && alt < 4000) {    // maintain a vertical speed of [20-25] m/s
                target_vs = 24;
            }
            if (500 < alt && alt < 2000) {    // maintain a vertical speed of [20-25] m/s
                target_vs = 16;
            }
            if (100 < alt && alt < 500) {    // maintain a vertical speed of [20-25] m/s
                target_vs = 12;
            }
            if (20 < alt && alt < 100) {    // maintain a vertical speed of [20-25] m/s
                target_vs = 6;
            }
            if (5 < alt && alt < 20) {    // maintain a vertical speed of [20-25] m/s
                target_vs = 1;
            }
            if (alt <= 5) {
                target_vs = 0;
            }
            if (hs < 2) {
                hs = 0;
                ang = 0;
            }

            // Adjust NN based on vertical PID
            double pid_output = pid.update(vs - target_vs, 1.0);
            NN = Math.max(0, Math.min(NN + pid_output, 1)); // Limit between 0 and 1


            // main computations
            double ang_rad = Math.toRadians(ang);
            double h_acc = Math.sin(ang_rad) * acc;
            double v_acc = Math.cos(ang_rad) * acc;
            double vacc = Moon.getAcc(hs);
            time += dt;
            double dw = dt * ALL_BURN * NN;
            if (fuel > 0) {
                fuel -= dw;
                weight = WEIGHT_EMP + fuel;
                acc = NN * accMax(weight);
            } else { // ran out of fuel
                acc = 0;
            }

            v_acc -= vacc;
            if (hs > 0) {
                hs -= h_acc * dt;
            }
            dist -= hs * dt;
            vs -= v_acc * dt;
            alt -= dt * vs;
        }
    }
}