import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

/**
 * This class represents Solution for Ex0 at Aerospace Engineering Course at Ariel University.
 */
public class Bereshit_101 {

    public static final double WEIGHT_EMP = 165; // kg
    public static final double WEIGHT_FULE = 420; // kg
    public static final double WEIGHT_FULL = WEIGHT_EMP + WEIGHT_FULE; // kg
    public static final double MAIN_ENG_F = 430; // N
    public static final double SECOND_ENG_F = 25; // N
    public static final double MAIN_BURN = 0.15; // liter per sec
    public static final double SECOND_BURN = 0.009; // liter per sec
    public static final double ALL_BURN = MAIN_BURN + 8 * SECOND_BURN;

    public static double accMax(double weight) {
        return acc(weight, true, 8);
    }

    public static double acc(double weight, boolean main, int seconds) {
        double t = 0;
        if (main) t += MAIN_ENG_F;
        t += seconds * SECOND_ENG_F;
        return t / weight;
    }

    // Data container for CSV export
    static class LandingSnapshot {
        double time, vs, hs, dist, alt, ang, fuel, acc;

        public LandingSnapshot(double time, double vs, double hs, double dist,
                               double alt, double ang, double fuel, double acc) {
            this.time = time;
            this.vs = vs;
            this.hs = hs;
            this.dist = dist;
            this.alt = alt;
            this.ang = ang;
            this.fuel = fuel;
            this.acc = acc;
        }

        @Override
        public String toString() {
            return String.format("%.2f,%.6f,%.6f,%.2f,%.6f,%.6f,%.6f,%.6f",
                    time, vs, hs, dist, alt, ang, fuel, acc);
        }
    }

    public static void main(String[] args) {
        System.out.println("Simulating Bereshit's Landing with Interpolation:");

        double vs = 24.8;
        double hs = 932;
        double dist = 181 * 1000;
        double ang = 58.3;
        double alt = 13748;
        double time = 0;
        double dt = 1;
        double acc = 0;
        double fuel = 121;
        double weight = WEIGHT_EMP + fuel;

        System.out.println("time, vs, hs, dist, alt, ang, fuel, acc");

        PID vsController = new PID(0.03, 0.00002, 0.5, 0.5);
        PID angleController = new PID(0.03, 0.00002, 0.4, 0.5);

        double[] altitudes_vs = {0, 5, 20, 50, 100, 500, 2000, 4000};
        double[] vs_targets =  {0, 1, 2, 4, 6, 12, 20, 30};
        LinearInterpolator vsInterp = new LinearInterpolator(altitudes_vs, vs_targets);

        double[] altitudes_ang = {50, 100, 200, 500 ,1500};
        double[] angle_targets = {0, 20, 25, 40, 58.3};
        LinearInterpolator angleInterp = new LinearInterpolator(altitudes_ang, angle_targets);

        List<LandingSnapshot> report = new ArrayList<>();

        double NN = 0.7;
        while (alt > 0) {
            if (time % 10 == 0 || alt < 100) {
                System.out.printf("time: %.1f, vs: %.6f, hs: %.6f, dist: %.2f, alt: %.6f, ang: %.6f, fuel: %.6f, acc: %.6f\n",
                        time, vs, hs, dist, alt, ang, fuel, acc);
            }

            // Save current state to report
            report.add(new LandingSnapshot(time, vs, hs, dist, alt, ang, fuel, acc));

            /*
            PID Adjustments based on Altitude
             */

            if (alt < 2000) {
                vsController.Set_P(0.05);
                vsController.Set_I(0.00003);
                vsController.Set_D(0.3);

            }

            if (alt < 500) {
                vsController.Set_P(0.06);
                vsController.Set_I(0.00005);
                vsController.Set_D(0.25);

                angleController.Set_P(0.05);
                angleController.Set_I(0.00004);
                angleController.Set_D(0.2);
            }

            if (alt < 100) {
                angleController.Set_P(0.08);
                angleController.Set_I(0.00007);
                angleController.Set_D(0.4);
            }

            double target_vs = vsInterp.interpolate(alt);
            double target_angle = angleInterp.interpolate(alt);
            double vs_pid_output = vsController.update(vs - target_vs, dt);
            NN = Math.max(0, Math.min(NN + vs_pid_output, 1));

            double angle_error = ang - target_angle;
            double angle_pid_output = angleController.update(angle_error, dt);

            if (alt < 2000) {
                ang = Math.max(0, Math.min(ang - angle_pid_output, 60));
            }

            if (hs < 2) {
                hs = 0;
            }

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
            } else {
                acc = 0;
            }

            v_acc -= vacc;
            if (hs > 0) hs -= h_acc * dt;
            dist -= hs * dt;
            vs -= v_acc * dt;
            alt -= dt * vs;
        }

        // Export to CSV
        try (FileWriter writer = new FileWriter("landing_report.csv")) {
            writer.write("time,vs,hs,dist,alt,ang,fuel,acc\n");
            for (LandingSnapshot snap : report) {
                writer.write(snap + "\n");
            }
            System.out.println("Final landing report saved to landing_report.csv");
        } catch (IOException e) {
            System.err.println("Failed to write CSV report: " + e.getMessage());
        }
    }
}
