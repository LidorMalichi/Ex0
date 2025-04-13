package Drone;

import org.apache.poi.ss.usermodel.*;
import org.apache.poi.xssf.usermodel.XSSFWorkbook;

import java.io.FileOutputStream;
import java.io.IOException;

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

    public static void main(String[] args) {
        System.out.println("Simulating Bereshit's Landing with Interpolation:");

        // Excel setup
        Workbook workbook = new XSSFWorkbook();
        Sheet sheet = workbook.createSheet("Bereshit Simulation");

        // Header row
        Row header = sheet.createRow(0);
        String[] headers = {"time", "VS", "HS", "Distance", "Alt", "ang", "Fuel", "Acc", "DVS", "NN"};
        for (int i = 0; i < headers.length; i++) {
            header.createCell(i).setCellValue(headers[i]);
        }

        int rowNum = 1; // start writing from row 1

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
        System.out.println("time, vs, hs, dist, alt, ang, fuel, acc");

        //PID Controllers Vertical Speed, Angle.
        PID vsController = new PID(0.2, 0.00005, 0.3);
        PID angleController = new PID(0.06, 0.00004, 0.2);

        // New PID controller for horizontal speed - reduced gains for slower braking
        PID hsController = new PID(0.015, 0.000002, 0.5);

        // --- Interpolators for Angle and Vertical Speed
        double[] altitudes_vs = {0, 5, 10, 20, 50, 100, 500, 2000, 4000};
        double[] vs_targets = {0, 1, 2, 3, 4, 6, 10, 20, 30};
        LinearInterpolator vsInterp = new LinearInterpolator(altitudes_vs, vs_targets);

        double[] altitudes_ang = {50, 100, 200, 500, 1500};
        double[] angle_targets = {0, 0, 25, 45, 58.3};
        LinearInterpolator angleInterp = new LinearInterpolator(altitudes_ang, angle_targets);

        // New interpolator for horizontal speed targets - higher values for slower deceleration
        double[] altitudes_hs = {0, 100, 250, 500, 1000, 2000, 5000, 10000, 15000};
        double[] hs_targets = {5, 20, 35, 50, 100, 200, 500, 750, 950};
        LinearInterpolator hsInterp = new LinearInterpolator(altitudes_hs, hs_targets);

        LanderStateMachine.LanderState currentState = LanderStateMachine.LanderState.DESCENT;
        LanderStateMachine.LanderState previousState = currentState;

        double NN = 0.7;
        while (alt > 0) {
            if (time % 10 == 0 || alt < 100) {
                System.out.printf("time: %.1f, vs: %.6f, hs: %.6f, dist: %.2f, alt: %.6f, ang: %.6f, fuel: %.6f, acc: %.6f\n",
                        time, vs, hs, dist, alt, ang, fuel, acc);
            }

            // Update state
            currentState = LanderStateMachine.determineLanderState(alt);

            if (currentState == LanderStateMachine.LanderState.APPROACH && previousState == LanderStateMachine.LanderState.BREAKING) {
                vsController.Set_Integral(0.0);
            }

            // Apply state-specific control strategies
            LanderStateMachine.applyControlStrategy(currentState, vsController, angleController);

            // Use interpolators
            double target_vs = vsInterp.interpolate(alt);
            double target_angle = angleInterp.interpolate(alt);
            double target_hs = hsInterp.interpolate(alt);

            // Calculate PID outputs
            double vs_pid_output = vsController.update(vs - target_vs, dt);
            double angle_error = ang - target_angle;
            double angle_pid_output = angleController.update(angle_error, dt);
            double hs_pid_output = hsController.update(hs - target_hs, dt);

            // Write to Excel
            Row row = sheet.createRow(rowNum++);
            row.createCell(0).setCellValue(time);
            row.createCell(1).setCellValue(vs);
            row.createCell(2).setCellValue(hs);
            row.createCell(3).setCellValue(dist);
            row.createCell(4).setCellValue(alt);
            row.createCell(5).setCellValue(ang);
            row.createCell(6).setCellValue(fuel);
            row.createCell(7).setCellValue(acc);
            row.createCell(8).setCellValue(target_vs);
            row.createCell(9).setCellValue(NN);

            if (hs < 2) {
                hs = 0;
            }

            // Apply control logic specific to current state
            switch (currentState) {
                case DESCENT:
                    // Moderate thrust, focus on vertical speed
                    NN = Math.max(0.0, Math.min(NN + vs_pid_output, 1.0));
                    break;

                case BREAKING:
                    // Use the horizontal speed PID for thrust control during braking phase
                    NN = Math.max(0.0, Math.min(NN + hs_pid_output, 1.0));
                    // Begin transitioning to more vertical orientation, but more gradually
                    ang = Math.max(0, Math.min(ang - angle_pid_output, 60));
                    break;

                case APPROACH:
                    // Fine control of thrust
                    NN = Math.max(0.0, Math.min(NN + vs_pid_output, 1.0));
                    ang = Math.max(0, Math.min(ang - angle_pid_output, 60));
                    break;

                case TOUCHDOWN:
                    // Very precise thrust control
                    NN = Math.max(0.0, Math.min(NN + vs_pid_output, 1.0));
                    // Almost vertical orientation with minor adjustments for horizontal
                    ang = Math.max(0, Math.min(ang - angle_pid_output, 60));
                    break;
            }

            // Motion physics
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

            previousState = currentState;
        }
        // Save Excel file
        try (FileOutputStream outputStream = new FileOutputStream("BereshitSimulation.xlsx")) {
            workbook.write(outputStream);
            workbook.close();
            System.out.println("Excel file written successfully!");
        } catch (IOException e) {
            System.err.println("Error writing Excel file: " + e.getMessage());
        }
    }
}