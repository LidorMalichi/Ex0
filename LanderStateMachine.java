package Drone;

public class LanderStateMachine {

    public enum LanderState {
        DESCENT,     // Initial descent from high altitude
        BREAKING,     // Reducing velocity during mid-descent
        APPROACH,    // Final approach phase
        TOUCHDOWN    // Final landing phase
    }

    public static LanderState determineLanderState(double alt) {
        if (alt > 2000) {
            return LanderState.DESCENT;
        } else if (alt > 500) {
            return LanderState.BREAKING;
        } else if (alt > 200) {
            return LanderState.APPROACH;
        } else {
            return LanderState.TOUCHDOWN;
        }
    }

    public static void applyControlStrategy(LanderState state, PID vsController, PID angleController) {
        switch (state) {
            case DESCENT:
                // Focus on reducing horizontal speed and maintaining a reasonable vertical descent
                vsController.Set_P(0.05);
                vsController.Set_I(0.00002);
                vsController.Set_D(0.2);

                angleController.Set_P(0.04);
                angleController.Set_I(0.00002);
                angleController.Set_D(0.15);
                break;

            case BREAKING:
                angleController.Set_P(0.06);
                angleController.Set_I(0.00003);
                angleController.Set_D(0.2);
                break;

            case APPROACH:
                // Fine control for approach, focus on both vertical and horizontal speeds
                vsController.Set_P(0.004);
                vsController.Set_I(0.00002);
                vsController.Set_D(0.1);

                angleController.Set_P(0.08);
                angleController.Set_I(0.00005);
                angleController.Set_D(0.3);
                break;

            case TOUCHDOWN:
                // Very precise control for soft landing
                vsController.Set_P(0.05);
                vsController.Set_I(0.00002);
                vsController.Set_D(0.2);

                angleController.Set_P(0.1);
                angleController.Set_I(0.00007);
                angleController.Set_D(0.4);
                break;
        }
    }
}
