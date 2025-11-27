package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;

public class ShooterControllerTargetB extends ShooterController {

    // --- Configuration for Target B (e.g., Far Goal) ---
    private static final Pose GOAL_POSE_B = new Pose(17.0, 132.0, 0.0); // Different location

    // Example Curve 2: Optimized for long-range shots
    private static final double[][] POWER_CURVE_B = {
            {0.0, 0.45},
            {20.0, 0.45},
            {40.0, 0.50},
            {82, 0.60},
            {120, 0.75},
            {130,0.80} // Longer range, higher max power
    };

    public ShooterControllerTargetB(Follower follower) {
        super(follower);
    }

    @Override
    protected Pose getGoalPose() {
        return GOAL_POSE_B;
    }

    @Override
    protected double[][] getPowerCurve() {
        return POWER_CURVE_B;
    }
}