package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;

public class ShooterControllerTargetA extends ShooterController {

    // --- Configuration for Target A (e.g., Close Goal) ---
    private static final Pose GOAL_POSE_A = new Pose(127.0, 132.0, 0.0);

    // Example Curve 1: Optimized for close shots
    private static final double[][] POWER_CURVE_A = {
            {0.0, 0.45},
            {20.0, 0.45},
            {40.0, 0.50},
            {82, 0.60},
            {120, 0.75},
            {130,0.80}
            // Max power needed for this goal
    };

    public ShooterControllerTargetA(Follower follower) {
        super(follower);
    }

    @Override
    protected Pose getGoalPose() {
        return GOAL_POSE_A;
    }

    @Override
    protected double[][] getPowerCurve() {
        return POWER_CURVE_A;
    }
}