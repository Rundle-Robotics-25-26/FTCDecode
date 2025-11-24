package org.firstinspires.ftc.teamcode;

import com.pedropathing.geometry.Pose;

/**
 * Calculates the required turret position (in ticks) based on the robot's current pose
 * and the fixed target location.
 */
public class MomentaryTurretController {

    // --- Turret Physical Limits ---
    private static final int MAX_RIGHT_TICKS = 470;
    private static final int MAX_LEFT_TICKS = -470;

    // --- Target Location (Fixed Goal) ---
    private static final Pose TARGET_POSE = new Pose(10, 136, 0);

    // --- Conversion Factors ---
    private static final double TICK_PER_RADIAN = 470.0 / (Math.PI / 2.0);

    /**
     * Calculates the required turret motor position (in ticks) to aim at the target.
     */
    public int calculateTurretPosition(Pose currentRobotPose) {
        double deltaX = TARGET_POSE.getX() - currentRobotPose.getX();
        double deltaY = TARGET_POSE.getY() - currentRobotPose.getY();

        double absoluteAngleToGoal = Math.atan2(deltaY, deltaX);
        double robotHeading = currentRobotPose.getHeading();
        double relativeAngle = absoluteAngleToGoal - robotHeading;

        // Normalize angle to the range (-PI, PI]
        relativeAngle = normalizeAngle(relativeAngle);

        int targetTicks = (int) (relativeAngle * TICK_PER_RADIAN);

        // Clamp the position to physical limits
        return Math.min(Math.max(targetTicks, MAX_LEFT_TICKS), MAX_RIGHT_TICKS);
    }

    private double normalizeAngle(double angle) {
        while (angle > Math.PI) angle -= 2 * Math.PI;
        while (angle <= -Math.PI) angle += 2 * Math.PI;
        return angle;
    }

    public boolean isGoalInRange(Pose currentRobotPose) {
        int targetTicks = calculateTurretPosition(currentRobotPose);
        return targetTicks >= MAX_LEFT_TICKS && targetTicks <= MAX_RIGHT_TICKS;
    }

    public String getDebugInfo(Pose currentRobotPose) {
        double deltaX = TARGET_POSE.getX() - currentRobotPose.getX();
        double deltaY = TARGET_POSE.getY() - currentRobotPose.getY();
        double absoluteAngle = Math.atan2(deltaY, deltaY);
        double robotHeading = currentRobotPose.getHeading();
        double relativeAngle = normalizeAngle(absoluteAngle - robotHeading);

        return String.format("Abs: %.2f rad, Rel: %.2f rad (%.1f deg)",
                absoluteAngle, relativeAngle, Math.toDegrees(relativeAngle));
    }
}