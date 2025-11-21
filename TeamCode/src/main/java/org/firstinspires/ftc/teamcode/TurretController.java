package org.firstinspires.ftc.teamcode;

import com.pedropathing.geometry.Pose;

public class TurretController {
    private final Pose goalPose = new Pose(10, 136, 0); // Goal position

    // 470 ticks in each direction
    private final int MAX_RIGHT_TICKS = 470;   // 90° to the right
    private final int MAX_LEFT_TICKS = -470;   // 90° to the left

    public TurretController() {
    }

    /**
     * Calculate the absolute angle to goal (only depends on coordinates)
     */
    public double calculateAbsoluteAngle(Pose robotPose) {
        double deltaX = goalPose.getX() - robotPose.getX();
        double deltaY = goalPose.getY() - robotPose.getY();
        return Math.atan2(deltaX, deltaY); // Relative to vertical field line
    }

    /**
     * Calculate turret motor target position in ticks
     */
    public int calculateTurretPosition(Pose robotPose) {
        // Step 1: Get absolute angle to goal (only depends on coordinates)
        double absoluteAngle = calculateAbsoluteAngle(robotPose);

        // Step 2: Calculate turret angle using your formula
        // Turret Angle = (Robot Heading - 90°) + Absolute Angle
        double turretAngle = (robotPose.getHeading() - Math.toRadians(90)) + absoluteAngle;

        // Normalize angle to [-π, π] range
        turretAngle = normalizeAngle(turretAngle);

        // Convert angle to ticks
        double ticksPerRadian = 470.0 / (Math.PI / 2);
        int targetTicks = (int)(turretAngle * ticksPerRadian);

        // Constrain to physical limits
        return constrainTurretTicks(targetTicks);
    }

    /**
     * Normalize angle to [-π, π] range
     */
    private double normalizeAngle(double angle) {
        while (angle > Math.PI) angle -= 2 * Math.PI;
        while (angle < -Math.PI) angle += 2 * Math.PI;
        return angle;
    }

    /**
     * Constrain turret ticks to physical limits
     */
    private int constrainTurretTicks(int ticks) {
        if (ticks > MAX_RIGHT_TICKS) return MAX_RIGHT_TICKS;
        if (ticks < MAX_LEFT_TICKS) return MAX_LEFT_TICKS;
        return ticks;
    }

    /**
     * Check if goal is within turret's range
     */
    public boolean isGoalInRange(Pose robotPose) {
        int targetTicks = calculateTurretPosition(robotPose);
        return targetTicks > MAX_LEFT_TICKS && targetTicks < MAX_RIGHT_TICKS;
    }

    /**
     * Get the maximum right ticks
     */
    public int getMaxRightTicks() {
        return MAX_RIGHT_TICKS;
    }

    /**
     * Get the maximum left ticks
     */
    public int getMaxLeftTicks() {
        return MAX_LEFT_TICKS;
    }

    /**
     * Get debug info about the angle calculation
     */
    public String getDebugInfo(Pose robotPose) {
        double absoluteAngle = calculateAbsoluteAngle(robotPose);
        double turretAngle = (robotPose.getHeading() - Math.toRadians(90)) + absoluteAngle;
        turretAngle = normalizeAngle(turretAngle);
        int targetTicks = calculateTurretPosition(robotPose);

        return String.format("Abs: %.1f°, RobotH: %.1f°, Turret: %.1f°, Ticks: %d",
                Math.toDegrees(absoluteAngle),
                Math.toDegrees(robotPose.getHeading()),
                Math.toDegrees(turretAngle),
                targetTicks);
    }
}