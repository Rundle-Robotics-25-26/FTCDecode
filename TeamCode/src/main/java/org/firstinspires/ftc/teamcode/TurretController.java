package org.firstinspires.ftc.teamcode;

import com.pedropathing.geometry.Pose;

/**
 * Manages the calculation of the target position (in motor ticks) for a turret.
 * Assumes the turret is centered at its starting position (0 ticks) and has
 * a 180-degree range (-90 to +90 degrees, or -PI/2 to +PI/2 radians).
 */
public class TurretController {

    // --- Constants ---
    // A constant representing 90 degrees in radians (PI/2) for clarity
    private static final double DEGREE_90_IN_RADIANS = Math.PI / 2.0;

    // --- MODIFIED: GOAL_POSE is now a dynamic field ---
    private Pose goalPose = new Pose(0.0, 136.0, 0.0); // Turret's target location on the field (Default)

    // Motor limits (470 ticks corresponds to 90 degrees/PI/2 radians)
    private final int MAX_RIGHT_TICKS = 470;
    private final int MAX_LEFT_TICKS = -470;

    // Pre-calculate the conversion factor to avoid repeated calculation
    // Ticks / Radian = (470 ticks) / (PI/2 radians)
    private final double TICKS_PER_RADIAN = MAX_RIGHT_TICKS / DEGREE_90_IN_RADIANS;

    // --- NEW: Setter method to update the goal position (called from RobotFunctions) ---
    /**
     * Sets the new target goal position on the field.
     * @param pose The new Pose of the goal target.
     */
    public void setGoalPose(Pose pose) {
        this.goalPose = pose;
    }

    // --- Calculation Methods ---

    /**
     * Calculates the absolute angle from the robot's current position to the goal.
     * This angle is relative to the field's vertical (Y) axis (Math.atan2(dx, dy)).
     * @param robotPose The current pose of the robot on the field.
     * @return The absolute angle to the goal in radians.
     */
    public double calculateAbsoluteAngle(Pose robotPose) {
        // MODIFIED: References the dynamic 'goalPose' field
        double deltaX = goalPose.getX() - robotPose.getX();
        double deltaY = goalPose.getY() - robotPose.getY();
        // Uses Math.atan2(dx, dy) which is relative to the positive Y axis (vertical).
        return Math.atan2(deltaX, deltaY);
    }

    /**
     * Calculates the required turret motor target position in ticks to face the goal.
     * Turret Angle = (Robot Heading - 90°) + Absolute Angle
     * @param robotPose The current pose of the robot on the field.
     * @return The constrained target position for the turret motor in ticks.
     */
    public int calculateTurretPosition(Pose robotPose) {
        // Step 1: Get absolute angle to goal (only depends on coordinates)
        double absoluteAngle = calculateAbsoluteAngle(robotPose);

        // Step 2: Calculate turret angle based on robot heading and absolute angle.
        // Formula: Turret Angle = (Robot Heading - 90°) + Absolute Angle
        // We use the pre-calculated DEGREE_90_IN_RADIANS constant.
        double turretAngle = (robotPose.getHeading() - DEGREE_90_IN_RADIANS) + absoluteAngle;

        // Step 3: Normalize the angle to the required [-π, π] range.
        turretAngle = normalizeAngle(turretAngle);

        // Step 4: Convert angle (radians) to ticks using the pre-calculated constant.
        int targetTicks = (int) (turretAngle * TICKS_PER_RADIAN);

        // Step 5: Constrain to physical limits and return.
        return constrainTurretTicks(targetTicks);
    }

    // --- Utility Methods ---

    /**
     * Normalize an angle to the [-π, π] range.
     * @param angle The angle in radians.
     * @return The normalized angle in radians.
     */
    private double normalizeAngle(double angle) {
        // Canonical way to normalize a radian angle to [-pi, pi]
        return angle - 2 * Math.PI * Math.floor((angle + Math.PI) / (2 * Math.PI));
    }

    /**
     * Constrains the calculated turret ticks to the motor's physical limits.
     * @param ticks The raw target ticks.
     * @return The constrained ticks.
     */
    private int constrainTurretTicks(int ticks) {
        // Use Math.max and Math.min for cleaner, single-line constraint logic.
        return Math.max(MAX_LEFT_TICKS, Math.min(MAX_RIGHT_TICKS, ticks));
    }

    /**
     * Checks if the goal is within the turret's operational range given the robot's pose.
     * @param robotPose The current pose of the robot on the field.
     * @return true if the goal is reachable by the turret, false otherwise.
     */
    public boolean isGoalInRange(Pose robotPose) {
        int targetTicks = calculateTurretPosition(robotPose);
        // The constrainTurretTicks method ensures that targetTicks is exactly at the limit
        // if the raw angle was outside. We must check if the constrained value is NOT at the limit.
        return targetTicks > MAX_LEFT_TICKS && targetTicks < MAX_RIGHT_TICKS;
    }

    // --- Accessor (Getter) Methods ---

    public int getMaxRightTicks() {
        return MAX_RIGHT_TICKS;
    }

    public int getMaxLeftTicks() {
        return MAX_LEFT_TICKS;
    }

    // --- Debugging Method ---

    /**
     * Provides detailed debug information about the angle calculation process.
     * Note: This recalculates the steps for debugging purposes, which is acceptable here.
     * @param robotPose The current pose of the robot.
     * @return A formatted String containing debug data.
     */
    public String getDebugInfo(Pose robotPose) {
        double absoluteAngle = calculateAbsoluteAngle(robotPose);
        double turretAngleRaw = (robotPose.getHeading() - DEGREE_90_IN_RADIANS) + absoluteAngle;
        double turretAngleNormalized = normalizeAngle(turretAngleRaw);
        int targetTicks = calculateTurretPosition(robotPose); // Recalculate for final constrained value

        return String.format("Abs: %.1f°, RobotH: %.1f°, Turret(Raw): %.1f°, Turret(Norm): %.1f°, Ticks: %d",
                Math.toDegrees(absoluteAngle),
                Math.toDegrees(robotPose.getHeading()),
                Math.toDegrees(turretAngleRaw),
                Math.toDegrees(turretAngleNormalized),
                targetTicks);
    }
}