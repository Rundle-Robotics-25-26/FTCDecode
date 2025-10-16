package org.firstinspires.ftc.teamcode;

import com.pedropathing.geometry.Pose;

public class TurretController {
    private final Pose goalPose = new Pose(10, 136, 0); // Goal position
    private final double maxTurretAngle = Math.toRadians(270); // 270 degrees in radians
    private final double ticksPerRadians; // Encoder ticks per radian of turret rotation

    public TurretController(double ticksPerRotation) {
        this.ticksPerRadians = ticksPerRotation / (2 * Math.PI);
    }

    /**
     * Calculate the angle from robot to goal
     */
    public double calculateAngleToGoal(Pose robotPose) {
        double deltaX = goalPose.getX() - robotPose.getX();
        double deltaY = goalPose.getY() - robotPose.getY();

        // Calculate absolute angle to goal
        double absoluteAngleToGoal = Math.atan2(deltaY, deltaX);

        // Adjust for robot heading (field-centric to robot-centric)
        double relativeAngleToGoal = absoluteAngleToGoal - robotPose.getHeading();

        // Normalize angle to [-π, π] range
        return normalizeAngle(relativeAngleToGoal);
    }

    /**
     * Calculate turret motor target position (original method for compatibility)
     */
    public int calculateTurretPosition(Pose robotPose) {
        double angleToGoal = calculateAngleToGoal(robotPose);
        // Constrain angle to turret's physical limits
        double constrainedAngle = constrainTurretAngle(angleToGoal);
        // Convert to encoder ticks
        return (int)(constrainedAngle * ticksPerRadians);
    }

    /**
     * NEW: Calculate turret position with continuous rotation
     */
    public int calculateTurretPosition(Pose robotPose, double currentTurretAngle) {
        double desiredAngleToGoal = calculateAngleToGoal(robotPose);

        // Find the closest equivalent angle within our continuous rotation system
        double wrappedTargetAngle = findContinuousAngle(desiredAngleToGoal, currentTurretAngle);

        return (int)(wrappedTargetAngle * ticksPerRadians);
    }

    /**
     * Find the best target angle that's closest to current position
     * while maintaining continuous rotation within limits
     */
    private double findContinuousAngle(double desiredAngle, double currentAngle) {
        // Start with the basic desired angle
        double bestAngle = desiredAngle;
        double bestDistance = Math.abs(desiredAngle - currentAngle);

        // Try adding/subtracting full rotations to find closer alternatives
        for (int i = -2; i <= 2; i++) {
            if (i == 0) continue; // Skip the original

            double candidate = desiredAngle + (i * 2 * Math.PI);

            // Only consider candidates within our physical limits
            if (candidate >= -maxTurretAngle && candidate <= maxTurretAngle) {
                double distance = Math.abs(candidate - currentAngle);
                if (distance < bestDistance) {
                    bestDistance = distance;
                    bestAngle = candidate;
                }
            }
        }

        return bestAngle;
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
     * Constrain turret angle to physical limits (-135° to +135° from center)
     */
    private double constrainTurretAngle(double angle) {
        double max = maxTurretAngle / 2; // ±135 degrees
        double min = -maxTurretAngle / 2;

        if (angle > max) return max;
        if (angle < min) return min;
        return angle;
    }

    /**
     * Check if goal is within turret's range
     */
    public boolean isGoalInRange(Pose robotPose) {
        double angleToGoal = calculateAngleToGoal(robotPose);
        double max = maxTurretAngle / 2;
        return angleToGoal >= -max && angleToGoal <= max;
    }

    /**
     * NEW: Find the shortest path to the goal (continuous rotation optimization)
     */
    public double getShortestPathAngle(double currentTurretAngle, double desiredAngle) {
        double difference = desiredAngle - currentTurretAngle;

        // Normalize to find shortest path
        if (difference > Math.PI) difference -= 2 * Math.PI;
        if (difference < -Math.PI) difference += 2 * Math.PI;

        double targetAngle = currentTurretAngle + difference;

        // Constrain to physical limits
        return constrainTurretAngle(targetAngle);
    }

    /**
     * NEW: Get the maximum turret angle (for reference)
     */
    public double getMaxTurretAngle() {
        return maxTurretAngle;
    }

    /**
     * NEW: Get the center-to-limit angle (±135°)
     */
    public double getMaxTurretAngleFromCenter() {
        return maxTurretAngle / 2;
    }
}