package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class RobotFunctions {
    private TurretController turretController;
    private DcMotorEx turretMotor;
    private Follower follower;

    // Track current turret angle for reference
    private double currentTurretAngle = 0;

    //---------------------------TURRET-------------------------------------------------------------
    public void initTurret(HardwareMap hardwareMap, Follower follower) {
        this.follower = follower;
        turretController = new TurretController();

        // Initialize turret motor
        turretMotor = hardwareMap.get(DcMotorEx.class, "turret");
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turretMotor.setTargetPosition(0);
        turretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        turretMotor.setPower(0.6);

        currentTurretAngle = 0;
    }

    /**
     * Call this in your tele-op loop or autonomous to update turret position
     */
    public void updateTurret() {
        Pose currentRobotPose = follower.getPose();
        int targetPosition = turretController.calculateTurretPosition(currentRobotPose);

        turretMotor.setTargetPosition(targetPosition);

        // Only enable power when not at target to save battery
        if (Math.abs(turretMotor.getCurrentPosition() - targetPosition) > 10) {
            turretMotor.setPower(0.6);
        } else {
            turretMotor.setPower(0.3);
        }

        // Update our tracked angle
        currentTurretAngle = targetPosition * (Math.PI / 2) / 470.0;
    }

    /**
     * Manually override turret position (for testing)
     */
    public void setTurretManualPosition(int targetTicks) {
        turretMotor.setTargetPosition(targetTicks);
        currentTurretAngle = targetTicks * (Math.PI / 2) / 470.0;
    }

    /**
     * Get current turret angle in radians
     */
    public double getTurretAngle() {
        return turretMotor.getCurrentPosition() * (Math.PI / 2) / 470.0;
    }

    /**
     * Get current turret angle in continuous system
     */
    public double getContinuousTurretAngle() {
        return currentTurretAngle;
    }

    /**
     * Get current turret position in ticks
     */
    public int getTurretTicks() {
        return turretMotor.getCurrentPosition();
    }

    /**
     * Get current turret angle in degrees
     */
    public double getTurretAngleDegrees() {
        return getTurretTicks() * 90.0 / 470.0;
    }

    /**
     * Reset turret encoder and tracked angle
     */
    public void resetTurret() {
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turretMotor.setTargetPosition(0);
        turretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        currentTurretAngle = 0;
    }

    /**
     * Check if goal is within turret's range
     */
    public boolean isGoalInRange() {
        Pose currentRobotPose = follower.getPose();
        return turretController.isGoalInRange(currentRobotPose);
    }

    /**
     * Get turret limits for reference
     */
    public int getMaxRightTicks() {
        return turretController.getMaxRightTicks();
    }

    public int getMaxLeftTicks() {
        return turretController.getMaxLeftTicks();
    }

    /**
     * Get debug info about turret calculation
     */
    public String getTurretDebugInfo() {
        Pose currentRobotPose = follower.getPose();
        return turretController.getDebugInfo(currentRobotPose);
    }
}