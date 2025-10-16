package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class RobotFunctions {
    private TurretController turretController;
    private ShooterController shooterController; // ADD THIS
    private DcMotorEx turretMotor;
    private Follower follower;

    // Turret motor specs - adjust based on your motor
    private final double TURRET_TICKS_PER_REVOLUTION = 537.6; // Gobilda 312 RPM motor

    // Track current turret angle for continuous rotation
    private double currentTurretAngle = 0;

    //---------------------------TURRET-------------------------------------------------------------
    public void initTurret(HardwareMap hardwareMap, Follower follower) {
        this.follower = follower;
        turretController = new TurretController(TURRET_TICKS_PER_REVOLUTION);

        // Initialize turret motor
        turretMotor = hardwareMap.get(DcMotorEx.class, "turretMotor");
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turretMotor.setTargetPosition(0);
        turretMotor.setPower(0.3); // Adjust power as needed

        currentTurretAngle = 0; // Reset tracked angle
    }

    /**
     * Call this in your tele-op loop or autonomous to update turret position
     */
    public void updateTurret() {
        Pose currentRobotPose = follower.getPose();

        // Convert current encoder position to angle for continuous rotation
        currentTurretAngle = turretMotor.getCurrentPosition() / (TURRET_TICKS_PER_REVOLUTION / (2 * Math.PI));

        // Calculate new target with continuous rotation
        int targetPosition = turretController.calculateTurretPosition(
                currentRobotPose, currentTurretAngle);

        turretMotor.setTargetPosition(targetPosition);

        // Optional: Only enable power when not at target to save battery
        if (Math.abs(turretMotor.getCurrentPosition() - targetPosition) > 10) {
            turretMotor.setPower(0.3);
        } else {
            turretMotor.setPower(0.1); // Hold position with lower power
        }

        // Update our tracked angle
        currentTurretAngle = targetPosition / (TURRET_TICKS_PER_REVOLUTION / (2 * Math.PI));
    }

    //---------------------------SHOOTER------------------------------------------------------------
    public void initShooter(HardwareMap hardwareMap, Follower follower) {
        shooterController = new ShooterController();
        shooterController.initShooter(hardwareMap, follower);
    }

    public void updateShooter(boolean triggerHeld) {
        shooterController.setShooterPower(triggerHeld);
    }

    public double getCurrentShooterPower() {
        return shooterController.getCurrentPower();
    }

    public double getDistanceToGoal() {
        return shooterController.getDistanceToGoal();
    }
    //----------------------------------------------------------------------------------------------

    /**
     * Manually override turret position (for testing)
     */
    public void setTurretManualPosition(double angleRadians) {
        int position = (int)(angleRadians * (TURRET_TICKS_PER_REVOLUTION / (2 * Math.PI)));
        turretMotor.setTargetPosition(position);
        currentTurretAngle = angleRadians; // Update tracked angle
    }

    /**
     * Get current turret angle in radians
     */
    public double getTurretAngle() {
        return turretMotor.getCurrentPosition() / (TURRET_TICKS_PER_REVOLUTION / (2 * Math.PI));
    }

    /**
     * Get current turret angle in continuous system (-∞ to +∞)
     */
    public double getContinuousTurretAngle() {
        return currentTurretAngle;
    }

    /**
     * Reset turret encoder and tracked angle
     */
    public void resetTurret() {
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        currentTurretAngle = 0;
        turretMotor.setTargetPosition(0);
    }

    /**
     * Check if goal is within turret's range
     */
    public boolean isGoalInRange() {
        Pose currentRobotPose = follower.getPose();
        return turretController.isGoalInRange(currentRobotPose);
    }
}