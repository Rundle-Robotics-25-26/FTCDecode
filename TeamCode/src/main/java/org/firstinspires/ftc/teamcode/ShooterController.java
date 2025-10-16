package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotor;

public class ShooterController {
    private final Pose goalPose = new Pose(10, 136, 0);
    private DcMotorEx shooterMotor;
    private Follower follower;
    private double currentPower = 0;

    // Lookup table: {distance_in_inches, power}
    // TEST AND ADJUST THESE VALUES BASED ON REAL TESTING!
    private final double[][] POWER_CURVE = {
            {0, 0.3},   // 0 inches: 30% power
            {20, 0.4},  // 20 inches: 40% power
            {40, 0.55}, // 40 inches: 55% power
            {60, 0.75}, // 60 inches: 75% power
            {80, 0.9},  // 80 inches: 90% power
            {100, 1.0}  // 100+ inches: 100% power
    };

    public void initShooter(HardwareMap hardwareMap, Follower follower) {
        this.follower = follower;
        shooterMotor = hardwareMap.get(DcMotorEx.class, "shooterMotor");
        shooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    public double calculateDistanceToGoal() {
        Pose robotPose = follower.getPose();
        double deltaX = goalPose.getX() - robotPose.getX();
        double deltaY = goalPose.getY() - robotPose.getY();
        return Math.sqrt(deltaX * deltaX + deltaY * deltaY);
    }

    public double calculateShooterPower() {
        double distance = calculateDistanceToGoal();

        // Handle distances below first point
        if (distance <= POWER_CURVE[0][0]) {
            return POWER_CURVE[0][1];
        }

        // Handle distances above last point
        if (distance >= POWER_CURVE[POWER_CURVE.length - 1][0]) {
            return POWER_CURVE[POWER_CURVE.length - 1][1];
        }

        // Find the two closest data points and interpolate
        for (int i = 0; i < POWER_CURVE.length - 1; i++) {
            if (distance >= POWER_CURVE[i][0] && distance <= POWER_CURVE[i + 1][0]) {
                double x1 = POWER_CURVE[i][0];
                double y1 = POWER_CURVE[i][1];
                double x2 = POWER_CURVE[i + 1][0];
                double y2 = POWER_CURVE[i + 1][1];

                // Linear interpolation: y = y1 + (y2 - y1) * ((x - x1) / (x2 - x1))
                return y1 + (y2 - y1) * ((distance - x1) / (x2 - x1));
            }
        }

        // Fallback
        return POWER_CURVE[POWER_CURVE.length - 1][1];
    }

    public void setShooterPower(boolean triggerHeld) {
        if (triggerHeld) {
            currentPower = calculateShooterPower();
            shooterMotor.setPower(currentPower);
        } else {
            currentPower = 0;
            shooterMotor.setPower(0);
        }
    }

    public double getCurrentPower() {
        return currentPower;
    }

    public double getDistanceToGoal() {
        return calculateDistanceToGoal();
    }

    /**
     * For testing: manually override the power curve
     */
    public void setPowerCurve(double[][] newPowerCurve) {
        // You can use this method to update the power curve during testing
        // without recompiling
        // this.POWER_CURVE = newPowerCurve; // Would need to remove final modifier
    }
}