package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Base abstract class for all shooter power controllers.
 * Provides the core logic (interpolation) and motor control.
 * Subclasses define the GOAL_POSE and POWER_CURVE.
 */
public abstract class ShooterController {

    protected DcMotorEx shooterMotor;
    protected final Follower follower;
    protected double commandedPower = 0.0;

    // --- Abstract methods to be implemented by children ---
    protected abstract Pose getGoalPose();
    protected abstract double[][] getPowerCurve();

    public ShooterController(Follower follower) {
        this.follower = follower;
    }

    public void initShooter(HardwareMap hardwareMap) {
        // Since both controllers use the same physical motor, we only initialize it once.
        if (shooterMotor == null) {
            shooterMotor = hardwareMap.get(DcMotorEx.class, "shooter");
            shooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            shooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            shooterMotor.setPower(0.0);
        }
    }

    // --- Calculation Logic (Uses the abstract methods) ---

    public double calculateDistanceToGoal() {
        Pose robotPose = follower.getPose();
        Pose goalPose = getGoalPose(); // Gets the specific pose for this controller
        double deltaX = goalPose.getX() - robotPose.getX();
        double deltaY = goalPose.getY() - robotPose.getY();
        return Math.hypot(deltaX, deltaY);
    }

    public double calculateShooterPower() {
        double distance = calculateDistanceToGoal();
        double[][] curve = getPowerCurve(); // Gets the specific curve for this controller
        int lastIndex = curve.length - 1;

        if (distance <= curve[0][0]) return curve[0][1];
        if (distance >= curve[lastIndex][0]) return curve[lastIndex][1];

        // Linear Interpolation
        for (int i = 0; i < lastIndex; i++) {
            if (distance < curve[i + 1][0]) {
                double x1 = curve[i][0];
                double y1 = curve[i][1];
                double x2 = curve[i + 1][0];
                double y2 = curve[i + 1][1];
                return y1 + (y2 - y1) * ((distance - x1) / (x2 - x1));
            }
        }
        return curve[lastIndex][1];
    }

    // --- Control and Accessors ---
    public void setShooterPower(boolean triggerHeld) {
        if (triggerHeld) {
            commandedPower = calculateShooterPower();
            shooterMotor.setPower(commandedPower);
        } else {
            commandedPower = 0.0;
            shooterMotor.setPower(0.0);
        }
    }

    public double getCommandedPower() {
        return commandedPower;
    }
    public double getDistanceToGoal() {
        return calculateDistanceToGoal();
    }
}