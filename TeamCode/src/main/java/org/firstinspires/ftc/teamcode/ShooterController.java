package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Manages the shooter motor and calculates its required power based on
 * the robot's distance to the goal using linear interpolation from a curve.
 */
public class ShooterController {

    // --- Configuration Constants ---
    private final Pose GOAL_POSE = new Pose(10.0, 136.0, 0.0);

    // Lookup table: {distance_in_inches, power}
    private final double[][] POWER_CURVE = {
            {0.0, -0.3},
            {20.0, -0.45},
            {40.0, -0.5},
            {60.0, -0.55},
            {82.0, -0.6},
            {120.0, -0.75},
            {130.0, -0.8}
    };

    // --- Hardware and State ---
    private DcMotorEx shooterMotor;
    private final Follower follower; // Now final, must be set in constructor
    private double commandedPower = 0.0;

    // --- Constructor ---
    public ShooterController(Follower follower) {
        this.follower = follower;
    }

    //--------------------------- INITIALIZATION ---------------------------------------------------

    /**
     * Initializes the shooter motor hardware.
     */
    public void initShooter(HardwareMap hardwareMap) {
        shooterMotor = hardwareMap.get(DcMotorEx.class, "shooter");
        shooterMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooterMotor.setPower(0.0);
    }

    //--------------------------- CALCULATION METHODS ----------------------------------------------

    /**
     * Calculates the direct distance from the robot's current pose to the defined goal pose.
     * @return The distance in inches (assuming Pose coordinates are in inches).
     */
    public double calculateDistanceToGoal() {
        Pose robotPose = follower.getPose();
        double deltaX = GOAL_POSE.getX() - robotPose.getX();
        double deltaY = GOAL_POSE.getY() - robotPose.getY();
        return Math.hypot(deltaX, deltaY);
    }

    /**
     * Calculates the required shooter power by linearly interpolating the distance.
     * @return The target power (0.0 to 1.0).
     */
    public double calculateShooterPower() {
        double distance = calculateDistanceToGoal();

        // 1. Boundary Check: Below minimum distance
        if (distance <= POWER_CURVE[0][0]) {
            return POWER_CURVE[0][1];
        }

        // 2. Boundary Check: Above maximum distance
        int lastIndex = POWER_CURVE.length - 1;
        if (distance >= POWER_CURVE[lastIndex][0]) {
            return POWER_CURVE[lastIndex][1];
        }

        // 3. Linear Interpolation
        for (int i = 0; i < lastIndex; i++) {
            if (distance < POWER_CURVE[i + 1][0]) {
                double x1 = POWER_CURVE[i][0];
                double y1 = POWER_CURVE[i][1];
                double x2 = POWER_CURVE[i + 1][0];
                double y2 = POWER_CURVE[i + 1][1];

                // Linear interpolation: y = y1 + (y2 - y1) * ((x - x1) / (x2 - x1))
                return y1 + (y2 - y1) * ((distance - x1) / (x2 - x1));
            }
        }

        // Fallback
        return POWER_CURVE[lastIndex][1];
    }

    //--------------------------- CONTROL METHOD ---------------------------------------------------

    /**
     * Updates the shooter motor power based on the trigger state.
     * @param triggerHeld True if the driver is commanding the shooter to fire.
     */
    public void setShooterPower(boolean triggerHeld) {
        if (triggerHeld) {
            commandedPower = calculateShooterPower();
            shooterMotor.setPower(commandedPower);
        } else {
            commandedPower = 0.0;
            shooterMotor.setPower(0.0);
        }
    }

    //--------------------------- ACCESSOR METHODS -------------------------------------------------

    /**
     * @return The last power commanded to the motor (0.0 if off).
     */
    public double getCommandedPower() {
        return commandedPower;
    }

    /**
     * @return The distance to the goal in inches, recalculated on demand.
     */
    public double getDistanceToGoal() {
        return calculateDistanceToGoal();
    }
}