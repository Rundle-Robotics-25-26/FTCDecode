//package org.firstinspires.ftc.teamcode;
//
//import com.pedropathing.follower.Follower;
//import com.pedropathing.geometry.Pose;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorEx;
//import com.qualcomm.robotcore.hardware.HardwareMap;
//// Removed unused import: com.qualcomm.robotcore.util.ElapsedTime
//
///**
// * Manages the hardware control and state for the robot's subsystems.
// * This class serves as the central interface between the OpMode and the
// * dedicated controllers (TurretController, ShooterController).
// */
//public class RobotFunctions {
//
//    // --- Turret Constants ---
//    private static final double TURRET_POWER_DEFAULT = 0.3;
//    private static final double TURRET_POWER_HOLD = 0.1;
//    private static final int TURRET_TOLERANCE_TICKS = 10;
//
//    // Turret Conversion Ratios (Re-calculating on the fly based on best practice)
//    private static final double TICKS_PER_90_DEGREE = 470.0;
//    private static final double RADIAN_TO_TICK_RATIO = (Math.PI / 2.0) / TICKS_PER_90_DEGREE;
//    private static final double DEGREE_TO_TICK_RATIO = 90.0 / TICKS_PER_90_DEGREE;
//
//    // --- Hardware and Controller Objects ---
//    private TurretController turretController;
//    private ShooterController shooterController; // NEW: Instance of the separate controller
//    private DcMotorEx turretMotor;
//    private Follower follower;
//
//    // --- State Tracking ---
//    private int lastTurretTargetPosition = 0; // Renamed for clarity
//
//    // --- NEW: Field to hold the dynamic goal pose ---
//    private Pose targetPose;
//
//    //--------------------------- INITIALIZATION ---------------------------------------------------
//
//    /**
//     * Initializes all subsystems and required control components.
//     */
//    public void init(HardwareMap hardwareMap, Follower follower) {
//        this.follower = follower;
//        this.turretController = new TurretController();
//        this.shooterController = new ShooterController(follower); // Initialize ShooterController
//
//        // 1. Initialize Turret Motor
//        turretMotor = hardwareMap.get(DcMotorEx.class, "turret");
//        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        turretMotor.setTargetPosition(0);
//        turretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        turretMotor.setPower(TURRET_POWER_DEFAULT);
//        lastTurretTargetPosition = 0;
//
//        // 2. Initialize Shooter Motor (Delegating hardware setup to the controller)
//        shooterController.initShooter(hardwareMap);
//    }
//
//    //--------------------------- NEW: GOAL POSE SETTER --------------------------------------------
//
//    /**
//     * Sets the field coordinate of the goal target and passes it to the TurretController.
//     * This method resolves the "no set target pose" error in the TeleOp.
//     * @param target The field Pose of the high goal.
//     */
//    public void setTargetPose(Pose target) {
//        this.targetPose = target;
//        // Pass the pose to the controller so it can use it for calculations
//        // NOTE: This assumes TurretController.java has a public setGoalPose(Pose) method.
//        if (turretController != null) {
//            turretController.setGoalPose(target);
//        }
//    }
//
//    //--------------------------- TURRET CONTROL LOOP ----------------------------------------------
//
//    /**
//     * Updates the turret position based on auto-aim calculations.
//     */
//    public void updateTurret() {
//        // 1. Calculate Target
//        int targetPosition = turretController.calculateTurretPosition(follower.getPose());
//
//        // 2. Command Position: Only set the target if it has changed to minimize overhead.
//        if (targetPosition != lastTurretTargetPosition) {
//            turretMotor.setTargetPosition(targetPosition);
//            lastTurretTargetPosition = targetPosition;
//        }
//
//        // 3. Power Control: Use dynamic power to quickly reach the target and then hold position.
//        int currentPosition = turretMotor.getCurrentPosition();
//        if (Math.abs(currentPosition - targetPosition) > TURRET_TOLERANCE_TICKS) {
//            turretMotor.setPower(TURRET_POWER_DEFAULT);
//        } else {
//            turretMotor.setPower(TURRET_POWER_HOLD);
//        }
//    }
//
//    //--------------------------- SHOOTER CONTROL (PASS-THROUGH) -----------------------------------
//
//    /**
//     * Updates the shooter motor power based on the trigger state, delegating
//     * calculation and control to the ShooterController.
//     * @param triggerHeld True if the driver is commanding the shooter to fire.
//     */
//    public void setShooterPower(boolean triggerHeld) {
//        shooterController.setShooterPower(triggerHeld);
//    }
//
//    //--------------------------- TURRET UTILITIES & ACCESSORS -------------------------------------
//
//    /**
//     * Manually resets the turret to its center position (0 ticks).
//     */
//    public void resetTurret() {
//        turretMotor.setTargetPosition(0);
//        turretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        turretMotor.setPower(TURRET_POWER_DEFAULT);
//        lastTurretTargetPosition = 0;
//    }
//
//    /**
//     * Manually overrides the turret's position.
//     * @param targetTicks The target position in motor ticks.
//     */
//    public void setTurretManualPosition(int targetTicks) {
//        if (targetTicks != lastTurretTargetPosition) {
//            turretMotor.setTargetPosition(targetTicks);
//            lastTurretTargetPosition = targetTicks;
//            turretMotor.setPower(TURRET_POWER_DEFAULT);
//        }
//    }
//
//    public double getTurretAngle() {
//        return turretMotor.getCurrentPosition() * RADIAN_TO_TICK_RATIO;
//    }
//
//    public int getTurretTicks() {
//        return turretMotor.getCurrentPosition();
//    }
//
//    public double getTurretAngleDegrees() {
//        return turretMotor.getCurrentPosition() * DEGREE_TO_TICK_RATIO;
//    }
//
//    public boolean isGoalInRange() {
//        return turretController.isGoalInRange(follower.getPose());
//    }
//
//    public String getTurretDebugInfo() {
//        return turretController.getDebugInfo(follower.getPose());
//    }
//
//    // --- SHOOTER ACCESSORS (PASS-THROUGH) ---
//
//    /**
//     * @return The distance to the goal in inches.
//     */
//    public double getShooterDistanceToGoal() {
//        return shooterController.getDistanceToGoal();
//    }
//
//    /**
//     * @return The last power commanded to the shooter motor (0.0 if off).
//     */
//    public double getCommandedShooterPower() {
//        return shooterController.getCommandedPower();
//    }
//}