//package org.firstinspires.ftc.teamcode;
//
//import com.pedropathing.follower.Follower;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorEx;
//import com.qualcomm.robotcore.hardware.HardwareMap;
//
///**
// * Manages the hardware control for the Momentary Aim Turret System.
// * Turret aims only when commanded, otherwise defaults to 0 ticks.
// */
//public class MomentaryRobotFunctions {
//
//    // --- Turret Constants ---
//    private static final double TURRET_POWER_DEFAULT = 0.3;
//    private static final double TURRET_POWER_HOLD = 0.1;
//    private static final int TURRET_TOLERANCE_TICKS = 10;
//
//    // Turret Conversion Ratios
//    private static final double TICKS_PER_90_DEGREE = 470.0;
//    private static final double RADIAN_TO_TICK_RATIO = (Math.PI / 2.0) / TICKS_PER_90_DEGREE;
//    private static final double DEGREE_TO_TICK_RATIO = 90.0 / TICKS_PER_90_DEGREE;
//
//    // --- Hardware and Controller Objects ---
//    // Uses the new MomentaryTurretController
//    private MomentaryTurretController turretController;
//    private ShooterController shooterController;
//    private DcMotorEx turretMotor;
//    private Follower follower;
//
//    // --- State Tracking ---
//    private int lastTurretTargetPosition = 0;
//
//    //--------------------------- INITIALIZATION ---------------------------------------------------
//
//    /**
//     * Initializes all subsystems and required control components.
//     */
//    public void init(HardwareMap hardwareMap, Follower follower) {
//        this.follower = follower;
//        this.turretController = new MomentaryTurretController(); // Use the new controller
//        this.shooterController = new ShooterController(follower);
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
//        // 2. Initialize Shooter Motor
//        shooterController.initShooter(hardwareMap);
//    }
//
//    //--------------------------- TURRET CONTROL LOOP (MOMENTARY LOGIC) ----------------------------
//
//    /**
//     * Updates the turret position based on the aim command.
//     * If aimCommanded is true, the turret tracks the goal.
//     * If aimCommanded is false, the turret defaults to 0 ticks.
//     * @param aimCommanded True if the driver is holding the aim button/trigger.
//     */
//    public void updateTurret(boolean aimCommanded) {
//        int targetPosition;
//
//        if (aimCommanded) {
//            // Turret tracks the goal
//            targetPosition = turretController.calculateTurretPosition(follower.getPose());
//        } else {
//            // Turret returns to 0 ticks (center)
//            targetPosition = 0;
//        }
//
//        // 2. Command Position
//        if (targetPosition != lastTurretTargetPosition) {
//            turretMotor.setTargetPosition(targetPosition);
//            lastTurretTargetPosition = targetPosition;
//        }
//
//        // 3. Power Control
//        int currentPosition = turretMotor.getCurrentPosition();
//        if (Math.abs(currentPosition - targetPosition) > TURRET_TOLERANCE_TICKS) {
//            turretMotor.setPower(TURRET_POWER_DEFAULT);
//        } else {
//            turretMotor.setPower(TURRET_POWER_HOLD);
//        }
//    }
//
//    //--------------------------- MANUAL/UTILITY METHODS -------------------------------------------
//
//    // NOTE: These methods use the same hardware names, but are part of the new class.
//
//    public void setShooterPower(boolean triggerHeld) {
//        shooterController.setShooterPower(triggerHeld);
//    }
//
//    public void resetTurret() {
//        turretMotor.setTargetPosition(0);
//        turretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        turretMotor.setPower(TURRET_POWER_DEFAULT);
//        lastTurretTargetPosition = 0;
//    }
//
//    public void setTurretManualPosition(int targetTicks) {
//        if (targetTicks != lastTurretTargetPosition) {
//            turretMotor.setTargetPosition(targetTicks);
//            lastTurretTargetPosition = targetTicks;
//            turretMotor.setPower(TURRET_POWER_DEFAULT);
//        }
//    }
//
//    // --- Accessors ---
//    public int getTurretTicks() { return turretMotor.getCurrentPosition(); }
//    public double getTurretAngleDegrees() {
//        return turretMotor.getCurrentPosition() * DEGREE_TO_TICK_RATIO;
//    }
//    public boolean isGoalInRange() {
//        return turretController.isGoalInRange(follower.getPose());
//    }
//    public String getTurretDebugInfo() {
//        return turretController.getDebugInfo(follower.getPose());
//    }
//    public double getShooterDistanceToGoal() { return shooterController.getDistanceToGoal(); }
//    public double getCommandedShooterPower() { return shooterController.getCommandedPower(); }
//}