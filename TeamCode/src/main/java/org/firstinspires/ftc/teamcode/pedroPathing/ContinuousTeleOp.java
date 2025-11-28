//package org.firstinspires.ftc.teamcode.pedroPathing;
//
//import com.bylazar.configurables.annotations.Configurable;
//import com.bylazar.telemetry.PanelsTelemetry;
//import com.bylazar.telemetry.TelemetryManager;
//import com.pedropathing.follower.Follower;
//import com.pedropathing.geometry.BezierLine;
//import com.pedropathing.geometry.Pose;
//import com.pedropathing.paths.HeadingInterpolator;
//import com.pedropathing.paths.Path;
//import com.pedropathing.paths.PathChain;
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import org.firstinspires.ftc.teamcode.RobotFunctions; // *** NEW CLASS IMPORT ***
//
//import java.util.function.Supplier;
//
//@Configurable
//@TeleOp(name = "CONTINUOUS TeleOp - Turret Only")
//public class ContinuousTeleOp extends OpMode { // *** NEW CLASS NAME ***
//
//    // --- CONFIGURABLE CONSTANTS ---
//    public static final double MANUAL_TURRET_TICK_RATE = 15.0;
//    public static final double MANUAL_TURRET_DEADZONE = 0.1;
//
//    // --- FOLLOW, DRIVETRAIN & POSE ---
//    private Follower follower;
//    public static Pose startingPose = new Pose(40, 9, Math.toRadians(90));
//    private Supplier<PathChain> pathChainSupplier;
//
//    // --- STATE MANAGEMENT ---
//    private ContinuousRobotFunctions robotFunctions; // *** NEW CLASS INSTANCE ***
//    private boolean isAutomatedDrive = false;
//    private boolean isTurretAutoAimEnabled = true; // State for continuous tracking
//
//    // Simplifed drive controls for this Turret-Only example
//    private final double DRIVE_MULTIPLIER = 1.0;
//
//    private TelemetryManager telemetryM;
//
//    //--------------------------- INIT -------------------------------------------------------------
//    @Override
//    public void init() {
//        follower = Constants.createFollower(hardwareMap);
//        follower.setStartingPose(startingPose);
//        follower.update();
//
//        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
//
//        robotFunctions = new ContinuousRobotFunctions(); // *** USE CONTINUOUS CLASS ***
//        robotFunctions.initTurret(hardwareMap, follower); // Initialize only the turret
//
//        pathChainSupplier = () -> follower.pathBuilder()
//                .addPath(new Path(new BezierLine(follower::getPose, new Pose(45, 98))))
//                .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading, Math.toRadians(45), 0.8))
//                .build();
//    }
//
//    //--------------------------- START ------------------------------------------------------------
//    @Override
//    public void start() {
//        follower.startTeleopDrive();
//    }
//
//    //--------------------------- LOOP -------------------------------------------------------------
//    @Override
//    public void loop() {
//        // 1. Core Updates
//        follower.update();
//        telemetryM.update();
//
//        // 2. Turret Auto-Aim Update (if enabled)
//        if (isTurretAutoAimEnabled) {
//            robotFunctions.updateTurret(); // Runs continuous tracking logic
//        }
//
//        // 3. GamePad Input Handling (Driver 1 - Movement)
//        handleDriverOneControls();
//
//        // 4. GamePad Input Handling (Driver 2 - Turret & Utility)
//        handleDriverTwoControls();
//
//        // 5. Telemetry Output
//        updateTelemetry();
//    }
//
//    // --- DRIVER 1: MOVEMENT & AUTONOMOUS CONTROLS ---
//    private void handleDriverOneControls() {
//
//        // --- 1. PATH FOLLOWING (Simplified) ---
//        if (gamepad1.aWasPressed()) {
//            follower.followPath(pathChainSupplier.get());
//            isAutomatedDrive = true;
//        }
//
//        if (isAutomatedDrive && (gamepad1.bWasPressed() || !follower.isBusy())) {
//            follower.startTeleopDrive();
//            isAutomatedDrive = false;
//        }
//
//        // --- 2. MANUAL DRIVE ---
//        if (!isAutomatedDrive) {
//            follower.setTeleOpDrive(
//                    -gamepad1.left_stick_y * DRIVE_MULTIPLIER, // Forward/Backward
//                    -gamepad1.left_stick_x * DRIVE_MULTIPLIER, // Strafe Left/Right
//                    -gamepad1.right_stick_x * DRIVE_MULTIPLIER, // Turn
//                    true // Robot Centric
//            );
//        }
//    }
//
//    // --- DRIVER 2: TURRET CONTROLS ---
//    private void handleDriverTwoControls() {
//
//        // Toggle Turret Auto-Aim (GamePad2 Left Bumper)
//        if (gamepad2.left_bumper) {
//            isTurretAutoAimEnabled = !isTurretAutoAimEnabled;
//        }
//
//        // Manual Turret Control (only when auto-aim is off)
//        if (!isTurretAutoAimEnabled) {
//            double manualTurnInput = -gamepad2.right_stick_x;
//
//            if (Math.abs(manualTurnInput) > MANUAL_TURRET_DEADZONE) {
//                int currentTicks = robotFunctions.getTurretTicks();
//                int tickChange = (int)(manualTurnInput * MANUAL_TURRET_TICK_RATE);
//                int newTicks = currentTicks + tickChange;
//
//                robotFunctions.setTurretManualPosition(newTicks);
//            }
//        }
//
//        // Reset Turret to Center (only when auto-aim is off)
//        if (gamepad2.dpad_down && !isTurretAutoAimEnabled) {
//            robotFunctions.resetTurret();
//        }
//    }
//
//    // --- TELEMETRY OUTPUT ---
//    private void updateTelemetry() {
//
//        // FTC Driver Station Turret Telemetry
//        telemetry.addData("--- TURRET STATUS (CONTINUOUS) ---", "--------------------");
//        telemetry.addData("Auto-Aim", isTurretAutoAimEnabled ? "ON (Auto Tracking)" : "OFF (Manual)");
//        telemetry.addData("Position (Ticks)", robotFunctions.getTurretTicks());
//        telemetry.addData("Angle (Degrees)", "%.1f°", robotFunctions.getTurretAngleDegrees());
//        telemetry.addData("Goal In Range", robotFunctions.isGoalInRange() ? "YES" : "NO");
//        telemetry.addData("Turret Debug", robotFunctions.getTurretDebugInfo());
//
//        // Drivetrain Telemetry (Simplified)
//        telemetry.addData("--- DRIVETRAIN STATUS ---", "--------------------");
//        telemetry.addData("Current Pose (Field)", "X: %.1f, Y: %.1f, Heading: %.1f°",
//                follower.getPose().getX(),
//                follower.getPose().getY(),
//                Math.toDegrees(follower.getPose().getHeading()));
//
//        telemetry.update();
//    }
//}