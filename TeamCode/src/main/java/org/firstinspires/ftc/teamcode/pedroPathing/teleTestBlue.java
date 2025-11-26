package org.firstinspires.ftc.teamcode.pedroPathing;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.RobotFunctions;

import java.util.function.Supplier;

@Configurable
@TeleOp(name = "TeleOp Test Drive - G1 Core")
public class teleTestBlue extends OpMode {

    // --- CONFIGURABLE CONSTANTS ---
    public static final double SLOW_MODE_MULTIPLIER_INCREMENT = 0.25;
    public static final double MANUAL_TURRET_TICK_RATE = 15.0;
    public static final double MANUAL_TURRET_DEADZONE = 0.1;
    public static final double DEFAULT_SLOW_MODE_MULTIPLIER = 0.5;
    public static final double SHOOTER_TRIGGER_THRESHOLD = 0.1;

    // --- FOLLOW, DRIVETRAIN & POSE ---
    private Follower follower;
    public static Pose startingPose = new Pose(40, 9, Math.toRadians(90));
    private Supplier<PathChain> pathChainSupplier;

    // --- STATE MANAGEMENT ---
    private RobotFunctions robotFunctions;
    private boolean isAutomatedDrive = false;
    private boolean isTurretAutoAimEnabled = true;
    private double slowModeMultiplier = DEFAULT_SLOW_MODE_MULTIPLIER;
    private boolean isSlowMode = false;

    private TelemetryManager telemetryM;

    //--------------------------- INIT -------------------------------------------------------------
    @Override
    public void init() {
        // Initialize Follower and set start pose
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startingPose);
        follower.update();

        // Initialize Telemetry Manager
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        // Initialize ALL Systems (Turret and Shooter)
        robotFunctions = new RobotFunctions();
        robotFunctions.init(hardwareMap, follower);

        // Define PathChain lazily.
        pathChainSupplier = () -> follower.pathBuilder()
                .addPath(new Path(new BezierLine(follower::getPose, new Pose(45, 98))))
                .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading, Math.toRadians(45), 0.8))
                .build();
    }

    //--------------------------- START ------------------------------------------------------------
    @Override
    public void start() {
        follower.startTeleopDrive();
    }

    //--------------------------- LOOP -------------------------------------------------------------
    @Override
    public void loop() {
        // 1. Core Updates
        follower.update();
        telemetryM.update();

        // 2. Turret Auto-Aim Update (if enabled)
        if (isTurretAutoAimEnabled) {
            robotFunctions.updateTurret();
        }

        // 3. GamePad Input Handling (Driver 1 - Core Movement & Firing)
        handleDriverOneControls();

        // 4. GamePad Input Handling (Driver 2 - Precision/Manual Turret)
        handleDriverTwoControls();

        // 5. Telemetry Output
        updateTelemetry();
    }

    // --- DRIVER 1: CORE MOVEMENT, AUTONOMOUS, SLOW MODE & SHOOTER ---
    private void handleDriverOneControls() {

        // --- 1. SHOOTER CONTROL (MOVED TO GAMPAD 1 - Right Trigger) ---
        boolean triggerHeld = gamepad1.right_trigger > SHOOTER_TRIGGER_THRESHOLD;
        robotFunctions.setShooterPower(triggerHeld);

        // --- 2. DRIVE CONTROL ---
        if (!isAutomatedDrive) {
            // SLOW MODE TOGGLE (Right Bumper)
            if (gamepad1.rightBumperWasPressed()) {
                isSlowMode = !isSlowMode;
            }

            double driveMultiplier = isSlowMode ? slowModeMultiplier : 1.0;

            // Drive Command
            follower.setTeleOpDrive(
                    -gamepad1.left_stick_y * driveMultiplier, // Forward/Backward
                    -gamepad1.left_stick_x * driveMultiplier, // Strafe Left/Right
                    -gamepad1.right_stick_x * driveMultiplier, // Turn
                    true // Robot Centric
            );
        }

        // --- 3. PATH FOLLOWING ---
        // Start Path Following (A Button)
        if (gamepad1.aWasPressed()) {
            follower.followPath(pathChainSupplier.get());
            isAutomatedDrive = true;
        }

        // Stop Automated Following (B Button or Path Done)
        if (isAutomatedDrive && (gamepad1.bWasPressed() || !follower.isBusy())) {
            follower.startTeleopDrive();
            isAutomatedDrive = false;
        }

        // --- 4. SLOW MODE ADJUSTMENTS ---
        // Increase/Decrease Slow Mode Multiplier
        if (gamepad1.xWasPressed()) {
            slowModeMultiplier = Math.min(1.0, slowModeMultiplier + SLOW_MODE_MULTIPLIER_INCREMENT);
        }
        if (gamepad1.yWasPressed()) {
            slowModeMultiplier = Math.max(0.1, slowModeMultiplier - SLOW_MODE_MULTIPLIER_INCREMENT);
        }
    }

    // --- DRIVER 2: MANUAL TURRET & AUTO-AIM TOGGLE ---
    private void handleDriverTwoControls() {
        // --- 1. AUTO-AIM TOGGLE (Left Bumper) ---
        if (gamepad2.left_bumper) {
            isTurretAutoAimEnabled = !isTurretAutoAimEnabled;
        }

        // --- 2. MANUAL TURRET CONTROL ---
        if (!isTurretAutoAimEnabled) {
            // Manual Turret Control (Right Stick X-axis)
            double manualTurnInput = -gamepad2.right_stick_x;

            if (Math.abs(manualTurnInput) > MANUAL_TURRET_DEADZONE) {
                int currentTicks = robotFunctions.getTurretTicks();
                int tickChange = (int)(manualTurnInput * MANUAL_TURRET_TICK_RATE);
                int newTicks = currentTicks + tickChange;

                robotFunctions.setTurretManualPosition(newTicks);
            }
        }

        // --- 3. MANUAL TURRET RESET (D-pad Down) ---
        if (gamepad2.dpad_down && !isTurretAutoAimEnabled) {
            robotFunctions.resetTurret();
        }

        // --- 4. (Optional) Slow Mode Adjustments (D-pad Up/Down) ---
        if (gamepad2.dpad_up) {
            slowModeMultiplier = Math.min(1.0, slowModeMultiplier + SLOW_MODE_MULTIPLIER_INCREMENT);
        }
        // Allows G2 to decrease the multiplier, but not conflict with the Turret Reset on D-pad Down
        if (gamepad2.dpad_down && isTurretAutoAimEnabled) {
            slowModeMultiplier = Math.max(0.1, slowModeMultiplier - SLOW_MODE_MULTIPLIER_INCREMENT);
        }
    }

    // --- TELEMETRY OUTPUT (UNMODIFIED) ---
    private void updateTelemetry() {
        // Pedro Pathing Telemetry Panel (debug)
        telemetryM.debug("Robot Pose (X, Y, H)", follower.getPose().toString());
        telemetryM.debug("Robot Velocity", follower.getVelocity().toString());
        telemetryM.debug("Automated Drive", isAutomatedDrive);

        // FTC Driver Station Telemetry
        telemetry.addData("--- TURRET STATUS ---", "--------------------");
        telemetry.addData("Auto-Aim", isTurretAutoAimEnabled ? "ON (Auto)" : "OFF (Manual)");
        telemetry.addData("Position (Ticks)", robotFunctions.getTurretTicks());
        telemetry.addData("Angle (Degrees)", "%.1f°", robotFunctions.getTurretAngleDegrees());
        telemetry.addData("Goal In Range", robotFunctions.isGoalInRange() ? "YES" : "NO");
        telemetry.addData("Turret Debug", robotFunctions.getTurretDebugInfo());

        // --- SHOOTER STATUS ---
        telemetry.addData("--- SHOOTER STATUS ---", "--------------------");
        telemetry.addData("Distance to Goal", "%.1f in", robotFunctions.getShooterDistanceToGoal());
        telemetry.addData("Commanded Power", "%.2f", robotFunctions.getCommandedShooterPower());

        // --- DRIVETRAIN STATUS ---
        telemetry.addData("--- DRIVETRAIN STATUS ---", "--------------------");
        telemetry.addData("Current Pose (Field)", "X: %.1f, Y: %.1f, Heading: %.1f°",
                follower.getPose().getX(),
                follower.getPose().getY(),
                Math.toDegrees(follower.getPose().getHeading()));
        telemetry.addData("Slow Mode", isSlowMode ? "ON" : "OFF");
        telemetry.addData("Slow Multiplier", "%.2f", slowModeMultiplier);

        telemetry.update();
    }
}