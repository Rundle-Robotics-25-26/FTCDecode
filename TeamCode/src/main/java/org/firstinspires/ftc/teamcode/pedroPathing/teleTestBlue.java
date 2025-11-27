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
    public static final double DEFAULT_SLOW_MODE_MULTIPLIER = 0.5;
    public static final double SHOOTER_TRIGGER_THRESHOLD = 0.1;

    // --- FOLLOW, DRIVETRAIN & POSE ---
    private Follower follower;
    public static Pose startingPose = new Pose(40, 9, Math.toRadians(90));
    private Supplier<PathChain> pathChainSupplier;

    // --- STATE MANAGEMENT ---
    private RobotFunctions robotFunctions;
    private boolean isAutomatedDrive = false;
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

        // --- TURRET MODIFICATION: Set and hold position to 0 ticks ---
        // This command will move the turret to the center (0 ticks) and keep the motor
        // active to hold that position throughout the OpMode.
        robotFunctions.resetTurret();

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

        // 2. GamePad Input Handling (Driver 1 - Core Movement & Firing)
        handleDriverOneControls();

        // Removed: Driver 2 Turret controls (since turret is now locked)
        // Removed: Turret Auto-Aim Update

        // 3. Telemetry Output
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
    // Removed all content related to manual turret control.
    private void handleDriverTwoControls() {
        // No turret logic here. D2 controls are effectively disabled.
        // The turret is held at 0 ticks by the resetTurret() call in init().
    }

    // --- TELEMETRY OUTPUT (CLEANED UP) ---
    private void updateTelemetry() {
        // Pedro Pathing Telemetry Panel (debug)
        telemetryM.debug("Robot Pose (X, Y, H)", follower.getPose().toString());
        telemetryM.debug("Robot Velocity", follower.getVelocity().toString());
        telemetryM.debug("Automated Drive", isAutomatedDrive);

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