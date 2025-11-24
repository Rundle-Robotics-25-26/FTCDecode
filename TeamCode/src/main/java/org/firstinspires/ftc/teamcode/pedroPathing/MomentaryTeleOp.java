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
import org.firstinspires.ftc.teamcode.MomentaryRobotFunctions; // *** NEW CLASS IMPORT ***

import java.util.function.Supplier;

@Configurable
@TeleOp(name = "MOMENTARY TeleOp - G1 Core")
public class MomentaryTeleOp extends OpMode { // *** NEW CLASS NAME ***

    // --- CONFIGURABLE CONSTANTS ---
    public static final double SLOW_MODE_MULTIPLIER_INCREMENT = 0.25;
    public static final double MANUAL_TURRET_TICK_RATE = 15.0;
    public static final double MANUAL_TURRET_DEADZONE = 0.1;
    public static final double DEFAULT_SLOW_MODE_MULTIPLIER = 0.5;
    public static final double TRIGGER_THRESHOLD = 0.1;

    // --- FOLLOW, DRIVETRAIN & POSE ---
    private Follower follower;
    public static Pose startingPose = new Pose(40, 9, Math.toRadians(90));
    private Supplier<PathChain> pathChainSupplier;

    // --- STATE MANAGEMENT ---
    private MomentaryRobotFunctions robotFunctions; // *** NEW CLASS INSTANCE ***
    private boolean isAutomatedDrive = false;
    private double slowModeMultiplier = DEFAULT_SLOW_MODE_MULTIPLIER;
    private boolean isSlowMode = false;

    private TelemetryManager telemetryM;

    //--------------------------- INIT -------------------------------------------------------------
    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startingPose);
        follower.update();

        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        robotFunctions = new MomentaryRobotFunctions(); // *** USE MOMENTARY CLASS ***
        robotFunctions.init(hardwareMap, follower);

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

        // Check the primary trigger for the turret state
        boolean aimCommanded = gamepad1.left_trigger > TRIGGER_THRESHOLD;

        // 2. Turret Update: Call the new Momentary Logic
        robotFunctions.updateTurret(aimCommanded);

        // 3. GamePad Input Handling
        handleDriverOneControls();
        handleDriverTwoControls(aimCommanded); // Pass state to G2

        // 4. Telemetry Output
        updateTelemetry();
    }

    // --- DRIVER 1: CORE MOVEMENT, AUTONOMOUS, SLOW MODE & SHOOTER ---
    private void handleDriverOneControls() {
        // --- 1. SHOOTER CONTROL (GamePad 1 - Right Trigger) ---
        boolean shooterCommanded = gamepad1.right_trigger > TRIGGER_THRESHOLD;
        robotFunctions.setShooterPower(shooterCommanded);

        // --- 2. DRIVE CONTROL ---
        if (!isAutomatedDrive) {
            if (gamepad1.rightBumperWasPressed()) {
                isSlowMode = !isSlowMode;
            }

            double driveMultiplier = isSlowMode ? slowModeMultiplier : 1.0;

            follower.setTeleOpDrive(
                    -gamepad1.left_stick_y * driveMultiplier,
                    -gamepad1.left_stick_x * driveMultiplier,
                    -gamepad1.right_stick_x * driveMultiplier,
                    true
            );
        }

        // --- 3. PATH FOLLOWING (Unmodified) ---
        if (gamepad1.aWasPressed()) {
            follower.followPath(pathChainSupplier.get());
            isAutomatedDrive = true;
        }

        if (isAutomatedDrive && (gamepad1.bWasPressed() || !follower.isBusy())) {
            follower.startTeleopDrive();
            isAutomatedDrive = false;
        }

        // --- 4. SLOW MODE ADJUSTMENTS (Unmodified) ---
        if (gamepad1.xWasPressed()) {
            slowModeMultiplier = Math.min(1.0, slowModeMultiplier + SLOW_MODE_MULTIPLIER_INCREMENT);
        }
        if (gamepad1.yWasPressed()) {
            slowModeMultiplier = Math.max(0.1, slowModeMultiplier - SLOW_MODE_MULTIPLIER_INCREMENT);
        }
    }

    // --- DRIVER 2: MANUAL TURRET & UTILITY CONTROLS ---
    private void handleDriverTwoControls(boolean isG1Aiming) {

        // --- 1. MANUAL TURRET CONTROL (Only works when G1 is NOT aiming, i.e., turret is at 0) ---
        if (!isG1Aiming) {
            double manualTurnInput = -gamepad2.right_stick_x;

            if (Math.abs(manualTurnInput) > MANUAL_TURRET_DEADZONE) {
                int currentTicks = robotFunctions.getTurretTicks();
                int tickChange = (int)(manualTurnInput * MANUAL_TURRET_TICK_RATE);
                int newTicks = currentTicks + tickChange;

                robotFunctions.setTurretManualPosition(newTicks);
            }
        }

        // --- 2. MANUAL TURRET RESET (If G1 is not aiming, D-pad down enforces 0 ticks) ---
        if (gamepad2.dpad_down && !isG1Aiming) {
            robotFunctions.resetTurret();
        }

        // --- 3. SLOW MODE ADJUSTMENTS (Unmodified) ---
        if (gamepad2.dpad_up) {
            slowModeMultiplier = Math.min(1.0, slowModeMultiplier + SLOW_MODE_MULTIPLIER_INCREMENT);
        }
        if (gamepad2.dpad_down && isG1Aiming) {
            slowModeMultiplier = Math.max(0.1, slowModeMultiplier - SLOW_MODE_MULTIPLIER_INCREMENT);
        }
    }

    // --- TELEMETRY OUTPUT ---
    private void updateTelemetry() {
        // Pedro Pathing Telemetry Panel (debug)
        telemetryM.debug("Robot Pose (X, Y, H)", follower.getPose().toString());
        telemetryM.debug("Robot Velocity", follower.getVelocity().toString());
        telemetryM.debug("Automated Drive", isAutomatedDrive);

        // FTC Driver Station Telemetry
        telemetry.addData("--- TURRET STATUS (MOMENTARY) ---", "--------------------");
        telemetry.addData("Control Mode", robotFunctions.getTurretTicks() != 0 ? "AIMING / MANUAL" : "IDLE (0 Ticks)");
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