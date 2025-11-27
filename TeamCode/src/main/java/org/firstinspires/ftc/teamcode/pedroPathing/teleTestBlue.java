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
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode.RobotFunctions;
import org.firstinspires.ftc.teamcode.Indexer;
import org.firstinspires.ftc.teamcode.Spindexer;

import java.util.function.Supplier;

@Configurable
@TeleOp(name = "TeleOp Test Drive - G1 Core", group = "Drive")
public class teleTestBlue extends OpMode {

    // --- CONFIGURABLE CONSTANTS ---
    public static final double SLOW_MODE_MULTIPLIER_INCREMENT = 0.25;
    public static final double DEFAULT_SLOW_MODE_MULTIPLIER = 0.5;
    public static final double SHOOTER_TRIGGER_THRESHOLD = 0.1;
    public static final double INTAKE_DEADZONE = 0.05;

    // --- FOLLOW, DRIVETRAIN & POSE ---
    private Follower follower;
    public static Pose startingPose = new Pose(40, 9, Math.toRadians(90));
    private Supplier<PathChain> pathChainSupplier;

    // --- STATE MANAGEMENT ---
    private RobotFunctions robotFunctions;
    private boolean isAutomatedDrive = false;
    private double slowModeMultiplier = DEFAULT_SLOW_MODE_MULTIPLIER;
    private boolean isSlowMode = false;

    // --- INTAKE, SPINDEXER & INDEXER ---
    private CRServo LeftServo, RightServo;
    private DcMotor spinner;
    private final Spindexer spindexer = new Spindexer();
    private final Indexer indexer = new Indexer();
    private boolean spindexerDirection = true;

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
        robotFunctions.resetTurret();

        // --- SPINNER SETUP - Constant Hold at Position 0 ---
        spinner = hardwareMap.get(DcMotor.class, "motor2");
        spinner.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        spinner.setTargetPosition(0);
        spinner.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        spinner.setPower(0.3); // Holding power to maintain position
        spinner.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // --- INTAKE SETUP ---
        LeftServo = hardwareMap.get(CRServo.class, "LeftServo");
        RightServo = hardwareMap.get(CRServo.class, "RightServo");

        // --- SPINDEXER SETUP ---
        spindexer.freshInit(hardwareMap);

        // --- INDEXER SETUP ---
        indexer.Init(hardwareMap, telemetry);

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

        // 2. GamePad Input Handling (Driver 1 - Core Movement & Systems)
        handleDriverOneControls();

        // 3. Indexer Update (needs to run every loop)
        indexer.Update(gamepad1.circle);

        // 4. Spindexer Update (needs to run every loop)
        indexer.spindexerUpdate(spindexerDirection, spindexer);

        // 5. Telemetry Output
        updateTelemetry();
    }

    // --- DRIVER 1: CORE MOVEMENT, AUTONOMOUS, SLOW MODE & ALL SYSTEMS ---
    private void handleDriverOneControls() {

        // --- 1. SHOOTER CONTROL (Left Trigger) ---
        boolean shooterOn = gamepad1.left_trigger > SHOOTER_TRIGGER_THRESHOLD;
        robotFunctions.setShooterPower(shooterOn);

        // --- 2. DRIVE CONTROL ---
        if (!isAutomatedDrive) {
            // SLOW MODE TOGGLE (Right Bumper)
            if (gamepad1.rightBumperWasPressed()) {
                isSlowMode = !isSlowMode;
            }

            double driveMultiplier = isSlowMode ? slowModeMultiplier : 1.0;

            // Drive Command using Pedro Pathing system
            follower.setTeleOpDrive(
                    -gamepad1.left_stick_y * driveMultiplier, // Forward/Backward
                    -gamepad1.left_stick_x * driveMultiplier, // Strafe Left/Right
                    -gamepad1.right_stick_x * driveMultiplier, // Turn
                    true // Robot Centric
            );
        }

        // --- 3. INTAKE CONTROL ---
        // Right Trigger for intake
        double intakePower = gamepad1.right_trigger > INTAKE_DEADZONE ? -1.0 : 0;

        // Dpad Up for reverse intake
        if (gamepad1.dpad_up) {
            intakePower = 1.0; // Reverse intake
        }

        Intake(intakePower);

        // --- 4. SPINDEXER CONTROL (Bumpers) ---
        if (gamepad1.left_bumper || gamepad1.right_bumper) {
            spindexerDirection = gamepad1.dpad_left;
            indexer.spindex(gamepad1.left_bumper, spindexer);
        }

        // --- 5. AUTO DRIVE CONTROL (X Button) ---
        if (gamepad1.xWasPressed()) {
            follower.followPath(pathChainSupplier.get());
            isAutomatedDrive = true;
        }

        // Stop Automated Following (Y Button or Path Done)
        if (isAutomatedDrive && (gamepad1.yWasPressed() || !follower.isBusy())) {
            follower.startTeleopDrive();
            isAutomatedDrive = false;
        }

        // --- 6. SLOW MODE ADJUSTMENTS (A and B Buttons) ---
        if (gamepad1.aWasPressed()) {
            slowModeMultiplier = Math.min(1.0, slowModeMultiplier + SLOW_MODE_MULTIPLIER_INCREMENT);
        }
        if (gamepad1.bWasPressed()) {
            slowModeMultiplier = Math.max(0.1, slowModeMultiplier - SLOW_MODE_MULTIPLIER_INCREMENT);
        }
    }

    // --- INTAKE CONTROL ---
    public void Intake(double power) {
        LeftServo.setPower(power);
        RightServo.setPower(-power);
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
        telemetry.addData("Shooter Active", gamepad1.left_trigger > SHOOTER_TRIGGER_THRESHOLD ? "YES" : "NO");

        // --- DRIVETRAIN STATUS ---
        telemetry.addData("--- DRIVETRAIN STATUS ---", "--------------------");
        telemetry.addData("Current Pose (Field)", "X: %.1f, Y: %.1f, Heading: %.1f°",
                follower.getPose().getX(),
                follower.getPose().getY(),
                Math.toDegrees(follower.getPose().getHeading()));
        telemetry.addData("Slow Mode", isSlowMode ? "ON" : "OFF");
        telemetry.addData("Slow Multiplier", "%.2f", slowModeMultiplier);

        // --- INTAKE & SPINDEXER STATUS ---
        telemetry.addData("--- INTAKE & SPINDEXER ---", "--------------------");
        telemetry.addData("Intake Power", "%.2f", LeftServo.getPower());
        telemetry.addData("Spinner Position", spinner.getCurrentPosition());
        telemetry.addData("Is Spindexer busy?", spindexer.spindexer.isBusy());

        // --- CONTROLS INFO ---
        telemetry.addData("--- CONTROLS ---", "--------------------");
        telemetry.addData("Drive", "Left Stick: Move, Right Stick: Rotate");
        telemetry.addData("Shooter", "Left Trigger: Shoot");
        telemetry.addData("Intake", "Right Trigger: Run, Dpad Up: Reverse");
        telemetry.addData("Spindexer", "Bumpers: Activate");
        telemetry.addData("Auto Drive", "X: Start Path, Y: Cancel");
        telemetry.addData("Slow Mode", "Right Bumper: Toggle, A/B: Adjust");

        telemetry.update();
    }
}