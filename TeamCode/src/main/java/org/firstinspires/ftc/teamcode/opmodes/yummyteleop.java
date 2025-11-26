package org.firstinspires.ftc.teamcode.opmodes;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
// --- NEW IMPORT: Added for the 10-second timer ---
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Indexer;
import org.firstinspires.ftc.teamcode.RobotFunctions;
import org.firstinspires.ftc.teamcode.Spindexer;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@TeleOp(name = "Yummy TeleOp - Field Turret", group = "Drive")
public class yummyteleop extends OpMode {

    // Drive / pose
    private Follower follower;

    // Intake / shooter / spinner / indexer / spindexer
    private DcMotor shooter;
    private DcMotor spinner;
    private CRServo LeftServo, RightServo;
    private final Spindexer spindexer = new Spindexer();
    private final Indexer indexer = new Indexer();

    // Shooter toggle state (copied from original Teleop)
    boolean pressedTriangle, pressedSquare, pressedCross, shooterOn = false;
    private double currentShooterPower = 0.0;

    // Turret via RobotFunctions (uses PedroPathing pose internally)
    private RobotFunctions robotFunctions;
    private boolean turretAutoAimEnabled = true;
    private boolean prevG2LeftBumper = false;

    // --- NEW: Timer for 10-second auto-reset ---
    private ElapsedTime turretResetTimer = new ElapsedTime();

    // --- NEW: Define the goal position here in the TeleOp ---
    // Assuming the Blue Alliance High Goal (6 inches from the left and 6 inches from the far end)
    private final Pose HIGH_GOAL_TARGET_POSE = new Pose(6, 144 - 6, 0);

    // Starting pose on the field (adjust coordinates/heading as needed)
    private final Pose startingPose = new Pose(40, 9, Math.toRadians(90));

    @Override
    public void init() {
        // Initialize follower and field pose
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startingPose);
        follower.update();

        // Hardware
        spinner = hardwareMap.get(DcMotor.class, "motor2");

        LeftServo = hardwareMap.get(CRServo.class, "LeftServo");
        RightServo = hardwareMap.get(CRServo.class, "RightServo");

        // Spinner hold (keep your original behavior)
        spinner.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        spinner.setTargetPosition(0);
        spinner.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        spinner.setPower(0.3);
        spinner.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Shooter
        shooter = hardwareMap.get(DcMotor.class, "shooter");
        shooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        // Indexer / spindexer
        spindexer.freshInit(hardwareMap);
        indexer.Init(hardwareMap, telemetry);

        // RobotFunctions manages turret/shooter higher-level functions and uses follower for field coordinates
        robotFunctions = new RobotFunctions();
        robotFunctions.init(hardwareMap, follower);

        // Pass the target pose to the RobotFunctions class
        // NOTE: This assumes you have a public setTargetPose(Pose target) method in RobotFunctions.java
        robotFunctions.setTargetPose(HIGH_GOAL_TARGET_POSE);
        telemetry.addData("Goal Target", "X: %.1f, Y: %.1f", HIGH_GOAL_TARGET_POSE.getX(), HIGH_GOAL_TARGET_POSE.getY());

        // --- NEW: Start the reset timer when the OpMode initializes ---
        turretResetTimer.reset();
    }

    @Override
    public void start() {
        follower.startTeleopDrive();
    }

    @Override
    public void loop() {
        // Update pose & subsystems that rely on pose
        follower.update();

        // --- NEW: 10-Second Auto-Reset Logic ---
        if (turretResetTimer.seconds() >= 10.0) {
            // Force reset the turret to 0 ticks (assuming 0 is the home/centered position)
            turretAutoAimEnabled = false; // Temporarily disable auto-aim to enforce the reset
            robotFunctions.setTurretManualPosition(0); // Send command for 0 ticks
            turretResetTimer.reset(); // Restart the timer
            telemetry.addLine("!!! Turret 10s Auto-Reset Triggered (0 Ticks) !!!");
        }

        // Drive (use pedropathing teleop drive so position is tracked)
        follower.setTeleOpDrive(
                -gamepad1.left_stick_y,
                -gamepad1.left_stick_x,
                -gamepad1.right_stick_x,
                true // robot-centric
        );

        // Intake controls (original behaviour)
        if (gamepad1.right_trigger > 0.1) {
            Intake(1);
        } else if (gamepad1.dpad_up) {
            Intake(-1);
        } else {
            Intake(0);
        }

        // Spinner telemetry
        telemetry.addData("Spinner Pos", spinner.getCurrentPosition());
        telemetry.addData("Spinner Target", spinner.getTargetPosition());
        telemetry.addData("Spinner Busy", spinner.isBusy());

        // Shooter toggles (keep original button mapping and logic)
        // Triangle button -> toggles -0.6
        if (gamepad1.square) {
            if (!pressedTriangle) {
                pressedTriangle = true;
                if (shooterOn && currentShooterPower == -0.6) {
                    shooter.setPower(0);
                    shooterOn = false;
                    currentShooterPower = 0.0;
                } else {
                    shooter.setPower(-0.6);
                    shooterOn = true;
                    currentShooterPower = -0.6;
                }
            }
        } else pressedTriangle = false;

        // Square button -> toggles -0.5
        if (gamepad1.triangle) {
            if (!pressedSquare) {
                pressedSquare = true;
                if (shooterOn && currentShooterPower == -0.5) {
                    shooter.setPower(0);
                    shooterOn = false;
                    currentShooterPower = 0.0;
                } else {
                    shooter.setPower(-0.5);
                    shooterOn = true;
                    currentShooterPower = -0.5;
                }
            }
        } else pressedSquare = false;

        // Cross button -> toggles -0.75
        if (gamepad1.cross) {
            if (!pressedCross) {
                pressedCross = true;
                if (shooterOn && currentShooterPower == -0.75) {
                    shooter.setPower(0);
                    shooterOn = false;
                    currentShooterPower = 0.0;
                } else {
                    shooter.setPower(-0.75);
                    shooterOn = true;
                    currentShooterPower = -0.75;
                }
            }
        } else pressedCross = false;

        // Indexer update & spindexer usage (same as original)
        indexer.Update(gamepad1.circle);
        if (gamepad1.left_bumper || gamepad1.right_bumper) {
            indexer.spindex(gamepad1.left_bumper, spindexer);
        }

        // Turret auto-aim toggle: edge-detect gamepad2.left_bumper
        boolean currentG2Left = gamepad2.left_bumper;
        if (currentG2Left && !prevG2LeftBumper) {
            turretAutoAimEnabled = !turretAutoAimEnabled;
        }
        prevG2LeftBumper = currentG2Left;

        // Turret handling
        if (turretAutoAimEnabled) {
            // RobotFunctions uses the target pose set in init() internally to compute aiming
            robotFunctions.updateTurret();
        } else {
            // Manual turret control with gamepad2 right stick (small deadzone)
            double manual = -gamepad2.right_stick_x;
            if (Math.abs(manual) > 0.08) {
                int currentTicks = robotFunctions.getTurretTicks();
                int delta = (int)(manual * 10); // tune this as needed
                robotFunctions.setTurretManualPosition(currentTicks + delta);
            }
            // Allow resetting to centered position
            if (gamepad2.dpad_down) robotFunctions.resetTurret();
        }

        // Telemetry: show field pose and turret/shooter info
        Pose pose = follower.getPose();
        telemetry.addData("Pose X", "%.1f", pose.getX());
        telemetry.addData("Pose Y", "%.1f", pose.getY());
        telemetry.addData("Pose H (deg)", "%.1f", Math.toDegrees(pose.getHeading()));
        telemetry.addData("Turret AutoAim", turretAutoAimEnabled ? "ON" : "OFF");
        telemetry.addData("Turret Ticks", robotFunctions.getTurretTicks());
        telemetry.addData("Turret Angle", "%.1f°", robotFunctions.getTurretAngleDegrees());
        telemetry.addData("Shooter Power", "%.2f", shooter.getPower());
        // --- NEW TELEMETRY: Show the timer countdown ---
        telemetry.addData("Reset Timer", "%.1f / 10.0s", turretResetTimer.seconds());


        telemetry.update();
    }

    public void Intake(double power) {
        LeftServo.setPower(power);
        RightServo.setPower(-power);
    }
}