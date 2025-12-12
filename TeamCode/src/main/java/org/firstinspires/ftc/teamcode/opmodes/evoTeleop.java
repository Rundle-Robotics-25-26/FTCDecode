package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.pedropathing.follower.Follower; // { changed code }
import com.pedropathing.geometry.Pose;     // { changed code }
import org.firstinspires.ftc.teamcode.pedroPathing.Constants; // { changed code }

import org.firstinspires.ftc.teamcode.Indexer;
//import org.firstinspires.ftc.teamcode.SimpleTurret;
import org.firstinspires.ftc.teamcode.SimpleTurret;
import org.firstinspires.ftc.teamcode.Spindexer;

public class evoTeleop  {
/*
    // Motor declarations
    private DcMotor frontLeft, frontRight, backLeft, backRight, spinner;
    // change shooter type to DcMotorEx to access velocity
    private DcMotorEx shooter;
    CRServo LeftServo, RightServo;

    boolean pressedTriangle, pressedSquare, pressedCross, shooterOn = false;
    // left-trigger toggle state (treat trigger as button; threshold avoids accidental small presses)
    private boolean leftTriggerPrev = false;
    private final double SHOOT_TRIGGER_THRESHOLD = 0.20;

    private double currentShooterPower = 0.0;

    // PID controller state for shooter velocity
    private double shooterTargetVel = 0.0; // target velocity (absolute units from getVelocity())
    private double shooter_kP = 0.0006; // initial P (tune on robot)
    private double shooter_kI = 0.000001; // initial I (small)
    private double shooter_kD = 0.00005; // initial D
    private double shooter_kF = 0.0002; // feedforward gain (tune)
    private double shooterIntegral = 0.0;
    private double shooterLastError = 0.0;
    private long shooterLastTimeNs = 0L;
    private final double INTEGRAL_MAX = 0.5; // clamp integral contribution (tune)
    private final double MAX_POWER = 1.0; // clamp final power
    private final double MIN_POWER = -1.0;

    // Pedro follower (used to get current robot pose for distance -> shooter mapping)
    private Follower follower = null;
    // Goal coordinates on the field (tune these values for your field / target)
    private final double GOAL_X = 72.0; // example X (in same units as follower pose)
    private final double GOAL_Y = 36.0; // example Y
    // Mapping constants: distance range maps linearly to velocity range
    private final double DISTANCE_MIN = 10.0; // below this distance use minVel
    private final double DISTANCE_MAX = 100.0; // above this distance use maxVel
    private final double SHOOTER_MIN_VEL = 2500.0; // ticks/sec or your encoder velocity units
    private final double SHOOTER_MAX_VEL = 4500.0;

    private final Spindexer spindexer = new Spindexer();
    private final SimpleTurret simpleTurret = new SimpleTurret();

    private final Indexer indexer = new Indexer();
    boolean spindexerDirection = true;

    @Override
    public void init() {
        // create follower so we can read robot pose and compute distance to goal
        try {
            follower = Constants.createFollower(hardwareMap);
        } catch (Exception e) {
            follower = null; // follower not available - keep graceful fallback
        }
        // ==== Mecanum Drive Setup ====
        frontLeft = hardwareMap.get(DcMotor.class, "lf");
        backLeft = hardwareMap.get(DcMotor.class, "lr");
        frontRight = hardwareMap.get(DcMotor.class, "rf");
        backRight = hardwareMap.get(DcMotor.class, "rr");

        spinner = hardwareMap.get(DcMotor.class, "motor2");

        // Set motor directions (adjust based on your robot's configuration)
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.FORWARD);

        // Set zero power behavior
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // ==== Spinner Setup - Constant Hold at Position 0 ====
        spinner.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        spinner.setTargetPosition(0);
        spinner.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        spinner.setPower(0.3); // Holding power to maintain position
        spinner.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // ==== Intake setup ====
        LeftServo = hardwareMap.get(CRServo.class, "LeftServo");
        RightServo = hardwareMap.get(CRServo.class, "RightServo");

        // ==== Spindexer setup ====
        spindexer.freshInit(hardwareMap);

        // ==== Simple turret setup ====
        //simpleTurret.Init(hardwareMap);

        // ==== Shooter setup ====
        // Use DcMotorEx so we can read velocity for PID control
        shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER); // use encoder for velocity measurements
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT); // allow coast if desired
        shooterLastTimeNs = System.nanoTime(); // initialize PID timing

        // ==== Indexer setup ====
        //indexer.Init(hardwareMap, telemetry);
    }

    @Override
    public void loop() {
        // ==== Mecanum Drive ====
        double drive = -gamepad1.left_stick_y;  // Reverse Y axis//we made it separate for 2 controllers
        double strafe = gamepad1.left_stick_x;
        double rotate = gamepad1.right_stick_x;
        driveMecanum(drive, strafe, rotate);

        // ==== Intake ====
        // Use trigger value for proportional control, keep small deadzone
        double rt = gamepad1.right_trigger;
        if (rt > 0.05) {
            // change: always run intake at full speed when trigger is pressed at all
            Intake(-1.0); // preserve original negative direction used previously
        } else if (gamepad1.dpad_up) {
            // manual full-speed reverse/intake
            Intake(1);
        } else {
            Intake(0);  // Stop
        }

        // ==== Spinner Status ====
        telemetry.addData("Spinner Position", spinner.getCurrentPosition());
        telemetry.addData("Spinner Target", spinner.getTargetPosition());
        telemetry.addData("Spinner Power", spinner.getPower());
        telemetry.addData("Spinner Busy", spinner.isBusy());

        // ==== Simple turret ====
        //simpleTurret.TurretControl(gamepad1.left_trigger, gamepad1.right_trigger);
        //telemetry.addData("Current turret position: ", simpleTurret.turret.getCurrentPosition());

        // ==== Shooter ====
        // Single toggle (Left Trigger) to enable/disable shooter.
        boolean ltPressed = gamepad1.left_trigger > SHOOT_TRIGGER_THRESHOLD;
        if (ltPressed && !leftTriggerPrev) {
            shooterOn = !shooterOn;
            if (!shooterOn) shooterTargetVel = 0.0;
        }
        leftTriggerPrev = ltPressed;

        // Compute automatic target velocity from follower pose when shooter is enabled
        if (shooterOn && follower != null) {
            try {
                Pose robotPose = follower.getPose();
                double dx = GOAL_X - robotPose.getX();
                double dy = GOAL_Y - robotPose.getY();
                double dist = Math.hypot(dx, dy);

                // linear mapping from distance to shooter velocity
                double t = (dist - DISTANCE_MIN) / (DISTANCE_MAX - DISTANCE_MIN);
                if (t < 0) t = 0;
                if (t > 1) t = 1;
                shooterTargetVel = SHOOTER_MIN_VEL + t * (SHOOTER_MAX_VEL - SHOOTER_MIN_VEL);
                telemetry.addData("Shooter dist->vel map t", "%.3f", t);
                telemetry.addData("DistanceToGoal", "%.2f", dist);
            } catch (Exception e) {
                // follower not ready, fallback to a safe default
                shooterTargetVel = SHOOTER_MIN_VEL;
            }
        }

        // Update PID loop for shooter each cycle
        updateShooterPID();

        // ==== Indexer ====
        indexer.Update(gamepad1.circle);

        // ==== Spindexer ====
        telemetry.addData("Is Spindexer busy? ", spindexer.spindexer.isBusy());
        if (gamepad1.left_bumper || gamepad1.right_bumper) {
            spindexerDirection = gamepad1.left_bumper;
            indexer.spindex(gamepad1.left_bumper, spindexer);
        }
        indexer.spindexerUpdate(spindexerDirection, spindexer);

        telemetry.addData("Intake RT", "%.2f", rt);
        telemetry.addData("LeftServo Power", LeftServo.getPower());
        telemetry.addData("RightServo Power", RightServo.getPower());
        telemetry.update();
    }

    // PID update method for shooter
    private void updateShooterPID() {
        double currentVel = Math.abs(shooter.getVelocity()); // units: ticks/sec (absolute)
        long now = System.nanoTime();
        double dt = (now - shooterLastTimeNs) / 1e9; // seconds
        if (dt <= 0) dt = 1e-3;

        double error = shooterTargetVel - currentVel;
        shooterIntegral += error * dt;
        // clamp integral to avoid windup
        shooterIntegral = Math.max(-INTEGRAL_MAX, Math.min(INTEGRAL_MAX, shooterIntegral));

        double derivative = (error - shooterLastError) / dt;

        // PID + simple feedforward based on desired velocity
        double output = shooter_kP * error + shooter_kI * shooterIntegral + shooter_kD * derivative;
        double feedforward = shooter_kF * shooterTargetVel;
        double power = feedforward + output;

        // clamp and apply sign (original code used negative power to spin shooter)
        power = Math.max(-MAX_POWER, Math.min(MAX_POWER, power));
        if (shooterTargetVel == 0.0) {
            shooter.setPower(0.0);
        } else {
            shooter.setPower(-Math.abs(power)); // keep direction consistent with previous negative powers
        }

        // save for next iteration
        shooterLastError = error;
        shooterLastTimeNs = now;

        // telemetry for tuning
        telemetry.addData("Shooter targetVel", "%.0f", shooterTargetVel);
        telemetry.addData("Shooter curVel", "%.0f", currentVel);
        telemetry.addData("Shooter power(out)", "%.3f", power);
    }

    private void driveMecanum(double drive, double strafe, double rotate) {
        // Calculate motor powers using mecanum equations
        double frontLeftPower = drive + strafe + rotate;
        double frontRightPower = drive - strafe - rotate;
        double backLeftPower = drive - strafe + rotate;
        double backRightPower = drive + strafe - rotate;

        // Normalize powers to maintain ratio but not exceed 1.0
        double maxPower = Math.max(Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower)),
                Math.max(Math.abs(backLeftPower), Math.abs(backRightPower)));

        if (maxPower > 1.0) {
            frontLeftPower /= maxPower;
            frontRightPower /= maxPower;
            backLeftPower /= maxPower;
            backRightPower /= maxPower;
        }

        // Set motor powers
        frontLeft.setPower(frontLeftPower);
        frontRight.setPower(frontRightPower);
        backLeft.setPower(backLeftPower);
        backRight.setPower(backRightPower);
    }

    public void Intake(double power) {
        LeftServo.setPower(power);
        RightServo.setPower(-power);
    }

 */
}
