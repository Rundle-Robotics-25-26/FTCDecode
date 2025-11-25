package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.teamcode.Indexer;
import org.firstinspires.ftc.teamcode.SimpleTurret;
import org.firstinspires.ftc.teamcode.Spindexer;

@TeleOp(name = "evoTeleop", group = "Drive")
public class evoTeleop extends OpMode {

    // Motor declarations
    private DcMotor frontLeft, frontRight, backLeft, backRight, spinner;
    private DcMotorEx shooter; // Use DcMotorEx for PID control
    CRServo LeftServo, RightServo;

    // ==== Shooter PIDF and Velocity Constants ====
    // NOTE: These values MUST be tuned for your specific shooter setup.
    public static double SHOOTER_P = 10.0;
    public static double SHOOTER_I = 0.0;
    public static double SHOOTER_D = 0.0;
    // F is the most important for velocity control. A good starting point is 32767 / max_ticks_per_sec.
    // For a 6000 RPM motor (max ~2800 ticks/sec), F is around 11-12.
    public static double SHOOTER_F = 12.0;

    // Encoder ticks per revolution for a goBILDA 5203 series motor.
    public static final double TICKS_PER_REV = 28;

    // Define target velocities based on desired RPM.
    // Formula: (RPM / 60) * TICKS_PER_REV
    public static final double HIGH_VELOCITY = (4500.0 / 60.0) * TICKS_PER_REV;   // 2100 ticks/sec
    public static final double MEDIUM_VELOCITY = (3500.0 / 60.0) * TICKS_PER_REV; // ~1633 ticks/sec
    public static final double LOW_VELOCITY = (2500.0 / 60.0) * TICKS_PER_REV;    // ~1167 ticks/sec

    boolean pressedTriangle, pressedSquare, pressedCross, shooterOn = false;
    private double currentShooterVelocity = 0.0;

    private final Spindexer spindexer = new Spindexer();
    private final SimpleTurret simpleTurret = new SimpleTurret();
    private final Indexer indexer = new Indexer();

    @Override
    public void init() {
        // ==== Mecanum Drive Setup ====
        frontLeft = hardwareMap.get(DcMotor.class, "lf");
        backLeft = hardwareMap.get(DcMotor.class, "lr");
        frontRight = hardwareMap.get(DcMotor.class, "rf");
        backRight = hardwareMap.get(DcMotor.class, "rr");

        spinner = hardwareMap.get(DcMotor.class, "motor2");

        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.FORWARD);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // ==== Spinner Setup ====
        spinner.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        spinner.setTargetPosition(0);
        spinner.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        spinner.setPower(0.3);
        spinner.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // ==== Intake setup ====
        LeftServo = hardwareMap.get(CRServo.class, "LeftServo");
        RightServo = hardwareMap.get(CRServo.class, "RightServo");

        // ==== Spindexer setup ====
        spindexer.freshInit(hardwareMap);

        // ==== Simple turret setup ====
        //simpleTurret.Init(hardwareMap);

        // ==== Shooter setup with PID ====
        shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // Use RUN_USING_ENCODER for velocity control
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        // The shooter motor direction might need to be reversed depending on its physical orientation
        // shooter.setDirection(DcMotor.Direction.REVERSE);

        // Set the PIDF coefficients for the motor
        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(SHOOTER_P, SHOOTER_I, SHOOTER_D, SHOOTER_F);
        shooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);

        // ==== Indexer setup ====
        indexer.Init(hardwareMap, telemetry);
    }

    @Override
    public void loop() {
        // ==== Mecanum Drive ====
        double drive = -gamepad1.left_stick_y;
        double strafe = gamepad1.left_stick_x;
        double rotate = gamepad1.right_stick_x;
        driveMecanum(drive, strafe, rotate);

        // ==== Intake ====
        if (gamepad1.right_trigger > 0.1) {
            Intake(1);
        } else if (gamepad1.dpad_up) {
            Intake(-1);
        } else {
            Intake(0);
        }

        // ==== Spinner Status ====
        telemetry.addData("Spinner Position", spinner.getCurrentPosition());

        // ==== Simple turret ====
        //simpleTurret.TurretControl(gamepad1.left_trigger, gamepad1.right_trigger);

        // ==== Shooter with PID Velocity Control ====
        // Square button: Toggle Medium Velocity
        if (gamepad1.square) {
            if (!pressedTriangle) {
                pressedTriangle = true;
                if (shooterOn && currentShooterVelocity == MEDIUM_VELOCITY) {
                    shooter.setVelocity(0);
                    shooterOn = false;
                    currentShooterVelocity = 0.0;
                } else {
                    shooter.setVelocity(MEDIUM_VELOCITY);
                    shooterOn = true;
                    currentShooterVelocity = MEDIUM_VELOCITY;
                }
            }
        } else {
            pressedTriangle = false;
        }

        // Triangle button: Toggle Low Velocity
        if (gamepad1.triangle) {
            if (!pressedSquare) {
                pressedSquare = true;
                if (shooterOn && currentShooterVelocity == LOW_VELOCITY) {
                    shooter.setVelocity(0);
                    shooterOn = false;
                    currentShooterVelocity = 0.0;
                } else {
                    shooter.setVelocity(LOW_VELOCITY);
                    shooterOn = true;
                    currentShooterVelocity = LOW_VELOCITY;
                }
            }
        } else {
            pressedSquare = false;
        }

        // Cross button: Toggle High Velocity
        if (gamepad1.cross) {
            if (!pressedCross) {
                pressedCross = true;
                if (shooterOn && currentShooterVelocity == HIGH_VELOCITY) {
                    shooter.setVelocity(0);
                    shooterOn = false;
                    currentShooterVelocity = 0.0;
                } else {
                    shooter.setVelocity(HIGH_VELOCITY);
                    shooterOn = true;
                    currentShooterVelocity = HIGH_VELOCITY;
                }
            }
        } else {
            pressedCross = false;
        }

        // ==== Shooter Telemetry ====
        telemetry.addData("Shooter Target Velocity", "%.2f ticks/s (%d RPM)", currentShooterVelocity, (int) ((currentShooterVelocity / TICKS_PER_REV) * 60));
        telemetry.addData("Shooter Current Velocity", "%.2f ticks/s (%d RPM)", shooter.getVelocity(), (int) ((shooter.getVelocity() / TICKS_PER_REV) * 60));
        telemetry.addData("Shooter Status", shooterOn ? "RUNNING" : "STOPPED");

        // ==== Indexer ====
        indexer.Update(gamepad1.circle);

        // ==== Spindexer ====
        if (gamepad1.left_bumper || gamepad1.right_bumper) {
            indexer.spindex(gamepad1.left_bumper, spindexer);
        }

        telemetry.update();
    }

    private void driveMecanum(double drive, double strafe, double rotate) {
        double frontLeftPower = drive + strafe + rotate;
        double frontRightPower = drive - strafe - rotate;
        double backLeftPower = drive - strafe + rotate;
        double backRightPower = drive + strafe - rotate;

        double maxPower = Math.max(Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower)),
                Math.max(Math.abs(backLeftPower), Math.abs(backRightPower)));

        if (maxPower > 1.0) {
            frontLeftPower /= maxPower;
            frontRightPower /= maxPower;
            backLeftPower /= maxPower;
            backRightPower /= maxPower;
        }

        frontLeft.setPower(frontLeftPower);
        frontRight.setPower(frontRightPower);
        backLeft.setPower(backLeftPower);
        backRight.setPower(backRightPower);
    }

    public void Intake(double power) {
        LeftServo.setPower(power);
        RightServo.setPower(-power);
    }
}

//676767676767676767 MASON MUSTARD MANGO LABUBUUUUU