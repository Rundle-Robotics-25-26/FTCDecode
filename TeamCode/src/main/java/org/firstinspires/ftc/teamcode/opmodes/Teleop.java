package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Indexer;
//import org.firstinspires.ftc.teamcode.SimpleTurret;
import org.firstinspires.ftc.teamcode.SimpleTurret;
import org.firstinspires.ftc.teamcode.Spindexer;

@TeleOp(name = "Teleop", group = "Drive")
public class Teleop extends OpMode {

    // Motor declarations
    private DcMotor frontLeft, frontRight, backLeft, backRight, shooter, spinner;
    CRServo LeftServo, RightServo;

    boolean pressedTriangle, pressedSquare, pressedCross, shooterOn = false;
    private double currentShooterPower = 0.0;

    private final Spindexer spindexer = new Spindexer();
    private final SimpleTurret simpleTurret = new SimpleTurret();

    private final Indexer indexer = new Indexer();
    boolean spindexerDirection = true;

    @Override
    public void init() {
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
        shooter = hardwareMap.get(DcMotor.class, "shooter");
        // FIX: Use RUN_WITHOUT_ENCODER for consistent power output
        shooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        // ==== Indexer setup ====
        indexer.Init(hardwareMap, telemetry);
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

        //telemetry.addData("Current spindexer positions: ", "Pos1 (Indexer position): %d, Pos2 (To the right of indexer): %d, Pos3: %d", spindexer.positions[0], spindexer.positions[1], spindexer.positions[2]);

        // ==== Spinner Status ====
        telemetry.addData("Spinner Position", spinner.getCurrentPosition());
        telemetry.addData("Spinner Target", spinner.getTargetPosition());
        telemetry.addData("Spinner Power", spinner.getPower());
        telemetry.addData("Spinner Busy", spinner.isBusy());

        // ==== Simple turret ====
        //simpleTurret.TurretControl(gamepad1.left_trigger, gamepad1.right_trigger);
        //telemetry.addData("Current turret position: ", simpleTurret.turret.getCurrentPosition());

        // ==== Shooter ====
        // Triangle button: Toggle 0.6 power
        if (gamepad1.square) {
            if (!pressedTriangle) {
                pressedTriangle = true;

                if (shooterOn && currentShooterPower == -0.6) {
                    // If shooter is running at 0.6, turn it off
                    shooter.setPower(0);
                    shooterOn = false;
                    currentShooterPower = 0.0;
                } else {
                    // Turn on shooter at 0.6 power
                    shooter.setPower(-0.6);
                    shooterOn = true;
                    currentShooterPower = -0.6;
                }
            }
        } else {
            pressedTriangle = false;
        }

        // Square button: Toggle 0.4 power
        if (gamepad1.triangle) {
            if (!pressedSquare) {
                pressedSquare = true;

                if (shooterOn && currentShooterPower == -0.5) {
                    // If shooter is running at 0.4, turn it off
                    shooter.setPower(0);
                    shooterOn = false;
                    currentShooterPower = 0.0;
                } else {
                    // Turn on shooter at 0.4 power
                    shooter.setPower(-0.5);
                    shooterOn = true;
                    currentShooterPower = -0.5;
                }
            }
        } else {
            pressedSquare = false;
        }

        // Cross button: Toggle 0.8 power
        if (gamepad1.cross) {
            if (!pressedCross) {
                pressedCross = true;

                if (shooterOn && currentShooterPower == -0.75) {
                    // If shooter is running at 0.8, turn it off
                    shooter.setPower(0);
                    shooterOn = false;
                    currentShooterPower = 0.0;
                } else {
                    // Turn on shooter at 0.8 power
                    shooter.setPower(-0.75);
                    shooterOn = true;
                    currentShooterPower = -0.75;
                }
            }
        } else {
            pressedCross = false;
        }

        // ==== Shooter Telemetry ====
        /*telemetry.addData("Shooter Power", "%.2f", shooter.getPower());
        telemetry.addData("Shooter Mode", shooter.getMode().toString());
        telemetry.addData("Shooter Status", shooterOn ? "RUNNING" : "STOPPED");
        telemetry.addData("Shooter Speed", "%.1f", Math.abs(currentShooterPower * 100));
        telemetry.addData("Controls", "△: 0.6 Power, □: 0.4 Power, X: 0.8 Power");
        */
        // ==== Indexer ====
        indexer.Update(gamepad1.circle);

        // ==== Spindexer ====
        telemetry.addData("Is Spindexer busy? ", spindexer.spindexer.isBusy());
        if (gamepad1.left_bumper || gamepad1.right_bumper) {
            spindexerDirection = gamepad1.dpad_left;
            indexer.spindex(gamepad1.left_bumper, spindexer);
        }
        indexer.spindexerUpdate(spindexerDirection, spindexer);

        telemetry.addData("Intake RT", "%.2f", rt);
        telemetry.addData("LeftServo Power", LeftServo.getPower());
        telemetry.addData("RightServo Power", RightServo.getPower());
        telemetry.update();
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
}