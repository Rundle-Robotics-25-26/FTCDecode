package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Indexer;
//import org.firstinspires.ftc.teamcode.SimpleTurret;
import org.firstinspires.ftc.teamcode.Intake;
import org.firstinspires.ftc.teamcode.SimpleTurret;
import org.firstinspires.ftc.teamcode.Spindexer;

@TeleOp(name = "Teleop", group = "Drive")
public class Teleop extends OpMode {

    // Motor declarations
    private DcMotor frontLeft, frontRight, backLeft, backRight, shooter, spinner, LeftIntake;
    DcMotor LeftServo, RightServo;

    boolean pressedTriangle, pressedSquare, pressedCross, shooterOn = false;
    private double currentShooterPower = 0.0;

    private final Spindexer spindexer = new Spindexer();
    private final SimpleTurret simpleTurret = new SimpleTurret();

    private final Indexer indexer = new Indexer();
    private final Intake intake = new Intake();
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
        intake.Init(telemetry, hardwareMap);


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
        indexer.Init(hardwareMap, telemetry, spindexer);
    }

    @Override
    public void loop() {
        // ==== Mecanum Drive ====
        double drive = gamepad1.left_stick_y;  // Reverse Y axis//we made it separate for 2 controllers
        double strafe = gamepad1.left_stick_x;
        double rotate = gamepad1.right_stick_x;
        driveMecanum(drive, strafe, rotate);

        // ==== Intake ====
        // Use trigger value for proportional control, keep small deadzone
        if (gamepad1.left_trigger > 0.05) {
            intake.spinOut();
        } else if (gamepad1.right_trigger > 0.05) {
            intake.stop();
        } else {
            intake.spinIn();
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
        if (gamepad1.squareWasPressed()) {
            setShooter(-0.7);
        }

        // Square button: Toggle 0.4 power
        if (gamepad1.triangleWasPressed()) {
            setShooter(-0.65);
        }

        // Cross button: Toggle 0.8 power
        if (gamepad1.crossWasPressed()) {
            setShooter(-1);
        }

        // ==== Shooter Telemetry ====
        /*telemetry.addData("Shooter Power", "%.2f", shooter.getPower());
        telemetry.addData("Shooter Mode", shooter.getMode().toString());
        telemetry.addData("Shooter Status", shooterOn ? "RUNNING" : "STOPPED");
        telemetry.addData("Shooter Speed", "%.1f", Math.abs(currentShooterPower * 100));
        telemetry.addData("Controls", "△: 0.6 Power, □: 0.4 Power, X: 0.8 Power");
        */

        // ==== Manual Spindexer ====
        // ==== Indexer ====
        if (gamepad1.circleWasPressed()) {
            indexer.Index();
        }
        indexer.Update();


        // ==== Spindexer ====
        telemetry.addData("Is Spindexer busy? ", spindexer.spindexer.isBusy());
        if (indexer.currentState == Indexer.State.IDLE) { // can't spindex if indexer is shooting
            if (gamepad1.left_bumper) {
                spindexer.rotateClockwise();
            } else if (gamepad1.right_bumper) {
                spindexer.rotateCounterclockwise();
            }
        }

        final int MANUAL_SPEED = 6;
        if (gamepad1.dpad_left) {
            spindexer.spindexer.setTargetPosition(spindexer.spindexer.getCurrentPosition() + MANUAL_SPEED);
        } else if (gamepad1.dpad_right) {
            spindexer.spindexer.setTargetPosition(spindexer.spindexer.getCurrentPosition() - MANUAL_SPEED);
        }

        // ====

        telemetry.update();
    }

    private void setShooter(double power) {
        if (power == currentShooterPower) {
            currentShooterPower = 0;
        } else {
            currentShooterPower = power;
        }
        shooter.setPower(currentShooterPower);
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

}