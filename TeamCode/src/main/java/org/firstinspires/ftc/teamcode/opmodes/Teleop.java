package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.SimpleTurret;
import org.firstinspires.ftc.teamcode.Spindexer;

@TeleOp(name = "Teleop", group = "Drive")
public class Teleop extends OpMode {

    // Motor declarations
    private DcMotor frontLeft, frontRight, backLeft, backRight, shooter;
    CRServo LeftServo, RightServo;

    boolean pressedTriangle, shooterOn = false;

    private final Spindexer spindexer = new Spindexer();
    private final SimpleTurret simpleTurret = new SimpleTurret();


    @Override
    public void init() {
        // ==== Mecanum Drive Setup ====
        frontLeft = hardwareMap.get(DcMotor.class, "leftFront");
        backLeft = hardwareMap.get(DcMotor.class, "leftBack");
        frontRight = hardwareMap.get(DcMotor.class, "rightFront");
        backRight = hardwareMap.get(DcMotor.class, "rightBack");

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

        // ==== Intake setup ====
        LeftServo = hardwareMap.get(CRServo.class, "LeftServo");
        RightServo = hardwareMap.get(CRServo.class, "RightServo");

        // ==== Spindexer setup ====
        spindexer.freshInit(hardwareMap);

        // ==== Simple turret setup ====
        simpleTurret.Init(hardwareMap);

        // ==== Shooter setup ====
        shooter = hardwareMap.get(DcMotor.class, "shooter");
    }

    @Override
    public void loop() {
        // ==== Mecanum Drive ====
        double drive = -gamepad1.left_stick_y;  // Reverse Y axis
        double strafe = gamepad1.left_stick_x;
        double rotate = gamepad1.right_stick_x;
        driveMecanum(drive, strafe, rotate);

        // ==== Intake ====
        Intake(0);
        if(gamepad1.right_bumper){
            Intake(1);
        }
        if(gamepad1.left_bumper) {
            Intake(-1);
        }

        // ==== Spindexer ====
        if (gamepad1.dpad_left) {
            spindexer.rotateClockwise();
        } else if (gamepad1.dpad_right) {
            spindexer.rotateCounterclockwise();
        }
        telemetry.addData("Current spindexer positions: ", "Pos1 (Indexer position): %d, Pos2 (To the right of indexer): %d, Pos3: %d", spindexer.positions[0], spindexer.positions[1], spindexer.positions[2]);

        // ==== Simple turret ====
        simpleTurret.TurretControl(gamepad1.left_trigger, gamepad1.right_trigger);
        telemetry.addData("Current turret position: ", simpleTurret.turret.getCurrentPosition());

        // ==== Shooter ====
        // You toggle whether the shooter is on or off
        final double SHOOT_POWER = 1;
        if (gamepad1.triangle) {
            if (!pressedTriangle) {
                pressedTriangle = true;
                // When triangle is first clicked

                if (shooterOn) {
                    shooter.setPower(0);
                } else {
                    shooter.setPower(SHOOT_POWER);
                }
                shooterOn = !shooterOn;
            }
        } else {
            pressedTriangle = false;
        }

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