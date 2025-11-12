package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.internal.system.Deadline;

import java.util.concurrent.TimeUnit;

//@TeleOp(name = "Field Centric Mecanum Drive")
public class FieldCentricDrive extends LinearOpMode {

    DcMotor turret;

    final double SPEED = 0.01;
    @Override
    public void runOpMode() {
        turret = hardwareMap.get(DcMotor.class, "turret");

        DcMotor leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        DcMotor leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        DcMotor rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        DcMotor rightBack = hardwareMap.get(DcMotor.class, "rightBack");

        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.REVERSE);

        Deadline gamepadRateLimit = new Deadline(500, TimeUnit.MILLISECONDS);

        IMU imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
        ));
        imu.initialize(parameters);

        waitForStart();

        while (opModeIsActive()) {
            TurretControl();

            double lx = gamepad1.left_stick_x;
            double ly = -gamepad1.left_stick_y;
            double rx = gamepad1.right_stick_x;

            double max = Math.max(Math.abs(lx) + Math.abs(ly) + Math.abs(rx), 1);

            double drivePower = 0.8 - (0.6 * gamepad1.right_trigger);

            if (gamepadRateLimit.hasExpired() && gamepad1.cross) {
                imu.resetYaw();
                gamepadRateLimit.reset();
            }

            double heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            double adjustedLx = -ly * Math.sin(heading) + lx * Math.cos(heading);
            double adjustedLy = ly * Math.cos(heading) + lx * Math.sin(heading);

            leftFront.setPower(((adjustedLy + adjustedLx + rx) / max) * drivePower);
            leftBack.setPower(((adjustedLy - adjustedLx + rx) / max) * drivePower);
            rightFront.setPower(((adjustedLy - adjustedLx - rx) / max) * drivePower);
            rightBack.setPower(((adjustedLy + adjustedLx - rx) / max) * drivePower);

            telemetry.update();
        }
    }

    public void TurretControl() {
        if (gamepad1.left_bumper) {
            turret.setPower(SPEED);
        }
        if (gamepad1.right_bumper) {
            turret.setPower(SPEED);
        }
        telemetry.addData("Turret position:", turret.getCurrentPosition());
    }
}