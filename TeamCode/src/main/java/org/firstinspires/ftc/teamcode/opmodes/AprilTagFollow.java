package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.subsystems.Limelight;

import java.util.List;

//@Autonomous(name="April Tag Following")
public class AprilTagFollow extends OpMode {
    private final Limelight limelight = new Limelight();
    private LLResult latestResult = null;

    final double STRAFE_SPEED = 0.3;
    final double ROTATE_SPEED = 0.1;

    private DcMotor front_left  = null;
    private DcMotor front_right = null;
    private DcMotor back_left   = null;
    private DcMotor back_right  = null;

    @Override
    public void loop() {
        latestResult = limelight.Update();
        follow();
    }

    private void follow() {
        List<LLResultTypes.FiducialResult> aprilTags = limelight.detectAprilTags(latestResult);
        LLResultTypes.FiducialResult aprilTag;

        telemetry.addData("===== Follow Data", "=====");

        // dont do anything if no apriltags
        if (aprilTags.isEmpty()) {
            telemetry.addData("Status: ", "No Apriltags");
            return;
        }

        aprilTag = aprilTags.get(0); // get best april tag

        final double DEADZONE = 1.5;

        double tx = aprilTag.getTargetXDegrees();

        telemetry.addData("AprilTag TX", "%.2f°", tx);
        telemetry.addData("AprilTag ID", aprilTag.getFiducialId());

        if (Math.abs(tx) < DEADZONE) { // correctly pointing don't do anything
            telemetry.addData("Status: ", "Pointing at Apriltag!");
        } else if (tx < 0) { // strafe left if target to the left
            telemetry.addData("Status: ", "To the left to the left");
            RotateCounterClockwise();
        } else { // strafe right if target to the right
            telemetry.addData("Status: ", "To the right to the right");
            RotateClockwise();
        }
    }

    @Override
    public void init() {
        front_left   = hardwareMap.get(DcMotor.class, "front_left");
        front_right  = hardwareMap.get(DcMotor.class, "front_right");
        back_left    = hardwareMap.get(DcMotor.class, "back_left");
        back_right   = hardwareMap.get(DcMotor.class, "back_right");
        limelight.Init(hardwareMap, telemetry);
    }

    private void StrafeLeft() {
        front_left.setPower(-STRAFE_SPEED);
        front_right.setPower(STRAFE_SPEED);
        back_left.setPower(STRAFE_SPEED);
        back_right.setPower(-STRAFE_SPEED);
    }

    private void StrafeRight() {
        front_left.setPower(STRAFE_SPEED);
        front_right.setPower(-STRAFE_SPEED);
        back_left.setPower(-STRAFE_SPEED);
        back_right.setPower(STRAFE_SPEED);
    }

    private void RotateClockwise() {
        front_left.setPower(ROTATE_SPEED);
        front_right.setPower(-ROTATE_SPEED);
        back_left.setPower(ROTATE_SPEED);
        back_right.setPower(-ROTATE_SPEED);
    }

    private void RotateCounterClockwise() {
        front_left.setPower(-ROTATE_SPEED);
        front_right.setPower(ROTATE_SPEED);
        back_left.setPower(-ROTATE_SPEED);
        back_right.setPower(ROTATE_SPEED);
    }

    @Override
    public void start() {
        limelight.Start();
    }
}
