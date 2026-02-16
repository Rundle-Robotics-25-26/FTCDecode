package org.firstinspires.ftc.teamcode.opmodes.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

// this is so we can find how much a full rotation of the spindexer is
@TeleOp(name = "Spindexer test", group = "Tests")
public class SpindexerTest extends OpMode {
    //Spindexer spindexer = new Spindexer();
    DcMotor spindexer;
    @Override
    public void init() {
//        spindexer.freshInit(hardwareMap);
        spindexer = hardwareMap.get(DcMotor.class, "motor2");
        spindexer.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        spindexer.setTargetPosition(0);
        spindexer.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        spindexer.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        telemetry.addData("Start Current position", spindexer.getCurrentPosition());
        telemetry.addData("Start Target position", spindexer.getTargetPosition());
        telemetry.update();
    }

    @Override
    public void loop() {
        int SPEED = 10;
        spindexer.setPower(-0.8);

        if (gamepad1.dpadRightWasPressed()) {
            spindexer.setTargetPosition(100);
        } else if (gamepad1.dpadLeftWasPressed()) {
            spindexer.setTargetPosition(200);
        }


        telemetry.addData("Current position", spindexer.getCurrentPosition());
        telemetry.addData("Target position", spindexer.getTargetPosition());
        telemetry.update();
    }
}
