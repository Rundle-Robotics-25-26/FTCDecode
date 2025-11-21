package org.firstinspires.ftc.teamcode.opmodes.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Spindexer;

// this is so we can find how much a full rotation of the spindexer is
@TeleOp(name = "Spindexer test", group = "Tests")
public class SpindexerTest extends OpMode {
    Spindexer spindexer = new Spindexer();
    @Override
    public void init() {
        spindexer.freshInit(hardwareMap);
    }

    @Override
    public void loop() {
        int SPEED = 1;
        if (gamepad1.dpad_left) {
            spindexer.spindexer.setTargetPosition(spindexer.spindexer.getCurrentPosition() + SPEED);
        } else if (gamepad1.dpad_right) {
            spindexer.spindexer.setTargetPosition(spindexer.spindexer.getCurrentPosition() - SPEED);
        }
        telemetry.addData("Current position", spindexer.spindexer.getCurrentPosition());
        telemetry.update();
    }
}
