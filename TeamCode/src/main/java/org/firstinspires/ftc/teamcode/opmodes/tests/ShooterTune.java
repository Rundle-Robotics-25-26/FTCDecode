package org.firstinspires.ftc.teamcode.opmodes.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.opmodes.Shooter;

public class ShooterTune extends OpMode {
    Shooter shooter = new Shooter();
    @Override
    public void init() {
        shooter.Init(hardwareMap, telemetry);
    }

    @Override
    public void loop() {
        if (gamepad1.left_bumper) {

        }
        else if (gamepad1.right_bumper) {
        }

        if (gamepad1.circle) {
        }
        telemetry.update();
    }
}
