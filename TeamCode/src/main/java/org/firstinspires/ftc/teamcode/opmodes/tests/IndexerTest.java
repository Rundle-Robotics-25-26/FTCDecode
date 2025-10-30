package org.firstinspires.ftc.teamcode.opmodes;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Indexer;

@TeleOp(name = "Indexer test")
public class IndexerTest extends OpMode {

    private final Indexer indexer = new Indexer();

    @Override
    public void init() {
        indexer.Init(hardwareMap, telemetry);
    }

    @Override
    public void loop() {
        int armMovement = 0;
        if (gamepad1.left_bumper) {
            armMovement = 1;
        }
        if (gamepad1.left_trigger > 0) {
            armMovement = -1;
        }

        int baseMovement = 0;
        if (gamepad1.right_bumper) {
            baseMovement = 1;
        }
        if (gamepad1.right_trigger > 0) {
            baseMovement = -1;
        }
        indexer.TestUpdate(armMovement, baseMovement);
    }
}
