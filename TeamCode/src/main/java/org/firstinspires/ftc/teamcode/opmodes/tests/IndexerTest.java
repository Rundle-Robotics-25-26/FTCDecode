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
        indexer.Update(gamepad1.cross);
    }
}
