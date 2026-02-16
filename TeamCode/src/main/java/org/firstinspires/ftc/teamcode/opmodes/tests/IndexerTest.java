package org.firstinspires.ftc.teamcode.opmodes;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Indexer;

@TeleOp(name = "Indexer test", group = "Tests")
public class IndexerTest extends OpMode {

    private final Indexer indexer = new Indexer();

    @Override
    public void init() {
        indexer.Init(hardwareMap, telemetry, null);
    }

    @Override
    public void loop() {
        int movement = 0;
        if (gamepad1.left_bumper) {
            movement = 1;
        }
        if (gamepad1.right_bumper) {
            movement = -1;
        }
       indexer.TestUpdate(movement);
    }
}
