package org.firstinspires.ftc.teamcode.opmodes.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

public class SpindexerBasic extends OpMode {

    DcMotor spindexer;
    @Override
    public void init() {
        spindexer = hardwareMap.get(DcMotor.class, "motor2");
    }

    @Override
    public void loop() {

    }
}
