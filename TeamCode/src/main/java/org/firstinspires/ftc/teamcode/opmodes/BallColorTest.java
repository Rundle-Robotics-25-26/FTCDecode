package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.ColorSensor;
import org.firstinspires.ftc.teamcode.Limelight;

//@TeleOp(name="Color sensing test")
public class BallColorTest extends OpMode {
    private final ColorSensor colorSensor = new ColorSensor();

    @Override
    public void loop() {
        colorSensor.BallDetermineUpdate();
    }

    @Override
    public void init() {
        colorSensor.Init(hardwareMap, telemetry);
    }
}
