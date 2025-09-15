package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Limelight;

@Autonomous(name = "April Tag Test")
public class AprilTagTest extends OpMode {
    private final Limelight limelight = new Limelight();
    private LLResult latestResult = null;

    @Override
    public void loop() {
        latestResult = limelight.Update();
        limelight.detectAprilTags(latestResult);
    }

    @Override
    public void init() {
        limelight.Init(hardwareMap, telemetry);
    }

    @Override
    public void start() {
        limelight.Start();
    }
}
