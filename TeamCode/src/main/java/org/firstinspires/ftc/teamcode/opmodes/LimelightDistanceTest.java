package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.subsystems.Limelight;

//@Autonomous(name = "Limelight Distance Test")
public class LimelightDistanceTest extends OpMode {
    private Limelight limelight = new Limelight();
    private LLResult latestResult;

    @Override
    public void init() {
        limelight.Init(hardwareMap, telemetry);
    }

    @Override
    public void loop() {
        latestResult = limelight.Update();
        double distance = limelight.getTargetDistance(latestResult);

        telemetry.addData("Distance to target (inches):", distance);
    }

    @Override
    public void start() {
        limelight.Start();
    }
}
