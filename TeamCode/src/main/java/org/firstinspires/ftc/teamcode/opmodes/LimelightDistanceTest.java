package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Limelight;

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
