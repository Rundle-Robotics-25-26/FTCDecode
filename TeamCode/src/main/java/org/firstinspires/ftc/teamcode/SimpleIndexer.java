package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class SimpleIndexer {
    private Telemetry telemetry;
    private HardwareMap hardwareMap;
    private Servo indexer;

    private final double ARM_UP = 0.4;
    private final double ARM_DOWN = 0;
    private int indexerState = 0;

    private RobotTimer indexTimer = new RobotTimer(2000);

    public void Init(HardwareMap hardwareMap, Telemetry telemetry) {
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;

        indexer = hardwareMap.get(Servo.class, "indexer");
        indexer.setPosition(ARM_DOWN);
    }
    public boolean isIndexing() {
        return indexerState == 1;
   }
    public void Index() {
        if (isIndexing()) {
            telemetry.addData("Indexer Status:", "indexing");
            return;
        }
        ArmUp();
        indexTimer.start();
        indexerState = 1;
    }
    public void ArmUp() {
        indexer.setPosition(ARM_UP);
    }
    public void ArmDown() {
        indexer.setPosition(ARM_DOWN);
    }
    public void Update() {
        switch (indexerState) {
            case 0:
                // do nothing
                break;
            case 1:
                // currently indexing
                if (indexTimer.IsDone()) {
                    ArmDown();
                    indexerState = 0;
                }
                break;
        }

        telemetry.addData("Indexer state: ", indexerState);
    }
}
