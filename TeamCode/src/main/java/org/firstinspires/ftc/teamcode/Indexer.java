package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Indexer {
    private HardwareMap hardwareMap;
    private Telemetry telemetry;

    private boolean indexerOpen = false;
    private boolean indexerPressed = false;

    private Servo baseIndexer, armIndexer;

    final double ROTATION_DEGREES = ; // 120 deg,
    final double ROTATE = ROTATION_DEGREES / 360.0;

    public void Init(HardwareMap hardware, Telemetry tele) {
        hardwareMap = hardware;
        telemetry = tele;

        baseIndexer = hardwareMap.get(Servo.class, "base");
        armIndexer = hardwareMap.get(Servo.class, "arm");
    }

    public void Update(boolean pressed) {
        if (pressed) {
            if (!indexerPressed) {
                indexerPressed = true;
                if (!indexerOpen) {
                    Open();
                } else {
                    Closed();
                }
                indexerOpen = !indexerOpen;
            }
        } else {
            indexerPressed = false;
        }

        telemetry.addData("Current base position", baseIndexer.getPosition());
        telemetry.addData("Current arm position", armIndexer.getPosition());
    }

    public void Open() {
        if (indexerOpen) {
            return;
        }
        double basePosition = baseIndexer.getPosition();
        double armPosition = armIndexer.getPosition();

        baseIndexer.setPosition(basePosition + ROTATE);
        armIndexer.setPosition(armPosition - ROTATE);
    }

    public void Closed() {
        if (!indexerOpen) {
            return;
        }
        double basePosition = baseIndexer.getPosition();
        double armPosition = armIndexer.getPosition();

        baseIndexer.setPosition(basePosition - ROTATE);
        armIndexer.setPosition(armPosition + ROTATE);
    }
}
