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

    final double ARM_START = 0;
    final double ARM_INDEX = 0.5;
    final double BASE_START = 0;
    final double BASE_INDEX = 0.5;

    //final double BASE_ROTATION_DEGREES = ;

    public void Init(HardwareMap hardware, Telemetry tele) {
        hardwareMap = hardware;
        telemetry = tele;

        baseIndexer = hardwareMap.get(Servo.class, "base");
        armIndexer = hardwareMap.get(Servo.class, "arm");
    }
    public void TestUpdate(int armMovement, int baseMovement) {
        double SPEED = 0.01;

        if (armMovement > 0) {
            armIndexer.setPosition(armIndexer.getPosition() + SPEED);
        } else if (armMovement < 0) {
            armIndexer.setPosition(armIndexer.getPosition() - SPEED);
        }

        if (baseMovement > 0) {
            baseIndexer.setPosition(baseIndexer.getPosition() + SPEED);
        } else if (baseMovement < 0) {
            baseIndexer.setPosition(baseIndexer.getPosition() - SPEED);
        }
        telemetry.addData("Current base position", baseIndexer.getPosition());
        telemetry.addData("Current arm position", armIndexer.getPosition());
        telemetry.update();
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

        baseIndexer.setPosition(BASE_INDEX);
        armIndexer.setPosition(ARM_INDEX);
    }

    public void Closed() {
        if (!indexerOpen) {
            return;
        }

        baseIndexer.setPosition(BASE_START);
        armIndexer.setPosition(ARM_START);
    }


}
