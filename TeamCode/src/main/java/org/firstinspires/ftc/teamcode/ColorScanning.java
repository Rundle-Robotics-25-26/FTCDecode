package org.firstinspires.ftc.teamcode;


import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.ColorSensor;
import org.firstinspires.ftc.teamcode.subsystems.Spindexer;

public class ColorScanning {
    private ColorSensor colorSensor;
    private Spindexer spindexer;

    private Telemetry telemetry;

    private enum ScanState {
        IDLE,
        SPINNING
    }
    private ScanState currentState = ScanState.IDLE;

    private RobotTimer spinTimer = new RobotTimer(1000); // how long it takes to spin

    int[] states = {0, 0, 0}; // 0 is empty, 1 is green, -1 is purple

    public void Init(ColorSensor sensor, Spindexer spin, Telemetry tele) {
        telemetry = tele;
        colorSensor = sensor;
        spindexer = spin;
    }

    public void Update() {
        switch (currentState) {
            case IDLE:
                // when idle check if
                String scanResult = colorSensor.BallDetermineUpdate();
                telemetry.addData("Scan result: ", scanResult);
                if (scanResult.equals("PURPLE") || scanResult.equals("GREEN")) {
                    // have sensed a ball so set it in memory and spin
                    if (scanResult.equals("PURPLE")) {
                        states[spindexer.currentPosition-1] = -1;
                    } else {
                        states[spindexer.currentPosition-1] = 1;
                    }
                    spindexer.rotateClockwise(false);
                    spinTimer.start();
                    colorSensor.Reset();
                    currentState = ScanState.SPINNING;
                }
                break;
            case SPINNING:
                if (spinTimer.IsDone()) {
                    currentState = ScanState.IDLE;
                }
                break;

        }
    }
}
