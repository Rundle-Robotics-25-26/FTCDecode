package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.ColorScanning;
import org.firstinspires.ftc.teamcode.ColorSensor;
import org.firstinspires.ftc.teamcode.Spindexer;

//@TeleOp(name="Color scan test")
public class ColorScanningTest extends OpMode {
    // this is just a test to see how fast we can scan all slots in spindexer
    // tune how much time needed to scan a ball
    private ColorSensor colorSensor = new ColorSensor();
    private Spindexer spindexer = new Spindexer();
    private ColorScanning colorScanning = new ColorScanning();
    @Override
    public void init() {
        colorSensor.Init(hardwareMap, telemetry);
        spindexer.freshInit(hardwareMap);

        colorScanning.Init(colorSensor, spindexer, telemetry); // color scanning requires color sensor and spindexer
    }

    @Override
    public void loop() {
        if (gamepad1.circle) {
            colorScanning.Reset();
        }
        //colorScanning.ScanAll(); // test 1
        colorScanning.MoveToEmptySlot(); // test 2
    }
}
