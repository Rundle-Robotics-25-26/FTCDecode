package org.firstinspires.ftc.teamcode.opmodes.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Intake;

@TeleOp(name="Intake test", group="test")
public class IntakeTest extends OpMode {
    private final Intake intake = new Intake();
    @Override
    public void init() {
        intake.Init(telemetry, hardwareMap);
    }

    @Override
    public void loop() {

        // ==== Intake ====
        // Use trigger value for proportional control, keep small deadzone
        double rt = gamepad1.right_trigger;
        double lt = gamepad1.left_trigger;
        if (rt > 0.05) {
            // change: always run intake at full speed when trigger is pressed at all
            intake.spinIn(); // preserve original negative direction used previously
        } else if (lt > 0.05) {
            // manual full-speed reverse/intake
            intake.spinOut();
        } else {
            intake.stop();
        }
    }
}
