package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name = "Single Motor Ticks", group = "Debug")
public class TicksTest extends OpMode {

    private DcMotorEx motor;

    @Override
    public void init() {
        // Change "turretMotor" to whatever your motor is named in the config
        motor = hardwareMap.get(DcMotorEx.class, "turret");
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor.setPower(0); // Ensure motor is off

        telemetry.addData("Status", "Ready - Rotate motor manually to see ticks");
    }

    @Override
    public void loop() {
        // Just show the ticks - nothing else
        telemetry.addData("Motor Ticks", motor.getCurrentPosition());
        telemetry.update();
    }
}