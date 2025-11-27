package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name = "Motor Power Control", group = "Test")
public class MotorPowerControl extends OpMode {
    private DcMotorEx motor2;
    private double motorPower = 0.0;
    private boolean trianglePressed = false;
    private boolean crossPressed = false;

    @Override
    public void init() {
        // Initialize motor
        motor2 = hardwareMap.get(DcMotorEx.class, "motor2");
        motor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        telemetry.addData("Status", "Initialized");
        telemetry.addData("Controls", "Triangle: +0.05, Cross: -0.05");
        telemetry.addData("Current Power", "%.2f", motorPower);
    }

    @Override
    public void loop() {
        // Triangle button - increase power by 0.05
        if (gamepad1.triangle && !trianglePressed) {
            motorPower += 0.05;
            motorPower = Math.min(1.0, motorPower); // Cap at 1.0
            trianglePressed = true;
        } else if (!gamepad1.triangle) {
            trianglePressed = false;
        }

        // Cross button - decrease power by 0.05
        if (gamepad1.cross && !crossPressed) {
            motorPower -= 0.05;
            motorPower = Math.max(-1.0, motorPower); // Cap at -1.0
            crossPressed = true;
        } else if (!gamepad1.cross) {
            crossPressed = false;
        }

        // Apply power to motor
        motor2.setPower(motorPower);

        // Telemetry
        telemetry.addData("Motor Power", "%.2f", motorPower);
        telemetry.addData("Triangle", "Increase +0.05");
        telemetry.addData("Cross", "Decrease -0.05");
        telemetry.addData("Note", "Power range: -1.0 to +1.0");
        telemetry.update();
    }
}