package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Motor Spin Test")
public class MotorSpin extends OpMode {

    DcMotor motor;
    Servo servo;


    @Override
    public void init() {
        motor = hardwareMap.get(DcMotor.class, "motor");
        servo = hardwareMap.get(Servo.class, "servo");
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    @Override
    public void loop() {

        if(gamepad1.circle){
            //servo.setPosition(0.4);
            motor.setPower(0.5);
        }
        if(gamepad1.cross) {
            //servo.setPosition(0.0);
            motor.setPower(0);
        }
    }

}



