package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

//@TeleOp(name = "Motor Spin Test")
public class MotorSpin extends OpMode {

    //DcMotor motor;
    Servo servo;
    Servo servo1;


    @Override
    public void init() {
        //motor = hardwareMap.get(DcMotor.class, "motor");
        servo = hardwareMap.get(Servo.class, "base");
        servo1 = hardwareMap.get(Servo.class, "arm");
        //motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    @Override
    public void loop() {

        if(gamepad1.square){
            servo.setPosition(0.3);
            //motor.setPower(1.0);
        }
        if(gamepad1.cross) {
            servo.setPosition(0.0);
            //motor.setPower(0);
        }
        //------------------
        if(gamepad1.circle) {
            servo.setPosition(-0.2);
            //motor.setPower(0.5);
        }
        if(gamepad1.triangle) {
            servo.setPosition(0.5);
            //motor.setPower(0.5);
        }
    }

}



