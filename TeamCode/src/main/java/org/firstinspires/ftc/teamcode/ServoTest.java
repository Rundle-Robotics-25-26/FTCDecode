package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Servo Spin Test")
public class ServoTest extends OpMode {

    //DcMotor motor;
    Servo servoTest;



    @Override
    public void init() {

        servoTest = hardwareMap.get(Servo.class, "servoTest");


    }

    @Override
    public void loop() {

        if(gamepad1.circle){
            servoTest.setPosition(0.0);
            //motor.setPower(1.0);
        }
        if(gamepad1.cross) {
            servoTest.setPosition(0.4);
            //motor.setPower(0);
        }

        if(gamepad1.square) {
            servoTest.setPosition(-0.4);
            //motor.setPower(0);
        }


    }

}



