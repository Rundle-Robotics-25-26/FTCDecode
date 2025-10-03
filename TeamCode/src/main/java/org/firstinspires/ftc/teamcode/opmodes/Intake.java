package org.firstinspires.ftc.teamcode.opmodes;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "intake")
public class Intake extends OpMode {

    CRServo LeftServo;
    CRServo RightServo;

    @Override
    public void init() {
        LeftServo = hardwareMap.get(CRServo.class, "LeftServo");
        RightServo = hardwareMap.get(CRServo.class, "RightServo");
    }

    public void in(double power) {
        LeftServo.setPower(power);
        RightServo.setPower(-power);
    }

    @Override
    public void loop() {
        if(gamepad1.right_bumper){
            in(1);
        }

        if(gamepad1.left_bumper) {
            in(0);
        }
    }
}
