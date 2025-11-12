package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

//This code has been checked by Harrison Webster

@Autonomous(name="BallAutoRed", group="Auto")
public class BallAutoRed extends LinearOpMode {

    private DcMotor frontLeft = null;
    private DcMotor frontRight = null;
    private DcMotor backLeft = null;
    private DcMotor backRight = null;
    private DcMotor extendMotor = null;
    private DcMotor shooter = null;
    private Servo baseIndexer = null;
    private Servo armIndexer = null;


    @Override
    public void runOpMode() {
        frontLeft = hardwareMap.get(DcMotor.class, "lf");
        frontRight = hardwareMap.get(DcMotor.class, "rf");
        backLeft = hardwareMap.get(DcMotor.class, "lr");
        backRight = hardwareMap.get(DcMotor.class, "rr");

        shooter = hardwareMap.get(DcMotor.class, "shooter");

        baseIndexer = hardwareMap.get(Servo.class, "base");
        armIndexer = hardwareMap.get(Servo.class, "arm");


        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.FORWARD);

        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        baseIndexer.setPosition(0.13);
        armIndexer.setPosition(0.1317);
        waitForStart();

        long DELAY_BETWEEN_MOVES = 1000;




        driveBackward(1000);
        sleep(1000);
        shooter.setPower(-0.5);
        sleep(3000);
        baseIndexer.setPosition(0.3961);
        sleep(400);
        armIndexer.setPosition(0.4967);
        sleep(1500);
        shooter.setPower(0);
        baseIndexer.setPosition(0.13);
        armIndexer.setPosition(0.1317);
        strafeLeft(750);



    }

//    private void setupAuto() {
//        extendMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//    }


    private void driveForward(long duration) {
        setMotorPowers(0.4, 0.4, 0.4, 0.4);
        sleep(duration);
        stopMotors();
    }

    private void driveBackward(long duration) {
        setMotorPowers(-0.4, -0.4, -0.4, -0.4);
        sleep(duration);
        stopMotors();
    }


    private void strafeLeft(long duration) {
        setMotorPowers(0.4, -0.4, -0.4, 0.4);
        sleep(duration);
        stopMotors();
    }

    private void strafeRight(long duration) {
        setMotorPowers(-0.4, 0.4, 0.4, -0.4);
        sleep(duration);
        stopMotors();
    }



    private void setMotorPowers(double fl, double fr, double bl, double br) {
        frontLeft.setPower(fl);
        frontRight.setPower(fr);
        backLeft.setPower(bl);
        backRight.setPower(br);
    }

    private void stopMotors() {
        setMotorPowers(0, 0, 0, 0);
    }
}