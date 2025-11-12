package org.firstinspires.ftc.teamcode.opmodes;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.ftc.localization.Encoder;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

//@TeleOp
public class ETele extends OpMode {
    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private DcMotor lfMotor = null;
    private DcMotor lrMotor = null;
    private DcMotor rfMotor = null;
    private DcMotor rrMotor = null;
    private TelemetryManager telemetryM;
    private DcMotorEx parallelEncoder;
    private DcMotorEx perpendicularEncoder;
    private double adjustSpeed = 0.5;
    private boolean slowModeActive = false;

    @Override
    public void init() {

        follower = Constants.createFollower(hardwareMap);

        rfMotor = hardwareMap.dcMotor.get("rf");
        lfMotor = hardwareMap.dcMotor.get("lf");
        rrMotor = hardwareMap.dcMotor.get("rr");
        lrMotor = hardwareMap.dcMotor.get("lr");

//        parallelEncoder = hardwareMap.get(DcMotorEx.class, "parallelEncoder");
//        perpendicularEncoder = hardwareMap.get(DcMotorEx.class, "perpendicularEncoder");

        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        pathTimer = new Timer();
        actionTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

    }

    @Override
    public void start(){
        loop();
        follower.startTeleopDrive();
    }

    @Override
    public void loop() {
        movement();

//        telemetry.addData("Front Left Power", lfMotor.getPower());
//        telemetry.addData("Front Right Power", rfMotor.getPower());
//        telemetry.addData("Back Left Power", lrMotor.getPower());
//        telemetry.addData("Back Right Power", rrMotor.getPower());
//        telemetry.addData("Perpendicular Encoders", perpendicularEncoder);
//        telemetry.addData("Parallel Encoders", parallelEncoder);
//        telemetryM.debug("position", follower.getPose());
//        telemetryM.debug("velocity", follower.getVelocity());
//        telemetry.update();
//        telemetryM.update();
    }

    public void movement(){
        if (gamepad1.dpad_up){
            slowModeActive = true;
        }
        if (gamepad1.dpad_down){
            slowModeActive = false;
        }

        if (!slowModeActive) {
            follower.setTeleOpDrive(
                    -gamepad1.left_stick_y,
                    -gamepad1.left_stick_x,
                    -gamepad1.right_stick_x,
                    true
            );
        }
        if (slowModeActive){
            follower.setTeleOpDrive(
                    -gamepad1.left_stick_y * adjustSpeed,
                    -gamepad1.left_stick_x * adjustSpeed,
                    -gamepad1.right_stick_x * adjustSpeed,
                    true
            );
        }
        if (gamepad1.dpadRightWasPressed() && adjustSpeed >= 1.0) {
            adjustSpeed += 0.2;
        }

        //Optional way to change slow mode strength
        if (gamepad2.dpadLeftWasPressed()) {
            adjustSpeed -= 0.2;
        }
    }

}

