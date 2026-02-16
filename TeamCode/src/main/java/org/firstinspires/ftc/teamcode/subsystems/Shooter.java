package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Shooter {
    DcMotorEx shooterMotor;
    Telemetry telemetry;

    // FIND ALL THESE VALUES PLEASE
    public final int MAX_SPEED = 0;
    public final int HIGH_SPEED = 0;
    public final int MEDIUM_SPEED = 0;
    public final int LOW_SPEED = 0;

    private double targetSpeed = 0; // keep private

    public void Init(HardwareMap hardwareMap, Telemetry telemetry) {
        shooterMotor = hardwareMap.get(DcMotorEx.class, "shooter");
        shooterMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        this.telemetry = telemetry;

        PIDFCoefficients pidf = new PIDFCoefficients(0, 0, 0, 0);
        shooterMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);
    }


    public void setSpeed(double speed) {
        targetSpeed = speed;
        shooterMotor.setVelocity(speed);
    }
    public void ManualSpeed(int speed) {
        setSpeed(targetSpeed + speed);

        // this might be better (IDK)
        // setSpeed(shooterMotor.getVelocity() + speed);
    }


    public void showTelemetry() {
        telemetry.addData("✡✡✡ Shooter ✡✡✡", "");
        telemetry.addData("Current velocity: ", shooterMotor.getVelocity());
        telemetry.addData("Target velocity: ", targetSpeed);
        telemetry.addData("Velocity error: ", (targetSpeed - shooterMotor.getVelocity()));
        telemetry.addData("✡✡✡✡✡✡✡", "");
    }
}
