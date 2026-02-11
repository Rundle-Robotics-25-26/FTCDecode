package org.firstinspires.ftc.teamcode.opmodes.tests.tuning;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

@Configurable
@TeleOp(name="Velocity PIDF Tuner", group="tuning")
public class VelocityPIDFTuner extends OpMode {
    final String MOTORNAME = "shooter"; // set this to the motor you want to tune

    DcMotorEx motor;

    // probably only need to tune kP and kF
    public static double kP = 0.01;
    public static double kI = 0.0;
    public static double kD = 0.0;
    public static double kF = 0.0;

    public static double targetVelocity;

    private final TelemetryManager panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
    private final ElapsedTime timer = new ElapsedTime();

    @Override
    public void init() {
        motor = hardwareMap.get(DcMotorEx.class, MOTORNAME);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        timer.reset();

    }

    @Override
    public void loop() {
        PIDFCoefficients pidf = new PIDFCoefficients(kP, kI, kD, kF);
        motor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);

        motor.setVelocity(targetVelocity);

        updateSignals();

    }

    private void updateSignals() {
        double t = timer.seconds();
        double curVelocity = motor.getVelocity();
        double error = targetVelocity - curVelocity;

        panelsTelemetry.addData("TargetVelocity", targetVelocity);
        panelsTelemetry.addData("ActualVelocity", curVelocity);
        panelsTelemetry.addData("Error", error);
        panelsTelemetry.addData("MaxVelocity", motor.getMotorType().getAchieveableMaxTicksPerSecond());
        panelsTelemetry.update();
    }
}
