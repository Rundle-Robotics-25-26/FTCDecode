package org.firstinspires.ftc.teamcode.opmodes.tests.tuning;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

public class BackupPIDFTuner extends OpMode {
    /*
    final String MOTORNAME = "shooter"; // set this to the motor you want to tune

    DcMotorEx motor;

    // probably only need to tune kP and kF
    public static double kP = 0.01;
    public static double kI = 0.0;
    public static double kD = 0.0;
    public static double kF = 0.0;

    double targetVelocity;

    char[] stepCoefficients = {'P', 'I', 'D', 'F'};
    int curStepSize = 0;
    int curStepVelocity = 0;
    int curStepCoefficients = 0;

    private TelemetryManager panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
*/
    @Override
    public void init() {
        /*
        motor = hardwareMap.get(DcMotorEx.class, MOTORNAME);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        initStepVelocities();

         */
    }
    /*
    public void initStepVelocities() {
        double maxVelocity = motor.getMotorType().getAchieveableMaxRPMFraction();
        for (int i = 0; i < stepVelocities.length; i++) {
            stepVelocities[i] *= maxVelocity;
        }
    }
*/
    @Override
    public void loop() {
/*
        if (gamepad1.dpadUpWasPressed() || gamepad1.dpadDownWasPressed()) {
            int upDown = gamepad1.dpadUpWasPressed() ? 1 : -1;
            curStepSize = (curStepSize + upDown) % stepSizes.length;
        }
        if (gamepad1.dpadLeftWasPressed() || gamepad1.dpadRightWasPressed()) {
            int upDown = gamepad1.dpadRightWasPressed() ? 1 : -1;
            curStepCoefficients = (curStepCoefficients + upDown) % stepCoefficients.length;
        }
        if (gamepad1.leftBumperWasPressed()|| gamepad1.rightBumperWasPressed()) {
            int upDown = gamepad1.rightBumperWasPressed() ? 1 : -1;
            curStepVelocity = (curStepVelocity + upDown) % stepVelocities.length;
        }

        if (gamepad1.circleWasPressed()) {
            targetVelocity = stepVelocities[curStepVelocity];
        }

        if (gamepad1.crossWasPressed() || gamepad1.triangleWasPressed()) {
            int direction = gamepad1.triangleWasPressed() ? 1 : -1;
            double amount = direction * stepSizes[curStepSize];
            switch (stepCoefficients[curStepCoefficients]) {
                case 'P':
                    kP += amount;
                    break;
                case 'I':
                    kI += amount;
                    break;
                case 'D':
                    kD += amount;
                    break;
                case 'F':
                    kF += amount;
                    break;
            }
        }


        PIDFCoefficients pidf = new PIDFCoefficients(kP, kI, kD, kF);
        motor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);

        motor.setVelocity(targetVelocity);

        double curVelocity = motor.getVelocity();
        double error = targetVelocity - curVelocity;

        panelsTelemetry.update();

 */
    }

}
