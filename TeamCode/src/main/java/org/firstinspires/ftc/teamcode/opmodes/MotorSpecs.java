package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class MotorSpecs {
    Telemetry telemetry;
    public void Init(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    public void DebugSpecs(String debugName, DcMotor motor) {
        MotorConfigurationType mct = motor.getMotorType();

        telemetry.addLine("☠☠☠ Motor specs for " + debugName + " ☠☠☠");

        telemetry.addData("Ticks / Rev", mct.getTicksPerRev());
        telemetry.addData("Max RPM (theoretical)", mct.getMaxRPM());
        telemetry.addData("Gearing", mct.getGearing());
        telemetry.addData("Achievable RPM fraction", mct.getAchieveableMaxRPMFraction());
        telemetry.addData("Max ticks/sec (real)",
                mct.getAchieveableMaxTicksPerSecondRounded());
        telemetry.addData("Has hub velocity PID",
                mct.hasExpansionHubVelocityParams());

        telemetry.addLine("☠☠☠☠☠☠☠");
    }

}
