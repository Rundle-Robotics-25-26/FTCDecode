package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class SimpleTurret {
    public DcMotor turret;

    final int MAX_ROTATE = 270; // desperately need to tune
    final double SPEED = 0.01;
    public void Init(HardwareMap hardwareMap) {
        turret = hardwareMap.get(DcMotor.class, "turretMotor");
    }

    public void TurretControl(float left, float right) {
        int position = turret.getCurrentPosition();
        if (left > 0 && position > -MAX_ROTATE) {
            turret.setPower(-SPEED);
        }

        if (right > 0 && position < MAX_ROTATE) {
            turret.setPower(SPEED);
        }
    }
}
