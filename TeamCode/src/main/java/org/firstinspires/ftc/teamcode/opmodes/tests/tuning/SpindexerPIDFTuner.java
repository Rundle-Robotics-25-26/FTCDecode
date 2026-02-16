package org.firstinspires.ftc.teamcode.opmodes.tests.tuning;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

@Configurable
@TeleOp(name = "Spinexer PIDF Tuner", group="tuning")
public class SpindexerPIDFTuner extends OpMode {

    public static double P = 0;
    public static double I = 0;
    public static double D = 0;
    public static double F = 0;


    public static double TICKS_PER_120 = 128.1667;
    public static double MAX_POWER = 0.5;
    public static int TARGET_TOL = 2; // how big the set target position deadzone is

    private final TelemetryManager panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
    private final ElapsedTime timer = new ElapsedTime();

    private DcMotorEx spindexer;

    private int zeroCount = 0;      // encoder tick value for slot 0 at intake
    private double accum = 0.0;     // cumulative (floating) encoder target; drift-free rounding
    private int targetCounts = 0;   // last set RUN_TO_POSITION target


    @Override
    public void init() {

        spindexer = hardwareMap.get(DcMotorEx.class, "spindexer");
        spindexer.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        spindexer.setTargetPosition(0);
        spindexer.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        spindexer.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        PIDFCoefficients pidf = new PIDFCoefficients(P, I, D, F);
        spindexer.setPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION, pidf);
        spindexer.setTargetPositionTolerance(TARGET_TOL);

        zeroCount = spindexer.getCurrentPosition();
        accum = 0.0;
        targetCounts = spindexer.getCurrentPosition();
        spindexer.setTargetPosition(targetCounts);
        spindexer.setPower(MAX_POWER);
    }

    @Override
    public void loop() {
        double t = timer.seconds();

        PIDFCoefficients newPidf = new PIDFCoefficients(P, I, D, F);
        spindexer.setPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION, newPidf);

        if (gamepad1.rightBumperWasPressed()) {
            spinClockwise();
        }
        if (gamepad1.dpadDownWasPressed()) {
            zeroNow();
        }
        panelsTelemetry.addData("pos", spindexer.getCurrentPosition());
        panelsTelemetry.addData("target", targetCounts);
        panelsTelemetry.addData("error", targetCounts - spindexer.getCurrentPosition());
        panelsTelemetry.update();
    }

    public void zeroNow() {
        zeroCount = spindexer.getCurrentPosition();
        accum = 0.0;
        setTarget(spindexer.getCurrentPosition());
    }

    public void spinClockwise() {
        accum += TICKS_PER_120;
        setTarget((int)Math.rint(zeroCount + accum)); // idk math.rint should minimize drift
    }

    public void setTarget(int t) {
        targetCounts = t;
        spindexer.setTargetPosition(targetCounts);
        spindexer.setPower(MAX_POWER);
    }

}
