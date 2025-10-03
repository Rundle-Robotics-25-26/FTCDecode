package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;


@TeleOp(name="Spindexer test")
public class OpSpindexerTest1 extends OpMode {
    private final spindexertest1 spindexer = new spindexertest1();
    DcMotor spindexerMotor;

    @Override
    public void init(){
        spindexerMotor = hardwareMap.get(DcMotor.class, "motor2");
        spindexerMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        spindexerMotor.setTargetPosition(0);
        spindexerMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        spindexerMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        spindexer.freshInit(spindexerMotor);
    }

    @Override
    public void start(){
        // idk what this does ngl
    }

    @Override
    public void loop(){
        if(gamepad1.cross) {
            spindexer.GoToPos(1);
        }
        else if(gamepad1.circle){
            spindexer.GoToPos(2);
        }
        else {
            spindexer.GoToPos(3);
        }


        //telemetry.addData("Nearest Spindexer Position: ",spindexer.getSpindexerNearest());
        telemetry.addData("Current position index", spindexer.currentPosition);
        telemetry.addData("Current encoded position", spindexerMotor.getCurrentPosition());
        telemetry.update();
    }
}
