package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;


public class OpSpindexerTest1 extends OpMode {
    private final spindexertest1 spindexer = new spindexertest1();

    @Override
    public void init(){
        spindexer.getMotor("motor2");
        spindexer.freshInit();
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
        telemetry.addData("Current encoded position", spindexer.getMotorPosition());
        telemetry.update();
    }
}
