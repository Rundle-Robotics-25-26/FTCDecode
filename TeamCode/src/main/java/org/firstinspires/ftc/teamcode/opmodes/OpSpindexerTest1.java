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
            spindexer.spindexerToPosOne();
        }
        else if(gamepad1.circle){
            spindexer.spindexerToPosTwo();
        }
        else {
            spindexer.spindexerToPosThree();
        }


        telemetry.addData("Nearest Spindexer Position: ",spindexer.getSpindexerNearest());
    }
}
