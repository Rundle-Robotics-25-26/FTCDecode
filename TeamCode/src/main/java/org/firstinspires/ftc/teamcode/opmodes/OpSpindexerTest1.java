package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Spindexer;


@TeleOp(name="Spindexer test")
public class OpSpindexerTest1 extends OpMode {
    private final Spindexer spindexer = new Spindexer();
    DcMotor spindexerMotor;

    @Override
    public void init(){
        spindexer.freshInit(hardwareMap);
    }

    @Override
    public void start(){
        // idk what this does ngl, gg
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
        if (spindexer.spindexer != null) {
            telemetry.addData("Current encoded position", spindexer.spindexer.getCurrentPosition());
        }
        telemetry.update();
    }
}
