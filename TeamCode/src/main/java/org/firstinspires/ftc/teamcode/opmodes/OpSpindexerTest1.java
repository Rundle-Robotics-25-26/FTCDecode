package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Indexer;
import org.firstinspires.ftc.teamcode.Spindexer;


@TeleOp(name="Spindexer test")
public class OpSpindexerTest1 extends OpMode {
    private final Spindexer spindexer = new Spindexer();
    private final Indexer indexer = new Indexer();
    DcMotor spindexerMotor;

    @Override
    public void init(){
        spindexer.dataInit(hardwareMap);
        indexer.Init(hardwareMap, telemetry);
    }

    @Override
    public void start(){
        // idk what this does ngl, gg
    }

    @Override
    public void loop(){
        telemetry.addData("Is Spindexer busy? ", spindexer.spindexer.isBusy());
        // indexer
         if (gamepad1.dpad_down) {
            if (spindexer.spindexer.isBusy()) {
                telemetry.addData("Status: ", "Spindexer is busy");
                return;
            }
            indexer.StopSpindex();
        }
        if (indexer.canSpindex()) {
            telemetry.addData("Status: ", "Can spindex");
            telemetry.addData("Is dpad_left down", gamepad1.dpad_left);
            if(gamepad1.dpad_left){
                spindexer.rotateClockwise();
            } else if (gamepad1.dpad_right) {
                spindexer.rotateCounterclockwise();
            }
        } else {
            telemetry.addData("Status: ", "CANNOT spindex");
            if (gamepad1.dpad_left || gamepad1.dpad_right) {
                indexer.SwitchSpindex();
            }
        }


        //telemetry.addData("Nearest Spindexer Position: ",spindexer.getSpindexerNearest());
        telemetry.addData("Current position index", spindexer.currentPosition);
        if (spindexer.spindexer != null) {
            telemetry.addData("Current encoded position", spindexer.spindexer.getCurrentPosition());
        }
        telemetry.update();
    }
}
