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
        // indexer
         if (gamepad1.dpad_down) {
            if (spindexer.spindexer.isBusy()) {
                return;
            }
            indexer.StopSpindex();
        }

        if (gamepad1.dpad_left || gamepad1.dpad_right) {
            indexer.SwitchSpindex();
        }
        if (!indexer.canSpindex()) {
            return;
        }
        if(gamepad1.dpad_left){
            spindexer.rotateClockwise();
        } else if (gamepad1.dpad_right) {
            spindexer.rotateCounterclockwise();
        }

        //telemetry.addData("Nearest Spindexer Position: ",spindexer.getSpindexerNearest());
        telemetry.addData("Current position index", spindexer.currentPosition);
        if (spindexer.spindexer != null) {
            telemetry.addData("Current encoded position", spindexer.spindexer.getCurrentPosition());
        }
        telemetry.update();
    }
}
