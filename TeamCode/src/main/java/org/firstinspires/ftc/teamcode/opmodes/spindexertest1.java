package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class spindexertest1 {
    // the spindexer is a motor driven 3 chamber ball storage
    /* this code aims to be a general class that can:
    - get the position of the spindexer - Done

    - get the state of each position of the spindexer (empty, purple, or green) - TODO

    - spin the spindexer from position to position - Done In theory needs testing and can be done better TODO

    - make spindexer move automatically to an empty position if available when intaking - TODO

    - have both encoders and timing for knowing the position [optional] - TODO

     */
    private DcMotor Spindexer; //Name motor accordingly (need to figure out how to make it work with any motor)
    private String motorName = null; // this is where the hardware map name would be put maybe TESTING NEEDED
<<<<<<< Updated upstream
<<<<<<< Updated upstream
<<<<<<< Updated upstream
    private double ticksPerRotation = 537.6; // CHANGE DEPENDING ON ROBOT
    private double targetPositionMultiple = ticksPerRotation / 3;
=======
=======
>>>>>>> Stashed changes
=======
>>>>>>> Stashed changes
    private double ticksPerRotation = 1580; // CHANGE DEPENDING ON ROBOT
    private double targetPositionMultiple = ticksPerRotation/3;
>>>>>>> Stashed changes
    // find positions of all spindexer indexes
    private double positionOne = targetPositionMultiple;
    private double positionTwo = targetPositionMultiple * 2;
    private double positionThree = targetPositionMultiple * 3;

    private final double SPEED = 0.5;

    public int currentPosition = 1;

    private int[] posStates = {0, 0, 0}; //the array that will store what is in each slot in the spindexer (0-empty, 1-green, 2-purple)

    //the function that would in theory if it works allow any motor to be put in
    public void getMotor(String nameOfMotor) {
        motorName = nameOfMotor;
    }

    // initialising the motor with fresh values (resets encoder)
    public void freshInit(DcMotor motor) {
        Spindexer = motor;

    }

    // init motor with data values in the encoder in theory
    /*
    public void dataInt(){
        Spindexer = hardwareMap.get(DcMotor.class, motorName);

        Spindexer.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        Spindexer.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
    */
    public long getSpindexerNearest() {
        // find nearest position
        double positionRatio = Spindexer.getCurrentPosition() / targetPositionMultiple;
        if (Math.round(positionRatio) == 0) {
            positionRatio = 1.0;
        }
        return Math.round(positionRatio);
    }

    public void GoToPos(int newPos) {
        if (currentPosition == newPos || Spindexer.isBusy()) {
            return;
        }

        int newClockwisePos = (currentPosition % 3) + 1;           // 1->2, 2->3, 3->1
        int newCounterClockwisePos = ((currentPosition + 1) % 3) + 1; // 1->3, 2->1, 3->2

        if (newPos == newClockwisePos) {
            rotateClockwise();
        } else if (newPos == newCounterClockwisePos) {
            rotateCounterclockwise();
        } else {
            // Should never happen if positions are 1,2,3
            return;
        }

        currentPosition = newPos;
    }

    private void rotateClockwise() {
        int targetPosition = Spindexer.getCurrentPosition() + (int)targetPositionMultiple;
        Spindexer.setPower(SPEED);
        Spindexer.setTargetPosition(targetPosition);
    }

    private void rotateCounterclockwise() {
        int targetPosition = Spindexer.getCurrentPosition() - (int)targetPositionMultiple;
        Spindexer.setPower(-SPEED);
        Spindexer.setTargetPosition(targetPosition);
    }
    /*
    // specific for each position should make a general position
    public void spindexerToPosOne(){
        // gets the nearest spindexer position as compared to its current position
        // find nearest position
        double positionRatio =  Spindexer.getCurrentPosition() / positionOne;
        int targetPosition = (int)((double) Math.round(positionRatio) * positionOne);

        if (targetPosition == 0) {
            targetPosition = (int) positionOne;
        }
        if (targetPosition > Spindexer.getCurrentPosition()){
            Spindexer.setPower(SPEED);
            Spindexer.setTargetPosition(targetPosition);
        }
        else {
            Spindexer.setPower(-SPEED);
            Spindexer.setTargetPosition(targetPosition);
        }
    }
    public void spindexerToPosTwo(){
        // gets the nearest spindexer position as compared to its current position
        // find nearest position
        double positionRatio =  Spindexer.getCurrentPosition() / positionTwo;
        int targetPosition = (int)((double) Math.round(positionRatio) * positionTwo);
        if (targetPosition == 0) {
            targetPosition = (int) positionTwo;
        }
        if (targetPosition > Spindexer.getCurrentPosition()){
            Spindexer.setPower(SPEED);
            Spindexer.setTargetPosition(targetPosition);
        }
        else {
            Spindexer.setPower(-SPEED);
            Spindexer.setTargetPosition(targetPosition);
        }
    }
    public void spindexerToPosThree(){
        // gets the nearest spindexer position as compared to its current position
        // find nearest position
        double positionRatio =  Spindexer.getCurrentPosition() / positionThree;
        int targetPosition = (int)((double) Math.round(positionRatio) * positionThree);
        if (targetPosition == 0) {
            targetPosition = (int) positionThree;
        }
        if (targetPosition > Spindexer.getCurrentPosition()){
            Spindexer.setPower(SPEED);
            Spindexer.setTargetPosition(targetPosition);
        }
        else {
            Spindexer.setPower(-SPEED);
            Spindexer.setTargetPosition(targetPosition);
        }
    }
    */
}
