package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Spindexer {
    // the spindexer is a motor driven 3 chamber ball storage
    /* this code aims to be a general class that can:
    - get the position of the spindexer - Done

    - get the state of each position of the spindexer (empty, purple, or green) - TODO

    - spin the spindexer from position to position - Done In theory needs testing and can be done better TODO

    - make spindexer move automatically to an empty position if available when intaking - TODO

    - have both encoders and timing for knowing the position [optional] - TODO

     */
    public DcMotor spindexer; //Name motor accordingly (need to figure out how to make it work with any motor)
    private String motorName = null; // this is where the hardware map name would be put maybe TESTING NEEDED
    private double ticksPerRotation = 1411;// CHANGE DEPENDING ON ROBOT
    private double targetPositionMultiple = ticksPerRotation / 3;
    // find positions of all spindexer indexes
    private double positionOne = targetPositionMultiple;
    private double positionTwo = targetPositionMultiple * 2;
    private double positionThree = targetPositionMultiple * 3;

    private final double SPEED = 0.5;
    private boolean putSpindexerDown = false;

    public int currentPosition = 1;


    // CHANGE THIS IF PRELOADING
    public int[] posStates = {-1, -1, -1}; //the array that will store what is in each slot in the spindexer ( -1 - unknown, 0-empty, 1-green, 2-purple)

    //the function that would in theory if it works allow any motor to be put in
    public void getMotor(String nameOfMotor) {
        motorName = nameOfMotor;
    }

    // initialising the motor with fresh values (resets encoder)
    public void freshInit(HardwareMap hardwareMap) {
        spindexer = hardwareMap.get(DcMotor.class, "motor2");
        spindexer.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        spindexer.setTargetPosition(0);
        spindexer.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        spindexer.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    // init motor with data values in the encoder in theory
    public void dataInit(HardwareMap hardwareMap){
        spindexer = hardwareMap.get(DcMotor.class, "motor2");
        spindexer.setTargetPosition(0);
        spindexer.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        spindexer.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
    /*
    public long getSpindexerNearest() {
        // find nearest position
        double positionRatio = spindexer.getCurrentPosition() / targetPositionMultiple;
        if (Math.round(positionRatio) == 0) {
            positionRatio = 1.0;
        }
        return Math.round(positionRatio);
    }
    */


    public boolean isAtPos(int pos) {
        if (currentPosition == pos) {
            if (!spindexer.isBusy()) {
                return true;
            }
        }
        return false;
    }

    public void GoToPos(int newPos, boolean priority) {
        // if priority is true then it will not wait for spindexer to stop moving (DONT DO THIS)
        if (!priority && (currentPosition == newPos || spindexer.isBusy())) {
            return;
        }

        int newClockwisePos = getNewClockwisePos();
        int newCounterClockwisePos = getNewCounterClockwisePos();

        if (newPos == newClockwisePos) {
            rotateClockwise();
        } else if (newPos == newCounterClockwisePos) {
            rotateCounterclockwise();
        } else {
            // Should never happen if positions are 1,2,3
            return;
        }
    }

    public void GoToPos(int newPos) { // if no second parameter default to false
        GoToPos(newPos, false);
    }
    public int getNewClockwisePos() {
        return (currentPosition % 3) + 1; // 1->2, 2->3, 3->1
    }
    public int getNewCounterClockwisePos() {
        return ((currentPosition + 1) % 3) + 1; // 1->3, 2->1, 3->2
    }
    public void rotateClockwise() {
        rotateClockwise(false);
    }
    public void rotateClockwise(boolean putDown) {
        if (spindexer.isBusy()) {
            return;
        }
        int targetPosition = spindexer.getCurrentPosition() + (int)targetPositionMultiple;
        spindexer.setPower(SPEED);
        spindexer.setTargetPosition(targetPosition);
        putSpindexerDown = putDown;
        currentPosition = getNewClockwisePos();

        /*
        // This shifts all the positions clockwise Example: {1,2,3} -> {3,1,2}
        int firstPos = positions[0];
        positions[0] = positions[2];
        positions[1] = firstPos;
        positions[2] = positions[1];
        */
    }
    public void rotateCounterclockwise() {
        rotateCounterclockwise(false);
    }
    public void rotateCounterclockwise(boolean putDown) {
        if (spindexer.isBusy()) {
            return;
        }
        int targetPosition = spindexer.getCurrentPosition() - (int)targetPositionMultiple;
        spindexer.setPower(-SPEED);
        spindexer.setTargetPosition(targetPosition);
        putSpindexerDown = putDown;

        currentPosition = getNewCounterClockwisePos();

        /*
        // This shifts all the positions counter clockwise Example: {1,2,3} -> {2,3,1}
        int firstPos = positions[0];
        positions[0] = positions[1];
        positions[1] = positions[2];
        positions[2] = firstPos;
        */
    }

    public void updateCurrentSlotState(String state) {
        if (state.equals("GREEN")) {
            posStates[currentPosition - 1] = 1;
        } else if (state.equals("PURPLE")) {
            posStates[currentPosition - 1] = 2;
        } else {
            posStates[currentPosition - 1] = -1;
        }
    }

    public boolean isSpindexerFull() {
        for (int i = 0; i < posStates.length; i++) {
            if (posStates[i] <= 0) { // if not purple or green
                return false;
            }
        }
        return true;
    }

    public void rotateEmptySlot() {
        for (int i = 0; i < posStates.length; i++) {
            if (posStates[i] <= 0) { // if not purple or green
                GoToPos(i+1);
            }
        }
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
