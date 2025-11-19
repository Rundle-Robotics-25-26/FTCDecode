package org.firstinspires.ftc.teamcode;


import org.firstinspires.ftc.robotcore.external.Telemetry;

public class ColorScanning {
    private ColorSensor colorSensor;
    private Spindexer spindexer;
    /*
    private enum ScanState {
        START, MOVING, SCANNING, COMPLETE
    }
    private ScanState currentState = ScanState.START;
    */

    private Telemetry telemetry;


    public void Init(ColorSensor sensor, Spindexer spin, Telemetry tele) {
        telemetry = tele;
        colorSensor = sensor;
        spindexer = spin;
    }

    public void ScanAll() {
        // Scan any unknown slots, DONT CALL WHEN SOMETHING ELSE IS USING SPINDEXER
        //HandleState();
        telemetry.addData("Current spindexer: ", "Pos1: %d, Pos2: %d, Pos3: %d", spindexer.posStates[0], spindexer.posStates[1], spindexer.posStates[2]);
    }

    public void MoveToEmptySlot() {
        // Always scan the slot and if it sees a ball it rotates clockwise unless all slots are full
        telemetry.addData("Current Spindexer position: ", spindexer.currentPosition);

        if (spindexer.spindexer.isBusy()) {
            telemetry.addData("Status:", "Rotating slot...");
            //if moving don't do anything
            return;
        }

        if (spindexer.isSpindexerFull()) {
            telemetry.addData("Status: ", "Spindexer is full!");
        }

        String currentColor = colorSensor.BallDetermineUpdate();

        if (currentColor.equals("GREEN") || currentColor.equals("PURPLE")) {
            spindexer.updateCurrentSlotState(currentColor);
            colorSensor.Reset(); // need to do this everytime switch slot
            spindexer.rotateEmptySlot();
        } else {
            telemetry.addData("Status:", "Unknown color value");
        }
    }

    //idk why i made it like this, too complicated
    /*

    private void HandleState() {
        switch(currentState) {
            case START:
                LookForScan();
                telemetry.addData("Status: ", "Starting scan");
                break;
            case MOVING:
                IsMovingDone();
                telemetry.addData("Status: ", "Moving to position %d", currentScanPosition);
                break;
            case SCANNING:
                Scanning();
                telemetry.addData("Status: ", "Scanning position %d", currentScanPosition);
            case COMPLETE:
                LookForScan();
                telemetry.addData("Status: ", "Done scan");
                break;
        }
    }
    private boolean Scanning() {
        // return true if done scan
        String colorResult = colorSensor.BallDetermineUpdate(); // "UNCERTAIN", "GREEN" or "PURPLE"
        if (colorResult.equals("GREEN")) { // confident its green so we dont have to wait the full SCAN_TIME
            spindexer.posStates[currentScanPosition-1] = 1;
            DoneScan();
            return true;
        }
        else if (colorResult.equals("PURPLE")) {
            spindexer.posStates[currentScanPosition-1] = 2;
            DoneScan();
            return true;
        }
        else if (System.currentTimeMillis() - arriveTime >= SCAN_TIME) {
            // done scan time, there is no ball in slot
            spindexer.posStates[currentScanPosition-1] = 0;
            DoneScan();
            return true;
        }
        else {
            return false;
        }
    }
    private void IsMovingDone() {
        if (spindexer.isAtPos(currentScanPosition)) {
            arriveTime = System.currentTimeMillis();
            currentState = ScanState.SCANNING;
        }
    }
    private void LookForScan() {
        int unknownSlot = FindSlot(-1);
        if (unknownSlot != -1) { // if there is an unknown slot
            currentState = ScanState.MOVING;
            currentScanPosition = unknownSlot;
            spindexer.GoToPos(unknownSlot);
        }
        else {
            currentState = ScanState.COMPLETE;

            int emptySlot = FindSlot(0);
            if (emptySlot != -1) {
                // automatically go to empty slot
                spindexer.GoToPos(emptySlot);
            }
        }
    }
    private int FindSlot(int targetState) {
        // -1, 0, 1, 2
        int i = 1;
        for (int posState : spindexer.posStates) {
            if (posState == targetState) {
                return i;
            }
            i++;
        }
        return -1; // -1 if no unknown positions
    }

    void DoneScan() {
        currentState = ScanState.COMPLETE;
        isScanning = false;
        colorSensor.Reset();
    }
    */
    /*
    public void Reset() {
        spindexer.posStates = new int[]{-1, -1, -1};
        currentState = ScanState.START;
        colorSensor.Reset();
    }
    */
}
