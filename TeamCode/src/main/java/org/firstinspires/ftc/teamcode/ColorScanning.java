package org.firstinspires.ftc.teamcode;


import org.firstinspires.ftc.robotcore.external.Telemetry;

public class ColorScanning {
    private ColorSensor colorSensor;
    private Spindexer spindexer;

    private int currentScanPosition = 1; // the position which is being sensed by the color sensor
    private boolean isScanning = false;
    private final long SCAN_TIME = 500; // How long to scan in ms for ball until we know it's color

    private long arriveTime; // The system time in ms when the spindexer reached the position to scan
    private enum ScanState {
        START, MOVING, SCANNING, COMPLETE
    }
    private ScanState currentState = ScanState.START;

    private Telemetry telemetry;

    /*
    This script gets the color ball in each slot, either GREEN, PURPLE or NONE

    The procedure:
    - All slots are unknown, move to position one
    - When at position: starts scanning position for SCAN_TIME miliseconds
    - Get the predicted colorResult and set it as the result
    - Move to next slot and repeat process till all balls are scanned
     */

    public void Init(ColorSensor sensor, Spindexer spin, Telemetry tele) {
        telemetry = tele;
        colorSensor = sensor;
        spindexer = spin;
    }

    public void ScanAll() {
        // Scan any unknown slots, DONT CALL WHEN SOMETHING ELSE IS USING SPINDEXER
        HandleState();
        telemetry.addData("Current spindexer: ", "Pos1: %d, Pos2: %d, Pos3: %d", spindexer.posStates[0], spindexer.posStates[1], spindexer.posStates[2]);
    }

    public void MoveToEmptySlot() {
        // Always scan the slot and if its a color it switches until it's on an empty slot
        if (spindexer.spindexer.isBusy()) {
            //if moving don't do anything
            return;
        }

        if (!isScanning) {
            isScanning = true;
            arriveTime = System.currentTimeMillis();
        }

        currentScanPosition = spindexer.positions[0]; // This is the position at the color sensor
        boolean doneScan = Scanning();
        if (doneScan) {
            int positionState = spindexer.posStates[currentScanPosition-1];
            if (positionState == 1 || positionState == 2) { // if green or purple switch slot
                telemetry.addData("Status:", "Switching slot...");
                spindexer.GoToPos(spindexer.getNewClockwisePos());
            } else {
                telemetry.addData("Status: ", "STAYING ON EMPTY SLOT");
            }
        } else {
            telemetry.addData("Status:", "COLOR SCANNING...");
        }

        telemetry.addData("Current spindexer: ", "Pos1: %d, Pos2: %d, Pos3: %d", spindexer.posStates[0], spindexer.posStates[1], spindexer.posStates[2]);
    }


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

    public void Reset() {
        spindexer.posStates = new int[]{-1, -1, -1};
        currentState = ScanState.START;
        colorSensor.Reset();
    }
}
