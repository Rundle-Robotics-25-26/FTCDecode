package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.LinkedList;
import java.util.Queue;

public class Indexer {
    private HardwareMap hardwareMap;
    private Telemetry telemetry;

    private boolean indexerOpen = false;
    private boolean indexerPressed = false;

    private Servo baseIndexer, armIndexer;

    // FIND THESE VALUES WITH THE TEST UPDATE
    final double ARM_START = 0.1317;
    final double ARM_INDEX = 0.4967;
    final double ARM_SPINDEX = 0.4428;
    final double BASE_START = 0.13;
    final double BASE_INDEX = 0.3961;
    final double BASE_SPINDEX = 0.2628;

    // New movement system variables
    private enum State { IDLE, IN_SEQUENCE, WAITING }
    private State currentState = State.IDLE;
    private long sequenceStartTime = 0;
    private long movingStartTime = 0;
    private long resetStartTime = 0;
    private int sequencePhase = 0;

    // Movement parameters (adjust these based on testing)
    private static final long BASE_MOVE_TIME = 400;  // ms//was 800
    private static final long ARM_MOVE_TIME = 400;   // ms;
    private static final long SPINDEX_MOVE_TIME = 200;
    private static final long ROTATE_MOVE_TIME = 500;
    private static final long PHASE_DELAY = 50;      // ms between phases
    private static final long AUTO_WAIT_TIME = 1000; // ms to wait between open and close
    private static final long RESET_TIME = 5000;

    // Start and target positions for smooth movement
    private double baseStartPos, armStartPos;
    private double baseTargetPos, armTargetPos;

    // Track which sequence we're running
    private boolean isOpeningSequence = false;
    private boolean autoCycleEnabled = false;
    private boolean isSpindexerIndexed = false;
    private boolean isMovingSpindexer = false;
    private boolean resetting = false;

    public void Init(HardwareMap hardware, Telemetry tele) {
        hardwareMap = hardware;
        telemetry = tele;

        baseIndexer = hardwareMap.get(Servo.class, "base");
        armIndexer = hardwareMap.get(Servo.class, "arm");

        // Initialize servos to start positions
        baseIndexer.setPosition(BASE_START);
        armIndexer.setPosition(ARM_START);
    }

    public void TestUpdate(int armMovement, int baseMovement) {
        double SPEED = 0.001;

        if (armMovement > 0) {
            armIndexer.setPosition(armIndexer.getPosition() + SPEED);
        } else if (armMovement < 0) {
            armIndexer.setPosition(armIndexer.getPosition() - SPEED);
        }

        if (baseMovement > 0) {
            baseIndexer.setPosition(baseIndexer.getPosition() + SPEED);
        } else if (baseMovement < 0) {
            baseIndexer.setPosition(baseIndexer.getPosition() - SPEED);
        }
        telemetry.addData("Current base position", baseIndexer.getPosition());
        telemetry.addData("Current arm position", armIndexer.getPosition());
        telemetry.update();
    }

    public void Update(boolean pressed) {
        if (isSpindexerIndexed) {
            telemetry.addData("State", "MOVING Spindexer cause spindexing");
            return;
        }
        if (currentState == State.IN_SEQUENCE || currentState == State.WAITING) {
            updateSequence();
        } else {
            handleIdleState(pressed);
        }

        telemetry.addData("Current base position", baseIndexer.getPosition());
        telemetry.addData("Current arm position", armIndexer.getPosition());
        telemetry.addData("State", currentState);
        telemetry.addData("Phase", sequencePhase);
        telemetry.addData("Sequence Type", isOpeningSequence ? "OPENING" : "CLOSING");
        telemetry.addData("Auto Cycle", autoCycleEnabled ? "ACTIVE" : "INACTIVE");
        telemetry.addData("Indexer Open", indexerOpen);
        telemetry.update();
    }
    public void initializeBlockPosition() {
        isSpindexerIndexed = true;
        sequenceStartTime = System.currentTimeMillis();
        baseIndexer.setPosition(BASE_SPINDEX);
        armIndexer.setPosition(ARM_SPINDEX);
    }
    public boolean isReadyToSpindex() {
        return currentState == State.IDLE;
    }

    public boolean isInBlockingPosition() {
        long elapsed = System.currentTimeMillis() - sequenceStartTime;
        if (isSpindexerIndexed && elapsed > SPINDEX_MOVE_TIME) { // the spindexer is at the position and are ready to switch slots
            return true;
        }
        return false;
    }

    public void rotateSpindexer(boolean clockwise, Spindexer spindexer) {
        isMovingSpindexer = true;
        movingStartTime = System.currentTimeMillis();
        if (clockwise) {
            spindexer.rotateClockwise();
        } else {
            spindexer.rotateCounterclockwise();
        }
    }

    public void stopSpindexer() {
        isMovingSpindexer = false;
        isSpindexerIndexed = false;
        baseIndexer.setPosition(BASE_START);
        armIndexer.setPosition(ARM_START);
    }
    public void spindexerUpdate(boolean clockwise, Spindexer spindexer) {
        if (isInBlockingPosition()) {
            if (isMovingSpindexer) {
                long movingElapsed = System.currentTimeMillis() - movingStartTime;
                if (movingElapsed > ROTATE_MOVE_TIME) {
                    stopSpindexer();
                }
            } else {
                rotateSpindexer(clockwise, spindexer);
            }
        } else {
            telemetry.addData("Status: ", "Still moving to blocking position");
        }
    }
    public void spindex(boolean clockwise, Spindexer spindexer) {
        // Is indexer shooting?
        if (!isReadyToSpindex()) {
            telemetry.addData("Status: ", "Must wait for indexer to be idle to spindex.");
            return;
        }

        // Is indexer moving
        if (!isSpindexerIndexed) {
            telemetry.addData("Status: ", "Starting indexer moving to blocking position");
            initializeBlockPosition();
        }
    }


    private void handleIdleState(boolean pressed) {
        // Toggle indexer open/closed
        if (pressed) {
            if (!indexerPressed) {
                indexerPressed = true;

                if (!indexerOpen) {
                    // Start auto cycle: open, wait, then close
                    startAutoCycle();
                } else {
                    // Manual close if already open
                    startCloseSequence();
                    autoCycleEnabled = false;
                }
                // DON'T toggle indexerOpen here - let the sequences handle it
            }
        } else {
            indexerPressed = false;
        }
    }

    private void startAutoCycle() {
        autoCycleEnabled = true;
        startOpenSequence();
    }

    private void startOpenSequence() {
        currentState = State.IN_SEQUENCE;
        sequenceStartTime = System.currentTimeMillis();
        sequencePhase = 0;
        isOpeningSequence = true;

        // Store start positions for interpolation
        baseStartPos = baseIndexer.getPosition();
        armStartPos = armIndexer.getPosition();

        baseTargetPos = BASE_INDEX;
        armTargetPos = ARM_INDEX;

        // Update state to reflect that we're opening
        indexerOpen = true;
    }

    private void startCloseSequence() {
        currentState = State.IN_SEQUENCE;
        sequenceStartTime = System.currentTimeMillis();
        sequencePhase = 0;
        isOpeningSequence = false;

        baseStartPos = baseIndexer.getPosition();
        armStartPos = armIndexer.getPosition();

        baseTargetPos = BASE_START;
        armTargetPos = ARM_START;

        // Update state to reflect that we're closing
        indexerOpen = false;
    }

    private void updateSequence() {
        long elapsed = System.currentTimeMillis() - sequenceStartTime;

        if (currentState == State.WAITING) {
            // Waiting phase between open and close
            if (elapsed >= AUTO_WAIT_TIME) {
                // Wait time complete, start closing sequence
                startCloseSequence();
            }
            return;
        }

        if (isOpeningSequence) {
            // OPENING: Base first, then arm
            updateOpeningSequence(elapsed);
        } else {
            // CLOSING: Arm first, then base
            updateClosingSequence(elapsed);
        }
    }

    private void updateOpeningSequence(long elapsed) {
        // Opening sequence: Base first, then arm
        switch (sequencePhase) {
            case 0: // Move base servo first
                if (elapsed <= BASE_MOVE_TIME) {
                    double progress = (double) elapsed / BASE_MOVE_TIME;
                    double currentPos = interpolate(baseStartPos, baseTargetPos, progress);
                    baseIndexer.setPosition(currentPos);
                } else {
                    baseIndexer.setPosition(baseTargetPos);
                    sequencePhase = 1;
                    sequenceStartTime = System.currentTimeMillis(); // Reset timer for next phase
                }
                break;

            case 1: // Brief delay before arm movement
                if (elapsed >= PHASE_DELAY) {
                    sequencePhase = 2;
                    sequenceStartTime = System.currentTimeMillis();
                }
                break;

            case 2: // Move arm servo second
                if (elapsed <= ARM_MOVE_TIME) {
                    double progress = (double) elapsed / ARM_MOVE_TIME;
                    double currentPos = interpolate(armStartPos, armTargetPos, progress);
                    armIndexer.setPosition(currentPos);
                } else {
                    armIndexer.setPosition(armTargetPos);

                    if (autoCycleEnabled) {
                        // Start waiting period before auto-close
                        currentState = State.WAITING;
                        sequenceStartTime = System.currentTimeMillis();
                    } else {
                        currentState = State.IDLE;
                        sequencePhase = 0;
                    }
                }
                break;
        }
    }

    private void updateClosingSequence(long elapsed) {
        // Closing sequence: Arm first, then base
        switch (sequencePhase) {
            case 0: // Move arm servo first
                if (elapsed <= ARM_MOVE_TIME) {
                    double progress = (double) elapsed / ARM_MOVE_TIME;
                    double currentPos = interpolate(armStartPos, armTargetPos, progress);
                    armIndexer.setPosition(currentPos);
                } else {
                    armIndexer.setPosition(armTargetPos);
                    sequencePhase = 1;
                    sequenceStartTime = System.currentTimeMillis(); // Reset timer for next phase
                }
                break;

            case 1: // Brief delay before base movement
                if (elapsed >= PHASE_DELAY) {
                    sequencePhase = 2;
                    sequenceStartTime = System.currentTimeMillis();
                }
                break;

            case 2: // Move base servo second
                if (elapsed <= BASE_MOVE_TIME) {
                    double progress = (double) elapsed / BASE_MOVE_TIME;
                    double currentPos = interpolate(baseStartPos, baseTargetPos, progress);
                    baseIndexer.setPosition(currentPos);
                } else {
                    baseIndexer.setPosition(baseTargetPos);
                    currentState = State.IDLE;
                    sequencePhase = 0;
                    autoCycleEnabled = false; // Reset auto cycle
                }
                break;
        }
    }

    private double interpolate(double start, double end, double progress) {
        // Use easing function for smoother movement
        progress = Math.max(0, Math.min(1, progress));
        // Simple linear interpolation
        return start + (end - start) * progress;
    }

    // Keep your original methods for compatibility
    public void Open() {
        if (!indexerOpen && currentState == State.IDLE) {
            startOpenSequence();
        }
    }

    public void Closed() {
        if (indexerOpen && currentState == State.IDLE) {
            startCloseSequence();
        }
    }

    // New method for auto cycle
    public void AutoCycle() {
        if (!indexerOpen && currentState == State.IDLE) {
            startAutoCycle();
        }
    }
}