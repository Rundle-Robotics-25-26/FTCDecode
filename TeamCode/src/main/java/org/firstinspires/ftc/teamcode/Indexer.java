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

    private Servo indexer;

    // FIND THESE VALUES WITH THE TEST UPDATE
    private final double INDEXER_START = 0.5;
    private final double INDEXER_END = 0.7;

    // New movement system variables
    public enum State { IDLE, INDEXING_UP, INDEXING_DOWN, SPINNING}
    public State currentState = State.IDLE;
    private long sequenceStartTime = 0;
    private long movingStartTime = 0;
    private long resetStartTime = 0;
    private int sequencePhase = 0;

    // Movement parameters (adjust these based on testing)
    private static final long BASE_MOVE_TIME = 500;  // ms//was 800
    private static final long ARM_MOVE_TIME = 500;   // ms;
    private static final long SPINDEX_MOVE_TIME = 200;
    private static final long ROTATE_MOVE_TIME = 500;
    private static final long PHASE_DELAY = 200;      // ms between phases
    private static final long AUTO_WAIT_TIME = 500; // ms to wait between open and close
    private static final long RESET_TIME = 500;

    // Start and target positions for smooth movement
    private double baseStartPos, armStartPos;
    private double baseTargetPos, armTargetPos;

    private boolean shootingAndSpinning = false;


    private RobotTimer indexerTimer = new RobotTimer(4000);
    private RobotTimer spindexerSpinningTimer = new RobotTimer(1000);
    // Track which sequence we're running



    public void Init(HardwareMap hardware, Telemetry tele) {
        hardwareMap = hardware;
        telemetry = tele;

        indexer = hardwareMap.get(Servo.class, "indexer");

        // Initialize servos to start positions
        indexer.setPosition(INDEXER_START);
    }

    public void TestUpdate(int movement) {
        double SPEED = 0.001;

        if (movement > 0) {
            indexer.setPosition(indexer.getPosition() + SPEED);
        } else if (movement < 0) {
            indexer.setPosition(indexer.getPosition() - SPEED);
        }

        telemetry.addData("Current indexer position", indexer.getPosition());
        telemetry.update();
    }

    public void Index() {
        if (currentState == State.IDLE) {
            indexer.setPosition(INDEXER_END);
            indexerTimer.start();
            currentState = State.INDEXING_UP;
        } else {
            telemetry.addData("CANT INDEX", "YET");
        }
    }

    public void ShootAndSpin() {
        if (currentState == State.IDLE) {
            shootingAndSpinning = true;
            Index();
        } else {
            telemetry.addData("CANT SHOOT AND SPIN", "YET");
        }
    }
    public void Update() {
         switch (currentState) {
             case IDLE:
                 // do nothing?
                 break;
             case INDEXING_UP:
                 if (indexerTimer.IsDone()) {
                     indexer.setPosition(INDEXER_START);
                     indexerTimer.start();
                     currentState = State.INDEXING_DOWN;
                 }
                 break;
             case INDEXING_DOWN:
                 if (indexerTimer.IsDone()) {
                     if (shootingAndSpinning) {
                         currentState = State.SPINNING;
                         spindexerSpinningTimer.start();
                         
                     } else {
                         currentState = State.IDLE;
                     }
                 }
         }

        telemetry.addData("Current indexer position", indexer.getPosition());
        telemetry.addData("State", currentState);
    }

}