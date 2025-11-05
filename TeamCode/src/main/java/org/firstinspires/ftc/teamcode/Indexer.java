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
    final double ARM_START = 0;
    final double ARM_INDEX = 0.5;
    final double BASE_START = 0;
    final double BASE_INDEX = 0.5;

    private enum State { IDLE, MOVING_BASE, MOVING_ARM }
    private State currentState = State.IDLE;
    private long moveStartTime = 0;
    private long currentMoveDuration = 0;
    private Queue<MoveCommand> moveQueue = new LinkedList<>();

    private class MoveCommand {
        String indexer;
        double position;
        long duration;

        MoveCommand(String indexer, double position, long duration) {
            this.indexer = indexer;
            this.position = position;
            this.duration = duration;
        }
    }

    public void Init(HardwareMap hardware, Telemetry tele) {
        hardwareMap = hardware;
        telemetry = tele;

        baseIndexer = hardwareMap.get(Servo.class, "base");
        armIndexer = hardwareMap.get(Servo.class, "arm");
    }
    public void TestUpdate(int armMovement, int baseMovement) {
        double SPEED = 0.0001;

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
        switch (currentState) {
            case IDLE:
                // if something in the queue do the move
                if (!moveQueue.isEmpty()) {
                    MoveCommand nextMove = moveQueue.poll();
                    executeMove(nextMove);
                }

                // toggle indexer open/closed
                if (pressed) {
                    if (!indexerPressed) {
                        indexerPressed = true;

                        if (!indexerOpen) {
                            Open();
                        } else {
                            Closed();
                        }
                        indexerOpen = !indexerOpen;
                    }
                } else {
                    indexerPressed = false;
                }
                break;
            case MOVING_BASE:
            case MOVING_ARM:
                if (System.currentTimeMillis() - moveStartTime >= currentMoveDuration) {
                    currentState = State.IDLE;
                }
                break;
        }



        telemetry.addData("Current base position", baseIndexer.getPosition());
        telemetry.addData("Current arm position", armIndexer.getPosition());
        telemetry.update();
    }

    public void Open() {
        if (indexerOpen) {
            return;
        }

        moveQueue.add(new MoveCommand("base", BASE_INDEX, 300)); // You can change how long to wait before doing the next move
        moveQueue.add(new MoveCommand("arm", ARM_INDEX, 300));
    }

    public void Closed() {
        if (!indexerOpen) {
            return;
        }

        // Queue movements instead of executing immediately
        moveQueue.add(new MoveCommand("base", BASE_START, 300));
        moveQueue.add(new MoveCommand("arm", ARM_START, 300));
    }

    private void executeMove(MoveCommand move) {
        if (move.indexer.equals("base")) {
            baseIndexer.setPosition(move.position);
            currentState = State.MOVING_BASE;
        } else if (move.indexer.equals("arm")) {
            armIndexer.setPosition(move.position);
            currentState = State.MOVING_ARM;
        }

        moveStartTime = System.currentTimeMillis();
        currentMoveDuration = move.duration;
    }


}
