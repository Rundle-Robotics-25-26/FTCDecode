package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Configurable
public class Spindexer {
    public DcMotor spindexer;
    private String motorName = "motor2";

    // PID Coefficients
    //FIXME: TUNE THESE VALUES
    public static double Kp = 0.0; // Proportional - Start with small value
    public static double Ki = 0.0;   // Integral - Start very small
    public static double Kd = 0.0; // Derivative - Start small

    private double ticksPerRotation = 384.5;
    private double targetPositionMultiple = ticksPerRotation / 3;
    public double positionOne = targetPositionMultiple;
    public double positionTwo = targetPositionMultiple * 2;
    public double positionThree = targetPositionMultiple * 3;

    // PID Variables
    private double lastError = 0;
    private double integralSum = 0;
    private final Timer pidTimer = new Timer();

    // PID tuning parameters
    private static final double POSITION_TOLERANCE = 5.0; // ticks - how close is "close enough"
    private static final double MAX_INTEGRAL = 0.3; // Prevent integral windup
    private static final double MAX_POWER = 0.8; // Maximum motor power
    private static final double MIN_POWER = -0.8; // Minimum motor power

    // Target position variable for PID
    private int targetPosTicks = 0;

    public int currentPosition = 1;
    public int[] posStates = {-1, -1, -1};

    public void getMotor(String nameOfMotor) {
        motorName = nameOfMotor;
    }

    // Initializing the motor with fresh values (resets encoder)
    public void freshInit(HardwareMap hardwareMap) {
        spindexer = hardwareMap.get(DcMotor.class, motorName);
        spindexer.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        spindexer.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        spindexer.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        spindexer.setDirection(DcMotorSimple.Direction.REVERSE);

        // Initialize PID
        targetPosTicks = 0;
        lastError = 0;
        integralSum = 0;
        pidTimer.resetTimer();
    }
    public void dataInit(HardwareMap hardwareMap){
        spindexer = hardwareMap.get(DcMotor.class, motorName);
        spindexer.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        spindexer.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        spindexer.setDirection(DcMotorSimple.Direction.REVERSE);

        // Initialize PID
        targetPosTicks = spindexer.getCurrentPosition();
        lastError = 0;
        integralSum = 0;
        pidTimer.resetTimer();
    }

    public void setTargetPosition(int ticks) {
        targetPosTicks = ticks;
        // Reset integral when target changes to prevent windup
        integralSum = 0;
    }

    // Main PID update loop - MUST be called repeatedly in your OpMode loop
    public void update(){
        double currentPos = spindexer.getCurrentPosition();
        double error = targetPosTicks - currentPos;

        // Get time delta
        double deltaTime = pidTimer.getElapsedTime();

        // Prevent division by zero on first run
        if (deltaTime == 0) {
            pidTimer.resetTimer();
            return;
        }

        // Calculate Derivative
        double derivative = (error - lastError) / deltaTime;

        // Calculate Integral with anti-windup
        // Only accumulate if we're not already at maximum integral
        if (Math.abs(integralSum) < MAX_INTEGRAL) {
            integralSum += error * deltaTime;
        }

        // Reset integral if we're very close to target (prevents oscillation)
        if (Math.abs(error) < POSITION_TOLERANCE) {
            integralSum = 0;
        }

        // Calculate PID Power
        double power = (Kp * error) + (Ki * integralSum) + (Kd * derivative);

        // Clamp power to safe limits
        power = Math.max(MIN_POWER, Math.min(MAX_POWER, power));

        // Apply power to motor
        spindexer.setPower(power);

        // Update for next iteration
        lastError = error;
        pidTimer.resetTimer();
    }

    // Check if we're at the target position within tolerance
    public boolean isAtTarget() {
        return Math.abs(targetPosTicks - spindexer.getCurrentPosition()) < POSITION_TOLERANCE;
    }

    public boolean isAtPos(int pos) {
        return currentPosition == pos && isAtTarget();
    }

    public void GoToPos(int newPos, boolean priority) {
        if (!priority && currentPosition == newPos && isAtTarget()) {
            return;
        }

        int targetTicks = 0;

        // Calculate the ticks based on the new position index
        if (newPos == 1) targetTicks = (int)positionOne;
        else if (newPos == 2) targetTicks = (int)positionTwo;
        else if (newPos == 3) targetTicks = (int)positionThree;

        // Set the target for the PID loop to follow
        setTargetPosition(targetTicks);
        currentPosition = newPos;
    }

    public void GoToPos(int newPos) {
        GoToPos(newPos, false);
    }

    public int getNewClockwisePos() {
        return (currentPosition % 3) + 1; // 1->2, 2->3, 3->1
    }

    public int getNewCounterClockwisePos() {
        return ((currentPosition + 1) % 3) + 1; // 1->3, 2->1, 3->2
    }

    public void rotateClockwise() {
        int newPos = getNewClockwisePos();
        GoToPos(newPos);
    }

    public void rotateCounterclockwise() {
        int newPos = getNewCounterClockwisePos();
        GoToPos(newPos);
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
                return; // Only rotate to the first empty slot found
            }
        }
    }

    // Getter methods for debugging/tuning
    public int getTargetPosition() {
        return targetPosTicks;
    }

    public int getCurrentPosition() {
        return spindexer.getCurrentPosition();
    }

    public double getCurrentError() {
        return targetPosTicks - spindexer.getCurrentPosition();
    }

    public double getIntegralSum() {
        return integralSum;
    }
}