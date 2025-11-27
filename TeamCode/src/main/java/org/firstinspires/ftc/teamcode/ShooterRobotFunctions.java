package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Manages the core hardware control for the Shooter subsystem only.
 * It holds instances of different Shooter Controllers to support multiple target poses.
 */
public class ShooterRobotFunctions {

    // --- Controller Instances ---
    // You can choose which target to use (e.g., targetAController or targetBController)
    private ShooterController targetAController;
    private ShooterController targetBController;
    private Follower follower;

    //--------------------------- INITIALIZATION ---------------------------------------------------

    /**
     * Initializes all shooter controllers and systems.
     */
    public void init(HardwareMap hardwareMap, Follower follower) {
        this.follower = follower;

        // Initialize the two separate shooter controllers, passing the follower to each.
        // We'll define the different goal poses inside the separate controller files.
        this.targetAController = new ShooterControllerTargetA(follower);
        this.targetBController = new ShooterControllerTargetB(follower);

        // Initialize the hardware for each controller (they will likely share the same motor)
        targetAController.initShooter(hardwareMap);

        // NOTE: If you need to switch targets, you must decide which controller's
        // power/distance methods to call in the OpMode.
    }

    //--------------------------- CONTROL METHOD (Primary Target A) --------------------------------

    /**
     * Updates the shooter motor power based on the trigger state for Target A.
     * Use this method in your OpMode loop to fire at Target A.
     * @param triggerHeld True if the driver is commanding the shooter to fire.
     */
    public void setShooterPowerTargetA(boolean triggerHeld) {
        targetAController.setShooterPower(triggerHeld);
    }

    //--------------------------- CONTROL METHOD (Secondary Target B) ------------------------------

    /**
     * Updates the shooter motor power based on the trigger state for Target B.
     * Use this method in your OpMode loop to fire at Target B.
     * @param triggerHeld True if the driver is commanding the shooter to fire.
     */
    public void setShooterPowerTargetB(boolean triggerHeld) {
        targetBController.setShooterPower(triggerHeld);
    }

    //--------------------------- ACCESSOR METHODS (Choose which target to report) -----------------

    // You would choose which target's telemetry to display, or display both.

    public double getDistanceToTargetA() {
        return targetAController.getDistanceToGoal();
    }

    public double getCommandedPowerTargetA() {
        return targetAController.getCommandedPower();
    }

    public double getDistanceToTargetB() {
        return targetBController.getDistanceToGoal();
    }
}