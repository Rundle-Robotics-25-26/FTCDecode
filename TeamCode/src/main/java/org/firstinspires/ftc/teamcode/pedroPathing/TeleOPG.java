package org.firstinspires.ftc.teamcode.pedroPathing;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.RobotFunctions; // ADD THIS IMPORT

import java.util.function.Supplier;

@Configurable
@TeleOp
public class TeleOPG extends OpMode {
    private Follower follower;
    public static Pose startingPose; //See ExampleAuto to understand how to use this
    private boolean automatedDrive;
    private Supplier<PathChain> pathChain;
    private TelemetryManager telemetryM;
    private boolean slowMode = false;
    private double slowModeMultiplier = 0.5;

    // ADD TURRET VARIABLES
    private RobotFunctions robotFunctions;
    private boolean turretEnabled = true; // Toggle for turret auto-aim

    @Override
    public void init() {
        // OPTION 1: Hardcode your starting position (48, 9) with 0 heading
        startingPose = new Pose(48, 9, 0);
        telemetry.addData("Starting Pose", "Hardcoded: (48, 9, 0°)");

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startingPose);
        follower.update();
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        // INITIALIZE TURRET SYSTEM
        robotFunctions = new RobotFunctions();
        robotFunctions.initTurret(hardwareMap, follower);

        pathChain = () -> follower.pathBuilder() //Lazy Curve Generation
                .addPath(new Path(new BezierLine(follower::getPose, new Pose(45, 98))))
                .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading, Math.toRadians(45), 0.8))
                .build();
    }

    @Override
    public void start() {
        //The parameter controls whether the Follower should use break mode on the motors (using it is recommended).
        //In order to use float mode, add .useBrakeModeInTeleOp(true); to your Drivetrain Constants in Constant.java (for Mecanum)
        //If you don't pass anything in, it uses the default (false)
        follower.startTeleopDrive();
    }

    @Override
    public void loop() {
        //Call this once per loop
        follower.update();
        telemetryM.update();

        // UPDATE TURRET AUTO-AIM (if enabled) - ADD THIS
        if (turretEnabled) {
            robotFunctions.updateTurret();
        }

        /*
        // SHOOTER CONTROL - right trigger on gamepad1 - COMMENTED OUT
        boolean shootTrigger = gamepad1.right_trigger > 0.5;
        robotFunctions.updateShooter(shootTrigger);
        */

        if (!automatedDrive) {
            //Make the last parameter false for field-centric
            //In case the drivers want to use a "slowMode" you can scale the vectors

            //This is the normal version to use in the TeleOp
            if (!slowMode) follower.setTeleOpDrive(
                    -gamepad1.left_stick_y,
                    -gamepad1.left_stick_x,
                    -gamepad1.right_stick_x,
                    false // Robot Centric
            );

                //This is how it looks with slowMode on
            else follower.setTeleOpDrive(
                    -gamepad1.left_stick_y * slowModeMultiplier,
                    -gamepad1.left_stick_x * slowModeMultiplier,
                    -gamepad1.right_stick_x * slowModeMultiplier,
                    false // Robot Centric
            );
        }

        //Automated PathFollowing
        if (gamepad1.aWasPressed()) {
            follower.followPath(pathChain.get());
            automatedDrive = true;
        }

        //Stop automated following if the follower is done
        if (automatedDrive && (gamepad1.bWasPressed() || !follower.isBusy())) {
            follower.startTeleopDrive();
            automatedDrive = false;
        }

        //Slow Mode
        if (gamepad1.rightBumperWasPressed()) {
            slowMode = !slowMode;
        }

        // TURRET CONTROLS - ADDED THIS SECTION
        // Toggle auto-aim on/off - Gamepad2 Left Bumper
        if (gamepad2.left_bumper) {
            turretEnabled = !turretEnabled;
        }

        // Manual turret control (when auto-aim is off) - Gamepad2 Right Stick
        if (!turretEnabled) {
            double manualTurretSpeed = -gamepad2.right_stick_x * 0.5; // Adjust sensitivity
            if (Math.abs(manualTurretSpeed) > 0.1) {
                // Get current position and adjust manually
                int currentTicks = robotFunctions.getTurretTicks();
                int newTicks = currentTicks + (int)(manualTurretSpeed * 10); // Adjust step size
                robotFunctions.setTurretManualPosition(newTicks);
            }
        }

        // Reset turret to center (when auto-aim is off) - Gamepad2 D-pad Down
        if (gamepad2.dpad_down && !turretEnabled) {
            robotFunctions.setTurretManualPosition(0);
        }

        //Optional way to change slow mode strength
        if (gamepad1.xWasPressed()) {
            slowModeMultiplier += 0.25;
        }

        //Optional way to change slow mode strength
        if (gamepad2.yWasPressed()) {
            slowModeMultiplier -= 0.25;
        }

        telemetryM.debug("position", follower.getPose());
        telemetryM.debug("velocity", follower.getVelocity());
        telemetryM.debug("automatedDrive", automatedDrive);

        // ADD TURRET TELEMETRY - ADD THIS SECTION
        telemetry.addData("Turret Auto-Aim", turretEnabled ? "ON" : "OFF");
        telemetry.addData("Turret Ticks", robotFunctions.getTurretTicks());
        telemetry.addData("Turret Angle", "%.1f°", robotFunctions.getTurretAngleDegrees());
        telemetry.addData("Turret Limits", "Left: %d, Right: %d",
                robotFunctions.getMaxLeftTicks(), robotFunctions.getMaxRightTicks());
        telemetry.addData("Goal In Range", robotFunctions.isGoalInRange() ? "YES" : "NO");
        telemetry.addData("Turret Debug", robotFunctions.getTurretDebugInfo());

        /*
        // SHOOTER TELEMETRY - COMMENTED OUT
        telemetry.addData("Shooter", "Power: %.2f, Distance: %.1f\"",
            robotFunctions.getCurrentShooterPower(),
            robotFunctions.getDistanceToGoal());
        telemetry.addData("Shooter Trigger", shootTrigger ? "HELD" : "OFF");
        */

        // Add current pose to regular telemetry for easy viewing
        telemetry.addData("Current Pose", "X: %.1f, Y: %.1f, Heading: %.1f°",
                follower.getPose().getX(),
                follower.getPose().getY(),
                Math.toDegrees(follower.getPose().getHeading()));
    }
}