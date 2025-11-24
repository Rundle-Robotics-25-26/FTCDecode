//package org.firstinspires.ftc.teamcode.pedroPathing;
//import com.bylazar.configurables.annotations.Configurable;
//import com.bylazar.telemetry.PanelsTelemetry;
//import com.bylazar.telemetry.TelemetryManager;
//import com.pedropathing.follower.Follower;
//import com.pedropathing.geometry.BezierLine;
//import com.pedropathing.geometry.Pose;
//import com.pedropathing.paths.HeadingInterpolator;
//import com.pedropathing.paths.Path;
//import com.pedropathing.paths.PathChain;
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import org.firstinspires.ftc.teamcode.RobotFunctions;
//
//import java.util.function.Supplier;
//
//@Configurable
//@TeleOp
//public class TeleOPG extends OpMode {
//    private Follower follower;
//    public static Pose startingPose;
//    private boolean automatedDrive;
//    private Supplier<PathChain> pathChain;
//    private TelemetryManager telemetryM;
//    private boolean slowMode = false;
//    private double slowModeMultiplier = 0.5;
//
//    // ADD TURRET VARIABLES
//    private RobotFunctions robotFunctions;
//    private boolean turretEnabled = true;
//
//    @Override
//    public void init() {
//        // OPTION 1: Hardcode your starting position (48, 9) with 0 heading
//        startingPose = new Pose(48, 9, 90);
//        telemetry.addData("Starting Pose", "Hardcoded: (48, 9, 90°)");
//
//        follower = Constants.createFollower(hardwareMap);
//
//        // IMU RESET CODE - ADDED THIS SECTION
//        telemetry.addData("Status", "Resetting IMU heading to 90°...");
//        telemetry.update();
//        try {
//            Thread.sleep(1000); // Wait 1 second
//            follower.setStartingPose(startingPose); // Force heading to 0
//            Thread.sleep(500); // Wait half second for reset to apply
//        } catch (InterruptedException e) {
//            telemetry.addData("IMU Reset", "Interrupted");
//        }
//
//        follower.update();
//        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
//
//        // INITIALIZE TURRET SYSTEM
//        robotFunctions = new RobotFunctions();
//        robotFunctions.initTurret(hardwareMap, follower);
//
//        pathChain = () -> follower.pathBuilder()
//                .addPath(new Path(new BezierLine(follower::getPose, new Pose(45, 98))))
//                .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading, Math.toRadians(45), 0.8))
//                .build();
//
//        telemetry.addData("Status", "IMU reset complete - check heading");
//    }
//
//    @Override
//    public void start() {
//        follower.startTeleopDrive();
//    }
//
//    @Override
//    public void loop() {
//        follower.update();
//        telemetryM.update();
//
//        // UPDATE TURRET AUTO-AIM (if enabled)
//        if (turretEnabled) {
//            robotFunctions.updateTurret();
//        }
//
//        if (!automatedDrive) {
//            if (!slowMode) follower.setTeleOpDrive(
//                    -gamepad1.left_stick_y,
//                    -gamepad1.left_stick_x,
//                    -gamepad1.right_stick_x,
//                    false
//            );
//            else follower.setTeleOpDrive(
//                    -gamepad1.left_stick_y * slowModeMultiplier,
//                    -gamepad1.left_stick_x * slowModeMultiplier,
//                    -gamepad1.right_stick_x * slowModeMultiplier,
//                    false
//            );
//        }
//
//        //Automated PathFollowing
//        if (gamepad1.aWasPressed()) {
//            follower.followPath(pathChain.get());
//            automatedDrive = true;
//        }
//
//        //Stop automated following if the follower is done
//        if (automatedDrive && (gamepad1.bWasPressed() || !follower.isBusy())) {
//            follower.startTeleopDrive();
//            automatedDrive = false;
//        }
//
//        //Slow Mode
//        if (gamepad1.rightBumperWasPressed()) {
//            slowMode = !slowMode;
//        }
//
//        // TURRET CONTROLS
//        if (gamepad2.left_bumper) {
//            turretEnabled = !turretEnabled;
//        }
//
//        // Manual turret control (when auto-aim is off)
//        if (!turretEnabled) {
//            double manualTurretSpeed = -gamepad2.right_stick_x * 0.5;
//            if (Math.abs(manualTurretSpeed) > 0.1) {
//                int currentTicks = robotFunctions.getTurretTicks();
//                int newTicks = currentTicks + (int)(manualTurretSpeed * 10);
//                robotFunctions.setTurretManualPosition(newTicks);
//            }
//        }
//
//        // Reset turret to center (when auto-aim is off)
//        if (gamepad2.dpad_down && !turretEnabled) {
//            robotFunctions.setTurretManualPosition(0);
//        }
//
//        //Optional way to change slow mode strength
//        if (gamepad1.xWasPressed()) {
//            slowModeMultiplier += 0.25;
//        }
//
//        //Optional way to change slow mode strength
//        if (gamepad2.yWasPressed()) {
//            slowModeMultiplier -= 0.25;
//        }
//
//        telemetryM.debug("position", follower.getPose());
//        telemetryM.debug("velocity", follower.getVelocity());
//        telemetryM.debug("automatedDrive", automatedDrive);
//
//        // ADD TURRET TELEMETRY
//        telemetry.addData("Turret Auto-Aim", turretEnabled ? "ON" : "OFF");
//        telemetry.addData("Turret Ticks", robotFunctions.getTurretTicks());
//        telemetry.addData("Turret Angle", "%.1f°", robotFunctions.getTurretAngleDegrees());
//        telemetry.addData("Turret Limits", "Left: %d, Right: %d",
//                robotFunctions.getMaxLeftTicks(), robotFunctions.getMaxRightTicks());
//        telemetry.addData("Goal In Range", robotFunctions.isGoalInRange() ? "YES" : "NO");
//        telemetry.addData("Turret Debug", robotFunctions.getTurretDebugInfo());
//
//        // Add current pose to regular telemetry for easy viewing
//        telemetry.addData("Current Pose", "X: %.1f, Y: %.1f, Heading: %.1f°",
//                follower.getPose().getX(),
//                follower.getPose().getY(),
//                Math.toDegrees(follower.getPose().getHeading()));
//    }
//}