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
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.subsystems.Indexer;
import org.firstinspires.ftc.teamcode.SimpleTurret;
import org.firstinspires.ftc.teamcode.subsystems.Spindexer;
import org.firstinspires.ftc.teamcode.ShooterRobotFunctions;

import java.util.function.Supplier;

@Configurable
@TeleOp(name = "SmallRedTele")
public class RedTeleSmall extends OpMode {
    private Follower follower;
    public static Pose startingPose = new Pose(96, 9, Math.toRadians(90)); // Default starting pose
    private boolean automatedDrive;
    private Supplier<PathChain> pathChain;
    private TelemetryManager telemetryM;
    private boolean slowMode = false;
    private double slowModeMultiplier = 0.5;
    // Motor declarations
    private DcMotor frontLeft, frontRight, backLeft, backRight, shooter, spinner;
    CRServo LeftServo, RightServo;
    boolean pressedTriangle, pressedSquare, pressedCross, shooterOn = false;
    private double currentShooterPower = 0.0;

    private final Spindexer spindexer = new Spindexer();
    private final SimpleTurret simpleTurret = new SimpleTurret();

    private final Indexer indexer = new Indexer();
    boolean spindexerDirection = true;
    private ShooterRobotFunctions shooterFunctions;
    public static final double TRIGGER_THRESHOLD = 0.1;

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);

        // Try to load pose from auto, fall back to default if not available
        Pose actualStartingPose = loadAutoPose();
        follower.setStartingPose(actualStartingPose);
        follower.update();
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        pathChain = () -> follower.pathBuilder() //Lazy Curve Generation
                .addPath(new Path(new BezierLine(follower::getPose, new Pose(38, 33))))
                .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading, Math.toRadians(45), 0.8))
                .build();

        spinner.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        spinner.setTargetPosition(0);
        spinner.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        spinner.setPower(0.3); // Holding power to maintain position
        spinner.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        spinner = hardwareMap.get(DcMotor.class, "motor2");
        LeftServo = hardwareMap.get(CRServo.class, "LeftServo");
        RightServo = hardwareMap.get(CRServo.class, "RightServo");

        // ==== Spindexer setup ====
        spindexer.freshInit(hardwareMap);
        //indexer
        //indexer.Init(hardwareMap, telemetry);
        //shooter
        shooterFunctions = new ShooterRobotFunctions();
        // The init method handles setting up hardware and the A & B controllers
        shooterFunctions.init(hardwareMap, follower);

        // Log which pose we're using
        telemetry.addData("Starting Pose", "Using %s pose: (%.1f, %.1f, %.1f°)",
                (actualStartingPose == startingPose) ? "DEFAULT" : "AUTO",
                actualStartingPose.getX(), actualStartingPose.getY(),
                Math.toDegrees(actualStartingPose.getHeading()));
    }

    /**
     * Loads the final pose from auto if available, otherwise returns the default starting pose
     */
    private Pose loadAutoPose() {
        try {
            android.content.SharedPreferences prefs = hardwareMap.appContext.getSharedPreferences("RobotPose", 0);
            float x = prefs.getFloat("finalX", Float.NaN);
            float y = prefs.getFloat("finalY", Float.NaN);
            float heading = prefs.getFloat("finalHeading", Float.NaN);

            // Check if we have valid auto pose data
            if (!Float.isNaN(x) && !Float.isNaN(y) && !Float.isNaN(heading)) {
                Pose autoPose = new Pose(x, y, heading);
                telemetry.addData("Auto Pose Loaded", "X: %.1f, Y: %.1f, Heading: %.1f°",
                        x, y, Math.toDegrees(heading));
                return autoPose;
            }
        } catch (Exception e) {
            telemetry.addData("Error", "Failed to load auto pose: " + e.getMessage());
        }

        // Fall back to default starting pose
        telemetry.addData("Auto Pose", "Not found, using default");
        return startingPose;
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

        if (!automatedDrive) {
            //Make the last parameter false for field-centric
            //In case the drivers want to use a "slowMode" you can scale the vectors

            //This is the normal version to use in the TeleOp
            if (!slowMode) follower.setTeleOpDrive(
                    -gamepad1.left_stick_y,
                    -gamepad1.left_stick_x,
                    -gamepad1.right_stick_x,
                    true // Robot Centric
            );

                //This is how it looks with slowMode on
            else follower.setTeleOpDrive(
                    -gamepad1.left_stick_y * slowModeMultiplier,
                    -gamepad1.left_stick_x * slowModeMultiplier,
                    -gamepad1.right_stick_x * slowModeMultiplier,
                    true // Robot Centric
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
        //shooter
        boolean shooterCommanded = gamepad1.left_trigger > TRIGGER_THRESHOLD;

        // *** Command ONLY Target A ***
        shooterFunctions.setShooterPowerTargetA(shooterCommanded);

        // Optional: Explicitly disable Target B for safety
        shooterFunctions.setShooterPowerTargetB(false);
        //intake
        double rt = gamepad1.right_trigger;
        if (rt > 0.05) {
            // change: always run intake at full speed when trigger is pressed at all
            Intake(-1.0); // preserve original negative direction used previously
        } else if (gamepad1.dpad_up) {
            // manual full-speed reverse/intake
            Intake(1);
        } else {
            Intake(0);  // Stop
        }

        telemetry.addData("Spinner Position", spinner.getCurrentPosition());
        telemetry.addData("Spinner Target", spinner.getTargetPosition());
        telemetry.addData("Spinner Power", spinner.getPower());
        telemetry.addData("Spinner Busy", spinner.isBusy());
        //indexer
        indexer.Update();
        //spindexer
        telemetry.addData("Is Spindexer busy? ", spindexer.spindexer.isBusy());
        if (indexer.currentState == Indexer.State.IDLE) { // can't spindex if indexer is shooting
            if (gamepad1.left_bumper) {
                spindexer.rotateClockwise();
            } else if (gamepad1.right_bumper) {
                spindexer.rotateCounterclockwise();
            }
        }

        telemetry.addData("Intake RT", "%.2f", rt);
        telemetry.addData("LeftServo Power", LeftServo.getPower());
        telemetry.addData("RightServo Power", RightServo.getPower());
        telemetry.update();





        //Slow Mode
//        if (gamepad1.rightBumperWasPressed()) {
//            slowMode = !slowMode;
//        }

        //Optional way to change slow mode strength
//        if (gamepad1.xWasPressed()) {
//            slowModeMultiplier += 0.25;
//        }
//
//        //Optional way to change slow mode strength
//        if (gamepad2.yWasPressed()) {
//            slowModeMultiplier -= 0.25;
//        }

        telemetryM.debug("position", follower.getPose());
        telemetryM.debug("velocity", follower.getVelocity());
        telemetryM.debug("automatedDrive", automatedDrive);
    }
    public void Intake(double power) {
        LeftServo.setPower(power);
        RightServo.setPower(-power);
    }
}