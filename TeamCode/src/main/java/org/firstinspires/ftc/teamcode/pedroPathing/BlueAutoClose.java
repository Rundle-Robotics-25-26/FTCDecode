
package org.firstinspires.ftc.teamcode.pedroPathing; // make sure this aligns with class location

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import  com.qualcomm.robotcore.eventloop.opmode.OpMode;
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
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Indexer;
import org.firstinspires.ftc.teamcode.SimpleTurret;
import org.firstinspires.ftc.teamcode.Spindexer;
import org.firstinspires.ftc.teamcode.ShooterRobotFunctions;

@Autonomous(name = "CloseBlueAuto", group = "Auto")
public class BlueAutoClose extends OpMode {

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;

    private int pathState;

    private final Pose startPose = new Pose(22, 122, Math.toRadians(143)); // Start Pose of our robot.
    private final Pose scorePose1 = new Pose(40, 106, Math.toRadians(135)); // Scoring Pose of our robot. It is facing the goal at a 135 degree angle.

    private final Pose intakePose = new Pose(28, 128, Math.toRadians(180));

    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;
    private DcMotor shooter;
    private DcMotor spinner;
    private DcMotor shooterMotor;
    CRServo LeftServo, RightServo;
    boolean pressedTriangle, pressedSquare, pressedCross, shooterOn = false;
    private double currentShooterPower = 0.0;

    private final Spindexer spindexer = new Spindexer();
    private final SimpleTurret simpleTurret = new SimpleTurret();

    private final Indexer indexer = new Indexer();
    boolean spindexerDirection = true;
    private ShooterRobotFunctions shooterFunctions;
    public static final double TRIGGER_THRESHOLD = 0.1;



    private PathChain path1, path2;

    public void buildPaths() {
        /* This is our scorePreload path. We are using a BezierLine, which is a straight line. */
    /* Here is an example for Constant Interpolation
    scorePreload.setConstantInterpolation(startPose.getHeading()); */

        /* This is our grabPickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        path1 = follower.pathBuilder()
                .addPath(new BezierLine(startPose, scorePose1))
                .setLinearHeadingInterpolation(startPose.getHeading(), scorePose1.getHeading())
                .build();

        /* This is our scorePickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
//        path2 = follower.pathBuilder()
//                .addPath(new BezierLine(scorePose, otherPose))
//                .setLinearHeadingInterpolation(scorePose.getHeading(), otherPose.getHeading())
//                .build();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                shooterMotor.setPower(-0.5);
                follower.followPath(path1,true);
                setPathState(1);
                break;
            case 1:

            /* You could check for
            - Follower State: "if(!follower.isBusy()) {}"
            - Time: "if(pathTimer.getElapsedTimeSeconds() > 1) {}"
            - Robot Position: "if(follower.getPose().getX() > 36) {}"
            */

                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {
                    /* Score Preload */
                    indexer.Update(true);
                    if(pathTimer.getElapsedTimeSeconds() > 1.5) {
                        indexer.spindex(true, spindexer); // Stop indexer after 1.5 seconds
                    }
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
//                    follower.followPath(path2,true);
                    setPathState(2);
                }
                break;
            case 2:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {
                    indexer.Update(true);
                    if(pathTimer.getElapsedTimeSeconds() > 1.5) {
                        indexer.spindex(true, spindexer); // Stop indexer after 1.5 seconds
                    }
                    /* Set the state to a Case we won't use or define, so it just stops running an new paths */
                    setPathState(3);
                }
                break;
            case 3:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {
                    indexer.Update(true);
                    if(pathTimer.getElapsedTimeSeconds() > 1.5) {
                        indexer.spindex(true, spindexer); // Stop indexer after 1.5 seconds
                    }
                    /* Set the state to a Case we won't use or define, so it just stops running an new paths */
                    setPathState(4);
                }
                break;
            case 4:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {
                    /* Set the state to a Case we won't use or define, so it just stops running an new paths */
                    setPathState(-1);
                }
                break;
        }
    }

    /** These change the states of the paths and actions. It will also reset the timers of the individual switches **/
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    /** This is the main loop of the OpMode, it will run repeatedly after clicking "Play". **/
    @Override
    public void loop() {

        // These loop the movements of the robot, these must be called continuously in order to work
        follower.update();
        autonomousPathUpdate();

        // Feedback to Driver Hub for debugging
        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();
    }

    /** This method is called once at the init of the OpMode. **/
    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();


        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(startPose);

//        spinner.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        spinner.setTargetPosition(0);
//        spinner.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        spinner.setPower(0.3); // Holding power to maintain position
//        spinner.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        spinner = hardwareMap.get(DcMotor.class, "motor2");
        LeftServo = hardwareMap.get(CRServo.class, "LeftServo");
        RightServo = hardwareMap.get(CRServo.class, "RightServo");

        // ==== Spindexer setup ====
        spindexer.freshInit(hardwareMap);
        //indexer
        indexer.Init(hardwareMap, telemetry);
        //shooter
        shooterFunctions = new ShooterRobotFunctions();
        // The init method handles setting up hardware and the A & B controllers
        shooterFunctions.init(hardwareMap, follower);

        shooterMotor = hardwareMap.get(DcMotorEx.class, "shooter");
        shooterMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooterMotor.setPower(0.0);


    }

    /** This method is called continuously after Init while waiting for "play". **/
    @Override
    public void init_loop() {}

    /** This method is called once at the start of the OpMode.
     * It runs all the setup actions, including building paths and starting the path system **/
    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
    }

    /** We do not use this because everything should automatically disable **/
    @Override
    public void stop() {
        // Capture final pose when auto ends
        Pose finalPose = follower.getPose();

        // Store it for tele-op
        hardwareMap.appContext.getSharedPreferences("RobotPose", 0)
                .edit()
                .putFloat("finalX", (float)finalPose.getX())
                .putFloat("finalY", (float)finalPose.getY())
                .putFloat("finalHeading", (float)finalPose.getHeading())
                .apply();

        telemetry.addData("Auto End", "Saved: %.1f, %.1f, %.1f°",
                finalPose.getX(), finalPose.getY(), Math.toDegrees(finalPose.getHeading()));
    }
}