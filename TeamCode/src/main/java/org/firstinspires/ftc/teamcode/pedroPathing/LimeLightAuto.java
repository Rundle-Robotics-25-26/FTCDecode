package org.firstinspires.ftc.teamcode.pedroPathing;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.teamcode.Indexer;
import org.firstinspires.ftc.teamcode.Limelight;
import org.firstinspires.ftc.teamcode.Spindexer;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;


@Autonomous(name = "LimeLightAuto", group = "Autonomous")
@Configurable
public class LimeLightAuto extends OpMode {
    private TelemetryManager panelsTelemetry; // Panels Telemetry instance
    private Follower follower; // Pedro Pathing follower instance
    private Timer pathTimer, actionTimer, opmodeTimer;
    private Limelight limelight = new Limelight();
    private Indexer indexer = new Indexer();
    private Spindexer spindexer = new Spindexer();
    private DcMotor shooter, spinner;

    boolean spindexerDirection = false;

    boolean shooterOn = false;

    int shootAmount = 3;

    private int pathState; // Current autonomous path state (state machine)

    private final Pose startPose = new Pose(20.135006157635466, 122.7192118226601, Math.toRadians(324));
    private final Pose obeliskPose = new Pose(64.000, 79.000, Math.toRadians(90));
    private final Pose scorePose= new Pose(64.000, 79.000, Math.toRadians(131));
    private final Pose leavePose = new Pose(44.00, 71.000, Math.toRadians(180));
    private PathChain path1, path2, path3;

    public void buildPaths() {

        path1 = follower.pathBuilder()
                .addPath(new BezierLine(startPose, obeliskPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), obeliskPose.getHeading())
                .build();


        path2 = follower.pathBuilder()
                .addPath(new BezierLine(obeliskPose, scorePose))
                .setLinearHeadingInterpolation(obeliskPose.getHeading(), scorePose.getHeading())
                .build();

        path3 = follower.pathBuilder()
                .addPath(new BezierLine(obeliskPose, leavePose))
                .setLinearHeadingInterpolation(obeliskPose.getHeading(), leavePose.getHeading())
                .build();
    }
    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                ShooterSet(0.6);
                follower.followPath(path1,true);
                setPathState(1);
                break;
            case 1:
                if (limelight.latestPattern.equals("GPP"))
                    {
                        if(!follower.isBusy())
                        {
                            follower.followPath(path2,true);
                            setPathState(2);
                        }

                    } else if (limelight.latestPattern.equals("PGP"))
                    {
                        if(!follower.isBusy())
                        {
                            follower.followPath(path2,true);
                            setPathState(3);
                        }

                    } else if (limelight.latestPattern.equals("PPG"))
                    {
                        if(!follower.isBusy())
                        {
                            follower.followPath(path2,true);
                            setPathState(4);
                        }
                    }
                break;

            case 2:
                /**GPP**/
                if(!follower.isBusy()){

                    if (shootAmount <= 0) {
                        follower.followPath(path3);
                        setPathState(5);
                    } else {
                        if (indexer.shooterSpinMacroState == Indexer.State.IN_SEQUENCE)
                        {
                            // do nothing
                        } else if (indexer.shooterSpinMacroState == Indexer.State.WAITING)
                        {
                            shootAmount--;
                            indexer.shooterSpinMacroState = Indexer.State.IDLE;
                        } else
                        {
                            indexer.ShootAndSpin();
                        }
                    }
                }
                break;

            case 3:
                /**PGP**/
                spindexer.rotateCounterclockwise(false);
                if(!follower.isBusy())
                {
                    if (shootAmount <= 0)
                    {
                        follower.followPath(path3);
                        setPathState(5);
                    } else
                    {
                        if (indexer.shooterSpinMacroState == Indexer.State.IN_SEQUENCE)
                        {
                            // do nothing
                        } else if (indexer.shooterSpinMacroState == Indexer.State.WAITING)
                        {
                            shootAmount--;
                            indexer.shooterSpinMacroState = Indexer.State.IDLE;
                        } else
                        {
                            indexer.ShootAndSpin();
                        }
                    }
                }
                break;

            case 4:
                /**PPG**/
                spindexer.rotateClockwise(false);
                if(!follower.isBusy())
                {

                    if (shootAmount <= 0)
                    {
                        follower.followPath(path3);
                        setPathState(5);
                    } else
                    {
                        if (indexer.shooterSpinMacroState == Indexer.State.IN_SEQUENCE)
                        {
                            // do nothing
                        } else if (indexer.shooterSpinMacroState == Indexer.State.WAITING)
                        {
                            shootAmount--;
                            indexer.shooterSpinMacroState = Indexer.State.IDLE;
                        } else
                        {
                            indexer.ShootAndSpin();
                        }
                    }
                }
                break;

            case 5:

                if(!follower.isBusy())
                {
                    setPathState(-1);
                }
                break;

        }
    }
    public void setPathState(int pState)
    {
        pathState = pState;
        pathTimer.resetTimer();
    }

    public void ShooterSet(double power)
    {
            shooter.setPower(power);
    }

    @Override
    public void loop()
    {
        LLResult result = limelight.Update();
        limelight.detectAprilTags(result);

        indexer.Update(false);
        indexer.spindexerUpdate(spindexerDirection, spindexer);
        indexer.ShootAndSpinUpdate(spindexer);

        follower.update();
        autonomousPathUpdate();

        telemetry.addData("Spinner Position", spinner.getCurrentPosition());
        telemetry.addData("Spinner Target", spinner.getTargetPosition());
        telemetry.addData("Spinner Power", spinner.getPower());
        telemetry.addData("Spinner Busy", spinner.isBusy());

        // Pedro status
        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();

    }
    @Override
    public void init()
    {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        // ==== Limelight setup ====
        limelight.Init(hardwareMap, telemetry);

        // ==== Indexer setup ====
        indexer.Init(hardwareMap, telemetry);

        // ==== Spinner Setup - Constant Hold at Position 0 ====
        spinner = hardwareMap.get(DcMotor.class, "motor2");
        spinner.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        spinner.setTargetPosition(0);
        spinner.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        spinner.setPower(0.3); // Holding power to maintain position
        spinner.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // ==== Spindexer setup ====
        spindexer.dataInit(hardwareMap);

        // ==== Shooter setup ====
        shooter = hardwareMap.get(DcMotor.class, "shooter");
        // FIX: Use RUN_WITHOUT_ENCODER for consistent power output
        shooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(startPose);
    }

    @Override
    public void init_loop() {}

    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
        limelight.Start();
    }

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



