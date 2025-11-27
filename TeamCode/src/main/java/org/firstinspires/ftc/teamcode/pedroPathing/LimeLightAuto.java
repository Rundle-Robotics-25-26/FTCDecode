package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.nio.file.Paths;
import java.util.List;
import java.util.Map;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
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
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name = "LimeLightAuto", group = "Autonomous")
@Configurable
public class LimeLightAuto extends OpMode {

    private Limelight3A limelight3A;
    private Telemetry telemetry;

    public void Init(HardwareMap hardwareMap, Telemetry tele) {
        telemetry = tele;
        limelight3A = hardwareMap.get(Limelight3A.class, "limelight");
        limelight3A.setPollRateHz(100);
        limelight3A.pipelineSwitch(0);
    }

    public void Start() {
        limelight3A.start();
    }
    public LLResult Update() {
        LLResult result = limelight3A.getLatestResult();
        return result;
    }
    private TelemetryManager panelsTelemetry; // Panels Telemetry instance
    private Follower follower; // Pedro Pathing follower instance
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState; // Current autonomous path state (state machine)

    private final Pose startPose = new Pose(64.000, 79.000, Math.toRadians(45));
    private final Pose pose2= new Pose(64.000, 79.000, Math.toRadians(135));
    private final Pose pose3 = new Pose(54.362, 37.773, Math.toRadians(180));
    private final Pose pose4 = new Pose(44.785, 37.773, Math.toRadians(180));
    private final Pose pose5= new Pose(35.918, 37.773, Math.toRadians(180));
    private final Pose endPose = new Pose(64.115, 79.094, Math.toRadians(135));

    private Path scorePreload;
    private PathChain path2, path3, path4,lastPath;

    public void buildPaths() {
        scorePreload = new Path(new BezierLine(startPose, pose2));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), pose2.getHeading());

        path2 = follower.pathBuilder()
                .addPath(new BezierLine(pose2, pose3))
                .setLinearHeadingInterpolation(pose2.getHeading(), pose3.getHeading())
                .build();

        path3 = follower.pathBuilder()
                .addPath(new BezierLine(pose3, pose4))
                .setTangentHeadingInterpolation()
                .build();

        path4 = follower.pathBuilder()
                .addPath(new BezierLine(pose4, pose5))
                .setTangentHeadingInterpolation()
                .build();

        lastPath = follower.pathBuilder()
                .addPath(new BezierLine(pose5, endPose))
                .setLinearHeadingInterpolation(pose5.getHeading(), endPose.getHeading())
                .build();
    }
    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(scorePreload);
                setPathState(1);
                break;




//            case 1:
//                if(!follower.isBusy()){
//                    follower.followPath(path2,true);
//                    setPathState(2);
//                }
//                break;
//            case 2:
//
//                if(!follower.isBusy()){
//                    follower.followPath(path3,true);
//                    setPathState(3);
//                }
//                break;
//            case 3:
//
//                if(!follower.isBusy()){
//                    follower.followPath(path4,true);
//                    setPathState(4);
//                }
//                break;
//            case 4:
//
//                if(!follower.isBusy()){
//                    follower.followPath(lastPath,true);
//                    setPathState(5);
//                }
//                break;
//            case 5:
//
//                if(!follower.isBusy()){
//                    setPathState(-1);
//                }
//                break;
        }
    }
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    @Override
    public void loop() {

        follower.update();
        autonomousPathUpdate();

        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();
    }
    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

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
    }
}


