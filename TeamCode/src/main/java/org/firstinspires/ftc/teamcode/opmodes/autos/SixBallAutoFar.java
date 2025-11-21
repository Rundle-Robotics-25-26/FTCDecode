package org.firstinspires.ftc.teamcode.opmodes.autos;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import  com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;


@Autonomous(name = "Six Ball Auto Far", group = "Autos")
public class SixBallAutoFar extends OpMode {
    /*
    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;

    private int pathState;

    private final Pose startPose = new Pose(56, 8, Math.toRadians(90));
    private final Pose shootPose = new Pose(56.000, 8.000, Math.toRadians(137));
    public static PathChain shoot1 = builder
            .addPath(new BezierLine(new Pose(56.000, 8.000), new Pose(68.838, 84.405)))
            .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(137))
            .build();

    public static PathChain toPickup = builder
            .addPath(new BezierLine(new Pose(68.838, 84.405), new Pose(41.351, 35.270)))
            .setLinearHeadingInterpolation(Math.toRadians(137), Math.toRadians(-180))
            .build();

    public static PathChain pickup1 = builder
            .addPath(new BezierLine(new Pose(41.351, 35.270), new Pose(34.541, 35.270)))
            .setTangentHeadingInterpolation()
            .build();

    public static PathChain pickup2 = builder
            .addPath(new BezierLine(new Pose(34.541, 35.270), new Pose(30.000, 35.270)))
            .setTangentHeadingInterpolation()
            .build();

    public static PathChain pickup3 = builder
            .addPath(new BezierLine(new Pose(30.000, 35.270), new Pose(19.000, 35.270)))
            .setTangentHeadingInterpolation()
            .build();

    public static PathChain lastShoot = builder
            .addPath(
                    new BezierCurve(
                            new Pose(19.000, 35.270),
                            new Pose(91.459, 68.108),
                            new Pose(70.054, 81.973)
                    )
            )
            .setLinearHeadingInterpolation(Math.toRadians(-180), Math.toRadians(137))
            .build();

    private PathChain shoot1, toPickup, pickup1, pickup2, pickup3, lastShoot;
    public void buildPaths() {
        shoot1 = follower.pathBuilder()
                .addPath(new BezierLine(startPose, shootPose))
                .setLinearHeadingInterpolation()
    }
    */
    @Override
    public void init() {

    }

    @Override
    public void loop() {

    }
}

