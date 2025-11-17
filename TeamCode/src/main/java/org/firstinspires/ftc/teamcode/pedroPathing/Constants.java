package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {
    public static MecanumConstants driveConstants = new MecanumConstants()
        .maxPower(1)
        .rightFrontMotorName("rf")
        .rightRearMotorName("rr")
        .leftRearMotorName("lr")
        .leftFrontMotorName("lf")
        .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
        .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
        .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
        .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)
        .xVelocity(62.06151238388903)
        .yVelocity(42.134110578401824);
    public static FollowerConstants followerConstants = new FollowerConstants()
        .mass(10.4)
        .forwardZeroPowerAcceleration(-34.18707208137649)
        .lateralZeroPowerAcceleration(-73.21419897528656)
        .useSecondaryTranslationalPIDF(false)
        .useSecondaryHeadingPIDF(false)
        .useSecondaryDrivePIDF(false)
        .translationalPIDFCoefficients(new PIDFCoefficients(0.1, 0, 0.01, 0.025))
        .headingPIDFCoefficients(new PIDFCoefficients(1, 0, 0.02, 0.03))
        .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.05,0.0,0.0001,0.6,0.02))
        .centripetalScaling(0.001);


    public static PinpointConstants localizerConstants = new PinpointConstants()
        .forwardPodY(-2.5)
        .strafePodX(0.25)
        .distanceUnit(DistanceUnit.INCH)
        .hardwareMapName("pinpoint")
        .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
        .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED)
        .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD);


    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 1);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
            .pinpointLocalizer(localizerConstants)
            .pathConstraints(pathConstraints)
            .mecanumDrivetrain(driveConstants)
            .build();
    }

}
