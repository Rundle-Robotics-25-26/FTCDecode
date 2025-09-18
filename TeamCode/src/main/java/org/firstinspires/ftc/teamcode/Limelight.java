package org.firstinspires.ftc.teamcode;

import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.bylazar.configurables.PanelsConfigurables;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.configurables.annotations.IgnoreConfigurable;
import com.bylazar.field.FieldManager;
import com.bylazar.field.PanelsField;
import com.bylazar.field.Style;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;


import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.List;
import java.util.Map;


public class Limelight {
    private Limelight3A limelight3A;
    private Telemetry telemetry;

    private String latestPattern = "None";

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
        if (result != null && result.isValid()) {
            telemetry.addData("Target X offset", result.getTx());
            telemetry.addData("Target Y offset", result.getTy());
            telemetry.addData("Target Area", result.getTa());
        } else {
            telemetry.addData("Limelight", "No Targets");
        }
        return result;
    }

    public List<LLResultTypes.FiducialResult> detectAprilTags(LLResult result) {
        Map<Integer, String> nameById = Map.of(
                20, "Blue Goal",
                21, "GPP",
                22, "PGP",
                23, "PPG",
                24, "Red Goal"
        );

        // fiducials are just april tags
        List<LLResultTypes.FiducialResult> aprilTags = result.getFiducialResults();
        for (LLResultTypes.FiducialResult aprilTag : aprilTags) {
            int id = aprilTag.getFiducialId();

            if (nameById.containsKey(id)) {
                telemetry.addData("April Tag ID: " + id, nameById.get(id));
            } else {
                telemetry.addData("April Tag ID: " + id, "UNKNOWN");
            }

            // If the limelight is GPP, PGP or PPG
            if (id >= 21 && id <= 23) {
                latestPattern = nameById.get(id);
            }
        }
        telemetry.addData("Latest pattern:", latestPattern);
        return aprilTags;
    }

    public double getTargetDistance(LLResult result){
        double ty = result.getTy();

        // all measurements in inches
        final double mountAngle = 0; // How much is limelight angled (degrees)
        final double height = 20.0; // Height from center of lens to the floor
        final double targetHeight = 60; // Height of target

        double angleDegrees = mountAngle + ty;
        double angleRadians = angleDegrees * (3.141592 / 180.0);

        double distance = (targetHeight - height) / Math.tan(angleRadians);
        return distance;
    }
}