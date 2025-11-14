// Not done yet

package org.firstinspires.ftc.teamcode;

import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import android.app.Activity;
import android.graphics.Color;
import android.view.View;
import java.util.ArrayList;
import java.util.List;
import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.SwitchableLight;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


public class ColorSensor {

    ArrayList<Float> hueHistory = new ArrayList<>();
    public List<Integer> colorScores = new ArrayList<>(); // Rolling list of numbers 1, 0, -1. Number is based off of sensed color (1 = green, -1 = purple, 0 = other)
    NormalizedColorSensor colorSensor;

    View relativeLayout;

    float gain = 2;

    // Once per loop, we will update this hsvValues array. The first element (0) will contain the
    // hue, the second element (1) will contain the saturation, and the third element (2) will
    // contain the value. See http://web.archive.org/web/20190311170843/https://infohost.nmt.edu/tcc/help/pubs/colortheory/web/hsv.html
    // for an explanation of HSV color.
    final float[] hsvValues = new float[3];

    Telemetry telemetry;
    private String latestColor = "OTHER"; // the latest checked ball color

    private final int SAMPLE_SIZE = 50;

    TelemetryManager telemetryM;
    private final float CONFIDENCE_THRESHOLD = 0.8f; // when confidence is above 80% its good
    public void Init(HardwareMap hardwareMap, Telemetry tele) {
        telemetry = tele;
        int relativeLayoutId = hardwareMap.appContext.getResources().getIdentifier("RelativeLayout", "id", hardwareMap.appContext.getPackageName());
        relativeLayout = ((Activity) hardwareMap.appContext).findViewById(relativeLayoutId);

        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "sensor_color");

        if (colorSensor instanceof SwitchableLight) {
            ((SwitchableLight)colorSensor).enableLight(true);
        }

        colorSensor.setGain(gain);
    }

    public String BallDetermineUpdate() {
        telemetryM.debug("Hello panels telemetry works!");
        telemetryM.update();
        float hue = GetHue();
        int ballScore = GetBallScore(hue);
        float confidence = UpdateConfidenceLevel(ballScore);
        int confidentResult = ConfidentEnough(confidence);

        if (confidentResult != 0) {
            return latestColor;
        }
        return "UNCERTAIN";
    }

    public void Reset() {
        colorScores.clear();
    }

    public int ConfidentEnough(float confidence) {
        // if confidence level is high enough
        if (confidence < -CONFIDENCE_THRESHOLD) {
            telemetry.addData("FULLY CONFIDENT: ", "PURPLE");
            latestColor = "PURPLE";
            return -1;
        } else if (confidence > CONFIDENCE_THRESHOLD) {
            telemetry.addData("FULLY CONFIDENT: ", "GREEN");
            latestColor = "GREEN";
            return 1;
        } else {
            telemetry.addData("NOT CONFIDENT YET: ", ":(");
            // not confident enough
            return 0;
        }
    }

    public float UpdateConfidenceLevel(int ballScore) {
        colorScores.add(ballScore);

        if (colorScores.size() > SAMPLE_SIZE) {
            colorScores.remove(0);
        }

        // Calculates confidence level of how likely it is to be certain ball
        if (colorScores.size() == SAMPLE_SIZE) {
            int total = colorScores.stream().mapToInt(Integer::intValue).sum(); // This fancy ai generated line just calculates the total sum of all numbers in the list
            float confidenceLevel = total / (float)SAMPLE_SIZE;
            telemetry.addData("Confidence Level", "%.3f", confidenceLevel);
            return confidenceLevel;
        }
        telemetry.addData("Confidence Level: Collecting data ...", colorScores.size());
        return 0; // no confidence if not all color data has been collected yet
    }

    public int GetBallScore(float hue) {
        // Green is 1, Purple is -1, Other is 0

        // Tune these color detector results
        // Hue values are from 0-360
        final Float COLOR_ACCURACY = 20f; // how much the hue can differ from the targeted value
        final Float GREEN_HUE = 160f;
        final Float PURPLE_HUE = 240f;

        int sensedColor = 0;
        if (GREEN_HUE - COLOR_ACCURACY <= hue && hue <= GREEN_HUE + COLOR_ACCURACY) {
            sensedColor = 1;
        } else if (PURPLE_HUE - COLOR_ACCURACY <= hue && hue <= PURPLE_HUE + COLOR_ACCURACY) {
            sensedColor = -1;
        }

        return sensedColor;
    }


    public String GetBallColor(int score) {
        // score should be 0, 1, or -1.
        // if not that is a big problem
        if (score == 1) {
            return "GREEN";
        } else if (score == -1) {
            return "PURPLE";
        } else if (score == 0) {
            return "OTHER";
        } else {
            // Hopefully should never happen
            telemetry.addData("ERROR:", "GetBallColor() WAS CALLED INCORRECTLY");
            return "WHAT THE BUST";
        }
    }

    public float GetHue() {
        // Show the gain value via telemetry
        telemetry.addData("Gain", gain);

        // Tell the sensor our desired gain value (normally you would do this during initialization,
        // not during the loop)
        colorSensor.setGain(gain);

        // Get the normalized colors from the sensor
        NormalizedRGBA colors = colorSensor.getNormalizedColors();

        /* Use telemetry to display feedback on the driver station. We show the red, green, and blue
         * normalized values from the sensor (in the range of 0 to 1), as well as the equivalent
         * HSV (hue, saturation and value) values. See http://web.archive.org/web/20190311170843/https://infohost.nmt.edu/tcc/help/pubs/colortheory/web/hsv.html
         * for an explanation of HSV color. */

        // Update the hsvValues array by passing it to Color.colorToHSV()
        Color.colorToHSV(colors.toColor(), hsvValues);

        telemetry.addLine()
                .addData("Red", "%.3f", colors.red)
                .addData("Green", "%.3f", colors.green)
                .addData("Blue", "%.3f", colors.blue);
        telemetry.addLine()
                .addData("Hue", "%.3f", hsvValues[0])
                .addData("Saturation", "%.3f", hsvValues[1])
                .addData("Value", "%.3f", hsvValues[2]);
        telemetry.addData("Alpha", "%.3f", colors.alpha);

        /* If this color sensor also has a distance sensor, display the measured distance.
         * Note that the reported distance is only useful at very close range, and is impacted by
         * ambient light and surface reflectivity. */
        if (colorSensor instanceof DistanceSensor) {
            telemetry.addData("Distance (cm)", "%.3f", ((DistanceSensor) colorSensor).getDistance(DistanceUnit.CM));
        }

        // Change the Robot Controller's background color to match the color detected by the color sensor.
        relativeLayout.post(new Runnable() {
            public void run() {
                relativeLayout.setBackgroundColor(Color.HSVToColor(hsvValues));
            }
        });

        return hsvValues[0];
    }


    public void Stop() {
        // I don't know if it's neccesary to do this but the original color sensor code does this so ya

        // On the way out, *guarantee* that the background is reasonable. It doesn't actually start off
        // as pure white, but it's too much work to dig out what actually was used, and this is good
        // enough to at least make the screen reasonable again.
        // Set the panel back to the default color
        relativeLayout.post(new Runnable() {
            public void run() {
                relativeLayout.setBackgroundColor(Color.WHITE);
            }
        });
    }
}
