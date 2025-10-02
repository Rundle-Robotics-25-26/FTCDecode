// Not done yet

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import android.app.Activity;
import android.graphics.Color;
import android.view.View;
import java.util.ArrayList;
import java.util.List;

import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.SwitchableLight;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


public class ColorSensor {

    ArrayList<Float> hueHistory = new ArrayList<>();
    NormalizedColorSensor colorSensor;

    View relativeLayout;

    float gain = 2;
    int i;

    float runningSum = 0f;

    // Once per loop, we will update this hsvValues array. The first element (0) will contain the
    // hue, the second element (1) will contain the saturation, and the third element (2) will
    // contain the value. See http://web.archive.org/web/20190311170843/https://infohost.nmt.edu/tcc/help/pubs/colortheory/web/hsv.html
    // for an explanation of HSV color.
    final float[] hsvValues = new float[3];

    Telemetry telemetry;

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

    public String Update() {
        // calculates the average hue out of 100 latest hue results
        float recentHue = GetHue();
        hueHistory.add(recentHue);
        runningSum += recentHue;
        i++;

        if (hueHistory.size() > 100) {
            float removedHue = hueHistory.remove(0);
            runningSum -= removedHue;
        }
        if (i >= 100) {
            i = 0;
        }

        float avg = runningSum / hueHistory.size();

        telemetry.addData("Rolling hue average:", "%.2f", avg);

        // Tune these color detector results
        // Hue values are from 0-360
        final Float COLOR_ACCURACY = 20f; // how much the hue can differ from the targeted value
        final Float GREEN_HUE = 160f;
        final Float PURPLE_HUE = 240f;

        String sensedColor = "NONE";
        if (GREEN_HUE - COLOR_ACCURACY <= avg && avg <= GREEN_HUE + COLOR_ACCURACY) {
            sensedColor = "GREEN BALL";
        } else if (PURPLE_HUE - COLOR_ACCURACY <= avg && avg <= PURPLE_HUE + COLOR_ACCURACY) {
            sensedColor = "PURPLE BALL";
        }

        telemetry.addData("Sensed ball color: ", sensedColor);
        telemetry.addData("Update", i);

        telemetry.update();
        return sensedColor;
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
