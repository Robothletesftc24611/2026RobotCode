package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
@Disabled
@TeleOp(name = "HSV Color Test", group = "Testing")
public class colortest extends OpMode {

    // ---------------------------
    // HSV CLASSIFIER MODULE
    // ---------------------------
    public static class HSV {
        public float h, s, v;
        public HSV(float h, float s, float v) {
            this.h = h;
            this.s = s;
            this.v = v;
        }
    }

    // Convert 0–255 RGB → HSV
    public static HSV rgbToHSV(int r, int g, int b) {
        float rf = r / 255f;
        float gf = g / 255f;
        float bf = b / 255f;

        float max = Math.max(rf, Math.max(gf, bf));
        float min = Math.min(rf, Math.min(gf, bf));
        float delta = max - min;

        float h;
        if (delta == 0) {
            h = 0;
        } else if (max == rf) {
            h = 60 * (((gf - bf) / delta) % 6);
        } else if (max == gf) {
            h = 60 * (((bf - rf) / delta) + 2);
        } else {
            h = 60 * (((rf - gf) / delta) + 4);
        }
        if (h < 0) h += 360;

        float s = (max == 0) ? 0 : (delta / max);
        float v = max;

        return new HSV(h, s, v);
    }

    // Thresholds (tune these)
    private static final float PURPLE_HUE_MIN = 190;
    private static final float PURPLE_HUE_MAX = 250;

    private static final float GREEN_HUE_MIN = 130;
    private static final float GREEN_HUE_MAX = 180;

    private static final float MIN_SAT = 0.25f;   // reject gray/white
    private static final float MIN_VAL = 0.15f;   // reject shadows

    public enum BallColor {
        PURPLE,
        GREEN,
        UNKNOWN
    }

    public static BallColor classify(int r, int g, int b) {
        HSV hsv = rgbToHSV(r, g, b);

        // sanity checks
        if (hsv.s < MIN_SAT || hsv.v < MIN_VAL) {
            return BallColor.UNKNOWN;
        }

        float h = hsv.h;

        if (h >= PURPLE_HUE_MIN && h <= PURPLE_HUE_MAX) {
            return BallColor.PURPLE;
        }

        if (h >= GREEN_HUE_MIN && h <= GREEN_HUE_MAX) {
            return BallColor.GREEN;
        }

        return BallColor.UNKNOWN;
    }

    // ---------------------------
    // HARDWARE
    // ---------------------------
    private ColorSensor sensor;

    // ---------------------------
    // INIT
    // ---------------------------
    @Override
    public void init() {
        sensor = hardwareMap.get(ColorSensor.class, "color");

        telemetry.addLine("HSV Color Test Ready");
        telemetry.addLine("Move objects in front of sensor");
    }

    // ---------------------------
    // LOOP (runs ~50 times/sec)
    // ---------------------------
    @Override
    public void loop() {

        int r = sensor.red();
        int g = sensor.green();
        int b = sensor.blue();

        HSV hsv = rgbToHSV(r, g, b);
        BallColor detected = classify(r, g, b);

        telemetry.addData("Raw R", r);
        telemetry.addData("Raw G", g);
        telemetry.addData("Raw B", b);

        telemetry.addData("Hue", hsv.h);
        telemetry.addData("Sat", hsv.s);
        telemetry.addData("Val", hsv.v);

        telemetry.addData("Detected Color", detected);
    }
}
