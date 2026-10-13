package org.firstinspires.ftc.teamcode.vision;

import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/*
 * HuskyLens red / blue Nectar detector with telemetry.
 *
 * SETUP (do this once, on the HuskyLens itself):
 *   1. Scroll the dial to "Color Recognition".
 *   2. Long-press the dial -> turn ON "Learn Multiple" -> Save & Return.
 *   3. Point the "+" at a RED nectar, hold the learn button until the box locks on.  -> ID 1
 *   4. Press the learn button again when asked, then learn a BLUE nectar.           -> ID 2
 *
 * ROBOT CONFIG:
 *   Plug the HuskyLens into an I2C port on the Control Hub and add it in the
 *   robot configuration as device type "HuskyLens" with the name "huskylens".
 */
@TeleOp(name = "HuskyLens Nectar", group = "Sensor")
public class HuskyLensNectar extends LinearOpMode {

    // These must match the order you trained the colors in.
    private static final int RED_ID  = 1;
    private static final int BLUE_ID = 2;

    // HuskyLens image is 320 x 240, so the center column is x = 160.
    private static final int CENTER_X = 160;

    // Ignore tiny blobs (noise). Raise this if you get false detections.
    private static final int MIN_AREA = 400;

    private HuskyLens huskyLens;

    @Override
    public void runOpMode() {
        huskyLens = hardwareMap.get(HuskyLens.class, "huskylens");

        // knock() checks that the camera is actually talking to the hub.
        if (!huskyLens.knock()) {
            telemetry.addData("HuskyLens", "NOT responding - check wiring and config name");
        } else {
            telemetry.addData("HuskyLens", "Connected. Press START");
        }

        huskyLens.selectAlgorithm(HuskyLens.Algorithm.COLOR_RECOGNITION);
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            HuskyLens.Block[] blocks = huskyLens.blocks();

            HuskyLens.Block bestRed  = null;
            HuskyLens.Block bestBlue = null;
            int redCount  = 0;
            int blueCount = 0;

            // Sort every block the camera sees into red / blue and keep the biggest of each.
            for (HuskyLens.Block b : blocks) {
                int area = b.width * b.height;
                if (area < MIN_AREA) continue;

                if (b.id == RED_ID) {
                    redCount++;
                    if (bestRed == null || area > bestRed.width * bestRed.height) bestRed = b;
                } else if (b.id == BLUE_ID) {
                    blueCount++;
                    if (bestBlue == null || area > bestBlue.width * bestBlue.height) bestBlue = b;
                }
            }

            telemetry.addData("Blocks seen", blocks.length);
            telemetry.addLine();
            showNectar("RED",  redCount,  bestRed);
            telemetry.addLine();
            showNectar("BLUE", blueCount, bestBlue);
            telemetry.update();

            sleep(50); // don't hammer the I2C bus
        }
    }

    /** Prints count, position, size and left/right offset for the biggest nectar of one color. */
    private void showNectar(String label, int count, HuskyLens.Block b) {
        telemetry.addData(label + " count", count);
        if (b == null) {
            telemetry.addData(label, "not detected");
            return;
        }
        int offset = b.x - CENTER_X;   // negative = left of center, positive = right
        String side = Math.abs(offset) < 20 ? "CENTER" : (offset < 0 ? "LEFT" : "RIGHT");

        telemetry.addData(label + " center (x, y)", "%d, %d", b.x, b.y);
        telemetry.addData(label + " size (w x h)", "%d x %d", b.width, b.height);
        telemetry.addData(label + " area", b.width * b.height);
        telemetry.addData(label + " offset", "%d px (%s)", offset, side);
    }
}
