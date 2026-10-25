package org.firstinspires.ftc.teamcode.vision;

import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.List;

/** Shows how many pollen are in each cluster the HuskyLens sees. */
@TeleOp(name = "HuskyLens Pollen Test", group = "Vision")
public class HuskyLensPollenTest extends LinearOpMode {

    @Override
    public void runOpMode() {
        HuskyLens huskyLens = hardwareMap.get(HuskyLens.class, "huskylens");
        if (!huskyLens.knock()) {
            telemetry.addData(">>", "Problem talking to " + huskyLens.getDeviceName());
        } else {
            telemetry.addData(">>", "HuskyLens connected. Press START.");
        }
        telemetry.update();

        HuskyLensPollenClusters pollen = new HuskyLensPollenClusters(huskyLens);
        telemetry.addData("Calibration", pollen.calibrationLoaded ? "loaded" : "NOT calibrated yet - use A / B");
        telemetry.update();
        waitForStart();

        boolean connected = huskyLens.knock();
        ElapsedTime knockTimer = new ElapsedTime();
        boolean prevA = false, prevB = false, prevUp = false, prevDown = false, prevY = false;
        String calMsg = "";

        while (opModeIsActive()) {
            if (knockTimer.seconds() > 1) {
                boolean now = huskyLens.knock();
                if (now && !connected) huskyLens.selectAlgorithm(HuskyLens.Algorithm.COLOR_RECOGNITION);
                connected = now;
                knockTimer.reset();
            }
            List<HuskyLensPollenClusters.Cluster> clusters = pollen.update();

            // Calibration on gamepad 1: A = one ball far, B = one ball near,
            // dpad up/down = count reads too low/high, Y = reset
            if (gamepad1.a && !prevA) calMsg = pollen.calibrateFar()  ? "far saved"  : "A: no pollen seen";
            if (gamepad1.b && !prevB) calMsg = pollen.calibrateNear() ? "near saved" : "B: no pollen seen";
            if (gamepad1.dpad_up && !prevUp)     pollen.adjustPacking(+0.05);
            if (gamepad1.dpad_down && !prevDown) pollen.adjustPacking(-0.05);
            if (gamepad1.y && !prevY) { pollen.resetCalibration(); calMsg = "reset"; }
            prevA = gamepad1.a; prevB = gamepad1.b; prevUp = gamepad1.dpad_up;
            prevDown = gamepad1.dpad_down; prevY = gamepad1.y;

            StringBuilder ids = new StringBuilder();
            for (HuskyLens.Block b : pollen.lastRaw) ids.append(ids.length() == 0 ? "" : ", ").append(b.id);
            telemetry.addLine("===== HUSKYLENS =====");
            telemetry.addData("Connection", connected ? "CONNECTED"
                    : "NOT RESPONDING - check I2C cable / port / Protocol Type = I2C");
            telemetry.addData("IDs seen", ids.length() == 0 ? "none" : ids.toString());

            telemetry.addLine("===== POLLEN =====");
            telemetry.addData("Clusters seen", clusters.size());
            telemetry.addData("Total pollen (steady)", Math.round(pollen.smoothedTotal));
            telemetry.addData("Best cluster (steady)", Math.round(pollen.smoothedBestCount) + " pollen");

            for (int i = 0; i < clusters.size(); i++) {
                HuskyLensPollenClusters.Cluster c = clusters.get(i);
                String side = c.bearingDeg > 3 ? "RIGHT" : c.bearingDeg < -3 ? "LEFT" : "CENTER";
                telemetry.addData("Cluster " + (i + 1), "%d pollen  |  %s  %.1f deg  (%d boxes)",
                        c.estimatedBalls, side, c.bearingDeg, c.blocks.size());
            }

            // Debug: raw boxes HuskyLens sends. If this is much bigger than the ball
            // count, HuskyLens is splitting balls into pieces -> re-learn the color.
            telemetry.addData("debug: raw yellow boxes", pollen.rawBoxCount);
            for (HuskyLensPollenClusters.Cluster c : clusters) {
                telemetry.addData("debug: one ball expected", "%.0f px wide at y=%.0f",
                        pollen.ballDiameterAt(c.centerY), c.centerY);
            }

            telemetry.addLine("--- calibration: A far, B near, dpad up/down, Y reset ---");
            telemetry.addData("far / near", "%.0fpx @y%.0f  /  %.0fpx @y%.0f",
                    pollen.ballDiamFarPx, pollen.ballFarY, pollen.ballDiamNearPx, pollen.ballNearY);
            telemetry.addData("packing", "%.2f", pollen.packing);
            if (!calMsg.isEmpty()) telemetry.addLine(calMsg);
            telemetry.update();
        }
    }
}