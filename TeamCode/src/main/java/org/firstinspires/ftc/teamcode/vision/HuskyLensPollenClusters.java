package org.firstinspires.ftc.teamcode.vision;

import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.File;
import java.util.ArrayList;
import java.util.List;

/**
 * Shows how many pollen are in each cluster the HuskyLens sees (COLOR_RECOGNITION mode).
 *
 * HuskyLens does the color detection on the camera and only sends back boxes
 * (x, y = center; width, height). A pile of touching yellow balls usually comes
 * back as ONE big box, so this OpMode:
 *   1. Estimates how many balls are in each box (box area / one ball's area,
 *      where one ball's size depends on how far down the image it is).
 *   2. Merges boxes that are close together into one cluster.
 *   3. Scores clusters: more balls + closer to robot = better.
 *
 * HuskyLens coordinates are 320 x 240, origin top-left.
 *
 * CALIBRATION (on the robot, with gamepad 1 -- saved to the Control Hub, so do it once):
 *   A          = ONE ball on the floor FAR away       -> records far size
 *   B          = ONE ball on the floor CLOSE (fully in view) -> records near size
 *   dpad up/dn = pile of a known count reads too low / too high -> adjusts packing
 *   Y          = reset calibration to defaults
 */
@TeleOp(name = "HuskyLens Pollen Clusters", group = "Vision")
public class HuskyLensPollenClusters extends LinearOpMode {

    public static final int IMG_W = 320, IMG_H = 240;
    private static final String CAL_FILE = "pollen_cal.txt";

    // ---------- defaults (overwritten by the saved calibration) ----------
    public double horizontalFovDeg = 60;  // HuskyLens is roughly 60 deg; measure yours
    public double ballDiamFarPx    = 20;  // one ball's size (px) at the far calibration spot
    public double ballFarY         = 60;  // that ball's center y at the far spot
    public double ballDiamNearPx   = 70;  // one ball's size (px) at the near calibration spot
    public double ballNearY        = 200; // that ball's center y at the near spot
    public double packing          = 0.75; // fraction of a pile's box actually filled by balls
    public double mergeGapFactor   = 1.0; // merge boxes if the gap between them < factor * ball diameter
    public double closenessWeight  = 1.5; // favor clusters near the bottom (closer)
    public int    pollenColorId    = 1;   // the ID you trained yellow as on the HuskyLens
    // ---------------------------------------------------------------------

    public static class Cluster {
        public final List<HuskyLens.Block> blocks = new ArrayList<>();
        public int estimatedBalls;
        public double centerX, centerY;
        public double bearingDeg;   // + = right of camera center
        public double score;
    }

    private HuskyLens huskyLens;
    private HuskyLens.Block[] lastRaw = new HuskyLens.Block[0];

    @Override
    public void runOpMode() {
        huskyLens = hardwareMap.get(HuskyLens.class, "huskylens");
        boolean loaded = loadCalibration();
        boolean connected = huskyLens.knock();
        if (connected) huskyLens.selectAlgorithm(HuskyLens.Algorithm.COLOR_RECOGNITION);
        ElapsedTime knockTimer = new ElapsedTime();

        // Keep checking the connection during init so plugging the cable in shows up right away
        while (opModeInInit()) {
            if (knockTimer.seconds() > 1) {
                boolean now = huskyLens.knock();
                if (now && !connected) huskyLens.selectAlgorithm(HuskyLens.Algorithm.COLOR_RECOGNITION);
                connected = now;
                knockTimer.reset();
            }
            telemetry.addData("HuskyLens", connected ? "CONNECTED. Press START."
                    : "NOT RESPONDING - check I2C cable / port / Protocol Type = I2C");
            telemetry.addData("Calibration", loaded ? "loaded from file" : "NOT calibrated yet - use A / B");
            telemetry.update();
            sleep(50);
        }

        boolean prevA = false, prevB = false, prevUp = false, prevDown = false, prevY = false;
        String calMsg = "";
        ElapsedTime loopTimer = new ElapsedTime();
        double loopsPerSec = 0;

        while (opModeIsActive()) {
            if (knockTimer.seconds() > 1) {
                boolean now = huskyLens.knock();
                if (now && !connected) huskyLens.selectAlgorithm(HuskyLens.Algorithm.COLOR_RECOGNITION);
                connected = now;
                knockTimer.reset();
            }
            List<Cluster> clusters = update();
            loopsPerSec = 0.9 * loopsPerSec + 0.1 / Math.max(1e-3, loopTimer.seconds());
            loopTimer.reset();

            // ---------- HuskyLens status ----------
            StringBuilder ids = new StringBuilder();
            for (HuskyLens.Block b : lastRaw) ids.append(ids.length() == 0 ? "" : ", ").append(b.id);
            telemetry.addLine("===== HUSKYLENS STATUS =====");
            telemetry.addData("Connection", connected ? "CONNECTED"
                    : "NOT RESPONDING - check I2C cable / port / Protocol Type = I2C");
            telemetry.addData("Boxes (any ID)", lastRaw.length);
            telemetry.addData("IDs seen", ids.length() == 0 ? "none" : ids.toString());
            telemetry.addData("Pollen ID", pollenColorId);
            telemetry.addData("Loop", "%.0f / sec", loopsPerSec);

            // ---------- gamepad calibration ----------
            HuskyLens.Block biggest = biggestPollenBlock();
            if (gamepad1.a && !prevA) {
                if (biggest == null) calMsg = "A: no yellow box seen";
                else {
                    ballDiamFarPx = Math.max(biggest.width, biggest.height);
                    ballFarY = biggest.y;
                    saveCalibration();
                    calMsg = String.format("A: far ball = %.0fpx at y=%.0f", ballDiamFarPx, ballFarY);
                }
            }
            if (gamepad1.b && !prevB) {
                if (biggest == null) calMsg = "B: no yellow box seen";
                else {
                    ballDiamNearPx = Math.max(biggest.width, biggest.height);
                    ballNearY = biggest.y;
                    saveCalibration();
                    calMsg = String.format("B: near ball = %.0fpx at y=%.0f", ballDiamNearPx, ballNearY);
                }
            }
            // more packing = more balls counted per box
            if (gamepad1.dpad_up && !prevUp)     { packing = packing + 0.05; saveCalibration(); }
            if (gamepad1.dpad_down && !prevDown) { packing = Math.max(0.1, packing - 0.05); saveCalibration(); }
            if (gamepad1.y && !prevY) {
                ballDiamFarPx = 20; ballFarY = 60; ballDiamNearPx = 70; ballNearY = 200; packing = 0.75;
                saveCalibration();
                calMsg = "Y: reset to defaults";
            }
            prevA = gamepad1.a; prevB = gamepad1.b; prevUp = gamepad1.dpad_up;
            prevDown = gamepad1.dpad_down; prevY = gamepad1.y;

            // ---------- telemetry ----------
            int totalPollen = 0;
            for (Cluster c : clusters) totalPollen += c.estimatedBalls;

            telemetry.addLine("===== POLLEN =====");
            telemetry.addData("Clusters seen", clusters.size());
            telemetry.addData("Total pollen", totalPollen);

            if (clusters.isEmpty()) {
                telemetry.addLine("No pollen visible");
            } else {
                for (int i = 0; i < clusters.size(); i++) {
                    Cluster c = clusters.get(i);
                    String side = c.bearingDeg > 3 ? "RIGHT" : c.bearingDeg < -3 ? "LEFT" : "CENTER";
                    String label = (i == 0) ? "Cluster 1 (BEST)" : "Cluster " + (i + 1);
                    telemetry.addData(label, "%d pollen  |  %s  %.1f deg",
                            c.estimatedBalls, side, c.bearingDeg);
                }
            }

            telemetry.addLine("----- calibration (A far, B near, dpad = count up/down, Y reset) -----");
            telemetry.addData("far", "%.0fpx at y=%.0f", ballDiamFarPx, ballFarY);
            telemetry.addData("near", "%.0fpx at y=%.0f", ballDiamNearPx, ballNearY);
            telemetry.addData("packing", "%.2f", packing);
            if (!calMsg.isEmpty()) telemetry.addLine(calMsg);

            telemetry.addLine("----- raw boxes -----");
            for (HuskyLens.Block b : lastRaw) {
                if (b.id != pollenColorId) continue;
                double d = ballDiameterAt(b.y);
                telemetry.addData("raw", "x=%d y=%d w=%d h=%d | 1 ball=%.0fpx -> %d balls",
                        b.x, b.y, b.width, b.height, d, ballsInBox(b));
                android.util.Log.i("PollenRaw", String.format("x=%d y=%d w=%d h=%d d=%.0f",
                        b.x, b.y, b.width, b.height, d));
            }
            telemetry.update();
        }
    }

    /** Expected diameter (px) of a single ball on the floor whose center is at image row y. */
    public double ballDiameterAt(double y) {
        double dy = ballNearY - ballFarY;
        if (Math.abs(dy) < 1) return Math.max(5, (ballDiamFarPx + ballDiamNearPx) / 2);
        double t = (y - ballFarY) / dy;
        return Math.max(5, ballDiamFarPx + t * (ballDiamNearPx - ballDiamFarPx));
    }

    /** Estimated number of balls in one HuskyLens box. */
    public int ballsInBox(HuskyLens.Block b) {
        double d = ballDiameterAt(b.y);
        double oneBallArea = Math.PI / 4 * d * d;
        long byArea = Math.round(b.width * b.height * packing / oneBallArea);
        long byWidth = Math.round(b.width / d);   // a row of balls side by side is at least this many
        return (int) Math.max(1, Math.max(byArea, byWidth));
    }

    /** Reads HuskyLens and returns clusters, best first. Empty list if nothing seen. */
    public List<Cluster> update() {
        HuskyLens.Block[] got = huskyLens.blocks();
        lastRaw = (got == null) ? new HuskyLens.Block[0] : got;
        List<HuskyLens.Block> blocks = new ArrayList<>();
        for (HuskyLens.Block b : lastRaw) {
            if (b.id == pollenColorId) blocks.add(b);
        }

        // Union-find merge of nearby boxes
        int n = blocks.size();
        int[] parent = new int[n];
        for (int i = 0; i < n; i++) parent[i] = i;
        for (int i = 0; i < n; i++) {
            for (int j = i + 1; j < n; j++) {
                HuskyLens.Block a = blocks.get(i), b = blocks.get(j);
                double gapX = Math.abs(a.x - b.x) - (a.width + b.width) / 2.0;
                double gapY = Math.abs(a.y - b.y) - (a.height + b.height) / 2.0;
                double gap = Math.max(gapX, gapY);   // < 0 means boxes overlap
                double d = ballDiameterAt((a.y + b.y) / 2.0);
                if (gap < mergeGapFactor * d) union(parent, i, j);
            }
        }

        List<Cluster> clusters = new ArrayList<>();
        Cluster[] byRoot = new Cluster[n];
        for (int i = 0; i < n; i++) {
            int r = find(parent, i);
            if (byRoot[r] == null) { byRoot[r] = new Cluster(); clusters.add(byRoot[r]); }
            byRoot[r].blocks.add(blocks.get(i));
        }

        for (Cluster c : clusters) {
            double sx = 0, sy = 0, wsum = 0;
            int balls = 0;
            for (HuskyLens.Block b : c.blocks) {
                int est = ballsInBox(b);
                balls += est;
                sx += b.x * est; sy += b.y * est; wsum += est;   // weight center by ball count
            }
            c.estimatedBalls = balls;
            c.centerX = sx / wsum;
            c.centerY = sy / wsum;
            c.bearingDeg = (c.centerX - IMG_W / 2.0) / IMG_W * horizontalFovDeg;
            c.score = balls * (1.0 + closenessWeight * c.centerY / IMG_H);
        }
        clusters.sort((a, b) -> Double.compare(b.score, a.score));
        return clusters;
    }

    private HuskyLens.Block biggestPollenBlock() {
        HuskyLens.Block best = null;
        for (HuskyLens.Block b : lastRaw) {
            if (b.id != pollenColorId) continue;
            if (best == null || b.width * b.height > best.width * best.height) best = b;
        }
        return best;
    }

    private void saveCalibration() {
        File f = AppUtil.getInstance().getSettingsFile(CAL_FILE);
        ReadWriteFile.writeFile(f, ballDiamFarPx + "," + ballFarY + "," + ballDiamNearPx + ","
                + ballNearY + "," + packing);
    }

    private boolean loadCalibration() {
        try {
            File f = AppUtil.getInstance().getSettingsFile(CAL_FILE);
            if (!f.exists()) return false;
            String[] v = ReadWriteFile.readFile(f).trim().split(",");
            ballDiamFarPx  = Double.parseDouble(v[0]);
            ballFarY       = Double.parseDouble(v[1]);
            ballDiamNearPx = Double.parseDouble(v[2]);
            ballNearY      = Double.parseDouble(v[3]);
            packing        = Double.parseDouble(v[4]);
            return true;
        } catch (Exception e) {
            return false;
        }
    }

    private static int find(int[] p, int i) {
        while (p[i] != i) { p[i] = p[p[i]]; i = p[i]; }
        return i;
    }
    private static void union(int[] p, int a, int b) {
        p[find(p, a)] = find(p, b);
    }
}
