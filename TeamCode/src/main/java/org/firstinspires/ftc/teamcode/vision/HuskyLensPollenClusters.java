package org.firstinspires.ftc.teamcode.vision;

import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.File;
import java.util.ArrayList;
import java.util.List;

/**
 * HELPER CLASS (not an OpMode) used by HuskyLensPollenTest. Don't paste OpModes into this file.
 * The one-file "Pollen Counter (Simple)" OpMode is in HuskyLensPollenSimple.java.
 *
 * Pollen cluster finder for HuskyLens (COLOR_RECOGNITION mode).
 * HuskyLens does the color detection on the camera and only sends back boxes
 * (x, y = center; width, height). A pile of touching yellow balls usually comes
 * back as ONE big box, so this class:
 *   1. Estimates how many balls are in each box (box area / one ball's area,
 *      where one ball's size depends on how far down the image it is).
 *   2. Merges boxes that are close together into one cluster.
 *   3. Scores clusters: more balls + closer to robot = better.
 *
 * HuskyLens coordinates are 320 x 240, origin top-left.
 *
 * Calibration (calibrateFar / calibrateNear / adjustPacking) is saved on the Control Hub
 * and loaded automatically, so it only needs doing once.
 */
public class HuskyLensPollenClusters {

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
    public double smoothing        = 0.2; // 0..1, how fast the "steady" numbers follow new readings
    public int    pollenColorId    = 1;   // the ID you trained yellow as on the HuskyLens
    // ---------------------------------------------------------------------

    public static class Cluster {
        public final List<HuskyLens.Block> blocks = new ArrayList<>();
        public int estimatedBalls;
        public double centerX, centerY;
        public double bearingDeg;   // + = right of camera center
        public double score;
    }

    // Updated by update()
    public HuskyLens.Block[] lastRaw = new HuskyLens.Block[0]; // every box, any ID
    public int rawBoxCount;              // boxes with the pollen ID
    public double smoothedTotal;         // total pollen, smoothed so it doesn't flicker
    public double smoothedBestCount;     // pollen in the best cluster, smoothed
    public final boolean calibrationLoaded;

    private final HuskyLens huskyLens;

    public HuskyLensPollenClusters(HuskyLens huskyLens) {
        this.huskyLens = huskyLens;
        huskyLens.selectAlgorithm(HuskyLens.Algorithm.COLOR_RECOGNITION);
        calibrationLoaded = loadCalibration();
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
            if (b.id == pollenColorId) {
                blocks.add(b);
                android.util.Log.i("PollenRaw", String.format("x=%d y=%d w=%d h=%d d=%.0f",
                        b.x, b.y, b.width, b.height, ballDiameterAt(b.y)));
            }
        }
        rawBoxCount = blocks.size();

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

        int total = 0;
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
            total += balls;
        }
        clusters.sort((a, b) -> Double.compare(b.score, a.score));

        int best = clusters.isEmpty() ? 0 : clusters.get(0).estimatedBalls;
        smoothedTotal     += smoothing * (total - smoothedTotal);
        smoothedBestCount += smoothing * (best - smoothedBestCount);
        return clusters;
    }

    // ---------- calibration ----------

    /** Biggest pollen box in the last update(), or null. Use with ONE ball in view. */
    public HuskyLens.Block biggestPollenBlock() {
        HuskyLens.Block best = null;
        for (HuskyLens.Block b : lastRaw) {
            if (b.id != pollenColorId) continue;
            if (best == null || b.width * b.height > best.width * best.height) best = b;
        }
        return best;
    }

    /** ONE ball on the floor far away. Returns false if no pollen box is visible. */
    public boolean calibrateFar() {
        HuskyLens.Block b = biggestPollenBlock();
        if (b == null) return false;
        ballDiamFarPx = Math.max(b.width, b.height);
        ballFarY = b.y;
        saveCalibration();
        return true;
    }

    /** ONE ball on the floor close up (fully in view). Returns false if no pollen box is visible. */
    public boolean calibrateNear() {
        HuskyLens.Block b = biggestPollenBlock();
        if (b == null) return false;
        ballDiamNearPx = Math.max(b.width, b.height);
        ballNearY = b.y;
        saveCalibration();
        return true;
    }

    /** + = count more balls per box, - = fewer. */
    public void adjustPacking(double delta) {
        packing = Math.max(0.1, packing + delta);
        saveCalibration();
    }

    public void resetCalibration() {
        ballDiamFarPx = 20; ballFarY = 60; ballDiamNearPx = 70; ballNearY = 200; packing = 0.75;
        saveCalibration();
    }

    public void saveCalibration() {
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
