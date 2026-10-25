package org.firstinspires.ftc.teamcode.vision;

import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.util.ArrayList;
import java.util.List;

/**
 * SIMPLE pollen counter for HuskyLens (Color Recognition, yellow learned as ID 1).
 *
 * How to use:
 *   1. Show the HuskyLens ONE ball and press X (cross) on the gamepad. That's it.
 *   2. Now show it a pile at about the same distance -> it tells you how many pollen.
 *   d-pad up/down fine-tunes if the count is a bit off.
 *
 * Everything is in this one file. No other files needed.
 */
@TeleOp(name = "Pollen Counter (Simple)", group = "Vision")
public class HuskyLensPollenSimple extends LinearOpMode {

    static final int POLLEN_ID = 1;          // color ID you learned yellow as
    static final int MIN_BOX_PX = 8;         // ignore tiny noise boxes

    @Override
    public void runOpMode() {
        HuskyLens husky = hardwareMap.get(HuskyLens.class, "huskylens");
        husky.selectAlgorithm(HuskyLens.Algorithm.COLOR_RECOGNITION);
        telemetry.addData("HuskyLens", husky.knock() ? "connected" : "NOT FOUND - check config name");
        telemetry.addLine("Press START, then show ONE ball and press X (cross).");
        telemetry.update();
        waitForStart();

        double oneBallArea = 0;    // learned when you press X
        double tune = 1.0;         // d-pad fine tune
        boolean lastX = false, lastUp = false, lastDown = false;

        // Steadiness: keep the last 25 readings, use the middle value (median),
        // and only change the shown number after it has been different for a while.
        final int HISTORY = 25, HOLD_FRAMES = 15;
        java.util.ArrayDeque<Integer> history = new java.util.ArrayDeque<>();
        int shown = 0, candidate = 0, candidateFrames = 0;
        java.util.ArrayDeque<Double> areaHistory = new java.util.ArrayDeque<>();

        while (opModeIsActive()) {
            // ---- read yellow boxes ----
            List<int[]> boxes = new ArrayList<>();   // {left, top, right, bottom}
            HuskyLens.Block[] seen = husky.blocks();
            if (seen == null) seen = new HuskyLens.Block[0];   // null when the HuskyLens doesn't answer
            for (HuskyLens.Block b : seen) {
                if (b.id == POLLEN_ID && b.width >= MIN_BOX_PX && b.height >= MIN_BOX_PX) {
                    boxes.add(new int[]{b.x - b.width / 2, b.y - b.height / 2,
                            b.x + b.width / 2, b.y + b.height / 2});
                }
            }

            // ---- one outer box around everything yellow = the cluster ----
            double area = 0;
            if (!boxes.isEmpty()) {
                int l = 9999, t = 9999, r = -1, btm = -1;
                for (int[] bx : boxes) {
                    l = Math.min(l, bx[0]); t = Math.min(t, bx[1]);
                    r = Math.max(r, bx[2]); btm = Math.max(btm, bx[3]);
                }
                area = (double) (r - l) * (btm - t);
            }

            // keep recent areas so "learn one ball" uses a steady average
            if (area > 0) {
                areaHistory.addLast(area);
                if (areaHistory.size() > 15) areaHistory.removeFirst();
            }

            // ---- buttons ----
            boolean x = gamepad1.a;              // A = X/cross on REV gamepad
            if (x && !lastX && !areaHistory.isEmpty()) {
                double sum = 0;
                for (double a : areaHistory) sum += a;
                oneBallArea = sum / areaHistory.size();
                tune = 1.0;
                history.clear(); shown = 1; candidate = 1; candidateFrames = 0;
            }
            lastX = x;
            if (gamepad1.dpad_up && !lastUp)     tune += 0.05;
            if (gamepad1.dpad_down && !lastDown) tune = Math.max(0.2, tune - 0.05);
            lastUp = gamepad1.dpad_up; lastDown = gamepad1.dpad_down;

            // ---- count ----
            telemetry.addLine("===== POLLEN COUNTER =====");
            if (oneBallArea == 0) {
                telemetry.addLine(">> Show ONE ball, then press X (cross)");
            } else {
                // this frame's raw count
                int raw = (area == 0) ? 0
                        : (int) Math.max(1, Math.round(area / (oneBallArea * 1.25 * tune)));

                history.addLast(raw);
                if (history.size() > HISTORY) history.removeFirst();

                // median of recent readings ignores one-off jumps
                List<Integer> sorted = new ArrayList<>(history);
                java.util.Collections.sort(sorted);
                int median = sorted.get(sorted.size() / 2);

                // only change the shown number after the new value holds steady
                if (median == shown) {
                    candidateFrames = 0;
                } else if (median == candidate) {
                    if (++candidateFrames >= HOLD_FRAMES) { shown = candidate; candidateFrames = 0; }
                } else {
                    candidate = median; candidateFrames = 1;
                }

                telemetry.addData("POLLEN IN CLUSTER", shown);
                telemetry.addData("(live reading)", raw);
            }

            telemetry.addLine("");
            telemetry.addData("tune (d-pad up/down)", "%.2f", tune);
            telemetry.addData("yellow boxes seen", boxes.size());
            telemetry.addLine("Too high? d-pad UP.  Too low? d-pad DOWN.  Re-learn: X");
            telemetry.update();
        }
    }
}