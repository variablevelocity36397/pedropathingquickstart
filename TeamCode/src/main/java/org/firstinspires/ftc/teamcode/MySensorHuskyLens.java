package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.internal.system.Deadline;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.TimeUnit;

/*
 * HuskyLens ball tracker.
 *
 * Reports the closest PURPLE ball and the closest GREEN ball, with a
 * horizontal aim offset for each.
 *
 * It does NOT count balls. COLOR_RECOGNITION returns one box per learned ID,
 * so a true count is impossible on this hardware.
 *
 * LEARN EXACTLY TWO IDs, with the LED ON:  ID 1 = purple,  ID 2 = green.
 */
@TeleOp(name = "HuskyLens: Ball Tracker", group = "Sensor")
public class MySensorHuskyLens extends LinearOpMode {

    private static final int READ_PERIOD_MS = 100;      // 10 reads/sec

    /** Ignore blobs smaller than this. Keep low -- partial bands are small. */
    private static final int MIN_WIDTH  = 8;
    private static final int MIN_HEIGHT = 8;

    /** Reject smears. Loose on purpose: colour recognition returns a band,
     *  not a circle. Set to 99 to disable. */
    private static final double MAX_ASPECT = 4.0;

    /** Different-coloured boxes overlapping this much are the same ball --
     *  keep the bigger, drop the other. */
    private static final double CONFLICT_OVERLAP = 0.40;

    /** Hold a lost ball at its last position for this long before giving up. */
    private static final long HOLD_MS = 400;

    /** Frames a ball must appear in a row before it counts as real. */
    private static final int MIN_CONSECUTIVE = 2;

    /** Which colour each learned ID is. Index == ID number. */
    private static final String[] COLOUR_OF_ID = {
            null,       // ID 0 - unlearned
            "PURPLE",   // ID 1
            "GREEN"     // ID 2
    };

    private static final int IMAGE_WIDTH = 320;         // HuskyLens is 320x240
    private static final int CENTER_X    = IMAGE_WIDTH / 2;

    private HuskyLens huskyLens;

    private final Tracker[] trackers = { new Tracker("PURPLE"), new Tracker("GREEN") };

    @Override
    public void runOpMode() {

        huskyLens = hardwareMap.get(HuskyLens.class, "huskylens");

        Deadline rateLimit = new Deadline(READ_PERIOD_MS, TimeUnit.MILLISECONDS);
        rateLimit.expire();

        if (!huskyLens.knock()) {
            telemetry.addData(">>", "Problem communicating with " + huskyLens.getDeviceName());
        } else {
            telemetry.addData(">>", "HuskyLens OK - LED should be ON - press START");
        }

        huskyLens.selectAlgorithm(HuskyLens.Algorithm.COLOR_RECOGNITION);
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            if (!rateLimit.hasExpired()) {
                continue;
            }
            rateLimit.reset();

            long now = System.currentTimeMillis();

            // ----- 1. read and clean -------------------------------------------
            HuskyLens.Block[] raw = huskyLens.blocks();

            List<Box> boxes = new ArrayList<>();
            int tooSmall = 0, badShape = 0;

            for (HuskyLens.Block b : raw) {
                if (colourOf(b.id) == null) continue;
                if (b.width < MIN_WIDTH || b.height < MIN_HEIGHT) { tooSmall++; continue; }

                double aspect = (double) b.width / (double) b.height;
                if (aspect > MAX_ASPECT || aspect < 1.0 / MAX_ASPECT) { badShape++; continue; }

                boxes.add(new Box(b));
            }

            // ----- 2. one ball can't be two colours ----------------------------
            int conflicts = resolveConflicts(boxes);

            // ----- 3. best (largest) box per colour ----------------------------
            for (Tracker t : trackers) {
                Box best = null;
                for (Box b : boxes) {
                    if (b.colour.equals(t.colour) && (best == null || b.area() > best.area())) {
                        best = b;
                    }
                }
                t.update(best, now);
            }

            // ----- 4. telemetry -------------------------------------------------
            telemetry.addData("Raw blocks", "%d   (small:%d  shape:%d  conflict:%d)",
                    raw.length, tooSmall, badShape, conflicts);
            telemetry.addLine();

            for (Tracker t : trackers) {
                if (t.confirmed(now)) {
                    int offset = t.x - CENTER_X;
                    String side = offset < -10 ? "LEFT" : offset > 10 ? "RIGHT" : "CENTER";
                    telemetry.addData(t.colour, "%-5s x=%3d  %dx%d  off=%+4d %s",
                            t.fresh ? "SEEN" : "HOLD",
                            t.x, t.w, t.h, offset, side);
                } else {
                    telemetry.addData(t.colour, "-- not visible --");
                }
            }

            telemetry.addLine();

            // ----- 5. aim at the purple ball -----------------------------------
            Tracker target = tracker("PURPLE");
            if (target.confirmed(now)) {
                double turn = (target.x - CENTER_X) / (double) CENTER_X;   // -1..+1
                telemetry.addData("Aim PURPLE", "turnPower=%.2f%s",
                        turn, target.fresh ? "" : "   (stale - coasting)");
                // leftMotor.setPower(turn * 0.4);
                // rightMotor.setPower(-turn * 0.4);
            } else {
                telemetry.addData("Aim PURPLE", "no target - stop or search");
                // leftMotor.setPower(0);
                // rightMotor.setPower(0);
            }

            telemetry.update();
        }
    }

    /** Different-coloured boxes on the same spot: keep the larger one. */
    private int resolveConflicts(List<Box> boxes) {
        int dropped = 0;
        boolean changed = true;
        while (changed) {
            changed = false;
            outer:
            for (int i = 0; i < boxes.size(); i++) {
                for (int j = i + 1; j < boxes.size(); j++) {
                    Box a = boxes.get(i), b = boxes.get(j);
                    if (a.colour.equals(b.colour)) continue;
                    if (a.overlapFraction(b) >= CONFLICT_OVERLAP) {
                        boxes.remove(a.area() >= b.area() ? j : i);
                        dropped++;
                        changed = true;
                        break outer;
                    }
                }
            }
        }
        return dropped;
    }

    private Tracker tracker(String colour) {
        for (Tracker t : trackers) {
            if (t.colour.equals(colour)) return t;
        }
        return trackers[0];
    }

    private static String colourOf(int id) {
        return (id >= 0 && id < COLOUR_OF_ID.length) ? COLOUR_OF_ID[id] : null;
    }

    /** A detection from one frame. */
    private static class Box {
        final String colour;
        final int left, top, right, bottom;

        Box(HuskyLens.Block b) {
            colour = colourOf(b.id);
            left = b.left;
            top = b.top;
            right = b.left + b.width;
            bottom = b.top + b.height;
        }

        int width()  { return right - left; }
        int height() { return bottom - top; }
        int x()      { return (left + right) / 2; }
        int y()      { return (top + bottom) / 2; }
        int area()   { return width() * height(); }

        /** Overlap area as a fraction of the smaller box. 0 if disjoint. */
        double overlapFraction(Box o) {
            int ovW = Math.min(right, o.right) - Math.max(left, o.left);
            int ovH = Math.min(bottom, o.bottom) - Math.max(top, o.top);
            if (ovW <= 0 || ovH <= 0) return 0;
            int smaller = Math.min(area(), o.area());
            return smaller <= 0 ? 0 : (double) (ovW * ovH) / (double) smaller;
        }
    }

    /** Remembers one colour across frames so brief dropouts don't flicker. */
    private static class Tracker {
        final String colour;
        int x, y, w, h;
        long lastSeen = 0;
        int consecutive = 0;
        boolean fresh = false;      // true = seen this frame, false = coasting

        Tracker(String colour) { this.colour = colour; }

        void update(Box box, long now) {
            if (box != null) {
                x = box.x(); y = box.y();
                w = box.width(); h = box.height();
                lastSeen = now;
                consecutive++;
                fresh = true;
            } else {
                fresh = false;
                if (now - lastSeen > HOLD_MS) {
                    consecutive = 0;
                }
            }
        }

        /** Real enough to act on. */
        boolean confirmed(long now) {
            return consecutive >= MIN_CONSECUTIVE && (now - lastSeen) <= HOLD_MS;
        }
    }
}
