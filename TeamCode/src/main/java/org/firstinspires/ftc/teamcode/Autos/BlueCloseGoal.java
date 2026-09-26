package org.firstinspires.ftc.teamcode.Autos; // make sure this aligns with class location

import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import static com.pedropathing.api.Paths.*;

import org.firstinspires.ftc.teamcode.pedro.Constants;

// NOTE: com.pedropathing:ivy:1.0.0 (used here pre-3.0 for Command/Scheduler-based sequencing)
// does not compile against Pedro 3.0.0 yet -- it's still built against the old
// com.pedropathing.paths.PathChain type, which no longer exists. Until Pedro-Pathing ships a
// 3.0-compatible Ivy release, this routine is sequenced manually with follower.follow(path) /
// follower.isBusy() instead of Ivy's Command/Scheduler. Re-introduce Ivy here once available.
@Autonomous(name = "BlueCLoseAuto", group = "Autonomous")
public class BlueCloseGoal extends LinearOpMode {

    private Follower follower;
    private final PoseFactory p = PoseFactory.degrees();
    private final Pose startPose = p.of(33, 133, 90); // Start Pose of our robot. This is against the goal facing AWAY
    private final Pose scorePose = p.of(58, 84, 180); // Scoring Pose of our robot.
    private final Pose pickup1Pose = p.of(12, 84, 180); // Highest (First Set) of Artifacts from the Spike Mark.
    private final Pose pickup2Pose = p.of(12, 60, 180); // Middle (Second Set) of Artifacts from the Spike Mark.
    private final Pose pickup3Pose = p.of(12, 36, 180); // Lowest (Third Set) of Artifacts from the Spike Mark.
    private final Pose endPose = p.of(38, 72, 0); // Final Pose of our robot, off the starting line

    //defining our Paths
    private Path scorePreload, grabPickup1, scorePickup1, grabPickup2, scorePickup2, grabPickup3, scorePickup3, leave;
    private Path[] routine;
    private int step = 0;

    public void buildPaths() {
        scorePreload = line(startPose, scorePose).linear(startPose, scorePose);

        /* This is our grabPickup1 Path. We are using a single path with a line, which is a straight line. */
        grabPickup1 = line(scorePose, pickup1Pose).linear(scorePose, pickup1Pose);

        /* This is our scorePickup1 Path. We are using a single path with a line, which is a straight line. */
        scorePickup1 = line(pickup1Pose, scorePose).linear(pickup1Pose, scorePose);

        /* This is our grabPickup2 Path. We are using a single path with a curve (curved line). */
        grabPickup2 = curve(scorePose, p.of(60, 54, 0), pickup2Pose).linear(scorePose, pickup2Pose);

        /* This is our scorePickup2 Path. We are using a single path with a curve (curved line). */
        scorePickup2 = curve(pickup2Pose, p.of(60, 54, 0), scorePose).linear(pickup2Pose, scorePose);

        /* This is our grabPickup3 Path. We are using a single path with a curve (curved line). */
        grabPickup3 = curve(scorePose, p.of(60, 21, 0), pickup3Pose).linear(scorePose, pickup3Pose);

        /* This is our scorePickup3 Path. We are using a single path with a curve (curved line). */
        scorePickup3 = curve(pickup3Pose, p.of(60, 30, 0), scorePose).linear(pickup3Pose, scorePose);

        /* This is our leave Path. We are using a single path using a line (straight line).
         * We use Constant Interpolation here instead of Linear*/
        leave = line(scorePose, endPose).constant(scorePose);

        routine = new Path[] {
                scorePreload,   // score preload
                grabPickup1,    // first 3 balls intake
                scorePickup1,   // fire first 3 balls
                grabPickup2,    // 2nd 3 balls intake
                scorePickup2,   // fire 2nd 3 balls
                grabPickup3,    // 3rd 3 balls intake
                scorePickup3,   // fire 3rd 3 balls
                leave           // leave score zone
        };
    }

    @Override
    public void runOpMode() {
        //These will run when the OpMode is initiated
        follower = Constants.create(hardwareMap);
        buildPaths();
        follower.setPose(startPose);

        waitForStart();
        //Start the first path in the routine
        step = 0;
        follower.follow(routine[step]);
        while (opModeIsActive()) {
            //Update the follower and advance to the next path once the current one finishes
            follower.update();

            if (!follower.isBusy() && step < routine.length - 1) {
                step++;
                follower.follow(routine[step]);
            }

            // Feedback to Driver Hub for debugging
            telemetry.addData("step", step);
            telemetry.addData("x", follower.pose().x());
            telemetry.addData("y", follower.pose().y());
            telemetry.addData("heading", follower.pose().heading());
            telemetry.update();
        }
    }
}
