package org.firstinspires.ftc.teamcode.Autos;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import com.skeletonarmy.marrow.TimerEx;
import com.skeletonarmy.marrow.prompts.BooleanPrompt;
import com.skeletonarmy.marrow.prompts.MessagePrompt;
import com.skeletonarmy.marrow.prompts.Prompter;
import com.skeletonarmy.marrow.prompts.ValuePrompt;

import org.firstinspires.ftc.teamcode.pedro.Constants;

/*
  Pollen Collector - P control to field coordinates + HuskyLens + Marrow
  ---------------------------------------------------------------------
  The Pinpoint tracks where the robot is on the field (x, y in inches, heading).
  A P controller drives the mecanum base to each target coordinate:

      power = kP * error        (error = target - where we are now)

  So the robot drives fast when it's far away and slows down as it gets close.

  The routine:
    1. Drive (P control) to each SEARCH SPOT coordinate
    2. At each spot, look for pollen with the HuskyLens and P-steer onto it
    3. Intake it, then go to the next spot
    4. When it has enough pollen or time is almost up, drive to the SCORE coordinate

  Coordinates use the same field as your Pedro autos (0-144 inches, heading in degrees).
  The ones below are PLACEHOLDERS - measure your real BIOBUZZ spots and change them.
*/

@Autonomous(name = "Pollen Collector (P)", group = "Autonomous")
public class PollenCollector extends LinearOpMode {

    // ---------- Field coordinates (CHANGE THESE) ----------
    private final PoseFactory p = PoseFactory.degrees();
    private final Pose START = p.of(72, 9, 90);       // where the robot starts
    private final Pose[] SEARCH_SPOTS = {             // where to look for pollen
            p.of(48, 48, 90),
            p.of(72, 60, 90),
            p.of(96, 48, 90),
    };
    private final Pose SCORE = p.of(72, 100, 90);     // where to go when done

    // ---------- P gains for driving to coordinates ----------
    private static final double KP_DRIVE    = 0.06;  // forward/back power per inch of error
    private static final double KP_STRAFE   = 0.08;  // sideways power per inch (strafing needs more)
    private static final double KP_TURN     = 1.2;   // turn power per radian of heading error
    private static final double MAX_POWER   = 0.6;   // speed limit while driving to a point
    private static final double POS_TOL     = 1.0;   // "arrived" when within this many inches...
    private static final double HEAD_TOL    = Math.toRadians(3); // ...and this many degrees
    private static final double DRIVE_TIMEOUT = 4.0; // give up on a point after this many seconds

    // ---------- P gains for steering onto pollen with the camera ----------
    private static final int    POLLEN_ID       = 1;
    private static final int    SCREEN_CENTER   = 160;   // HuskyLens screen is 320 wide
    private static final double KP_VISION_TURN  = 0.004; // turn power per pixel off-center
    private static final double APPROACH_POWER  = 0.3;   // forward power while driving at pollen
    private static final int    CLOSE_WIDTH     = 90;    // box this wide = pollen is close
    private static final double SEARCH_TURN     = 0.2;   // slow spin while looking
    private static final double SEARCH_TIMEOUT  = 3.0;   // seconds to look at one spot

    // ---------- Intake ----------
    private static final double INTAKE_TIME = 1.2;   // seconds to run intake while creeping forward

    // ---------- Match time ----------
    private static final double AUTO_LENGTH  = 30.0; // check the BIOBUZZ manual
    private static final double RETURN_TIME  = 5.0;  // head to SCORE when this many seconds are left

    // ---------- Marrow ----------
    private final Prompter prompter = new Prompter(this);
    private final TimerEx matchTimer = new TimerEx(AUTO_LENGTH);
    private int     maxPollen  = 4;
    private double  startDelay = 0.0;
    private boolean useTimer   = true;

    // ---------- Hardware ----------
    private PinpointLocalizer localizer;
    private HuskyLens huskyLens;
    private DcMotor flwheel, frwheel, blwheel, brwheel, intakemotor;
    private CRServo intakeservo;

    private int pollenCollected = 0;
    private String status = "Init";

    @Override
    public void runOpMode() {
        // Same names as your TeleOp and Constants.java
        flwheel = hardwareMap.get(DcMotor.class, "flwheel");
        frwheel = hardwareMap.get(DcMotor.class, "frwheel");
        blwheel = hardwareMap.get(DcMotor.class, "blwheel");
        brwheel = hardwareMap.get(DcMotor.class, "brwheel");
        flwheel.setDirection(DcMotorSimple.Direction.REVERSE);
        blwheel.setDirection(DcMotorSimple.Direction.REVERSE);
        for (DcMotor m : new DcMotor[]{flwheel, frwheel, blwheel, brwheel}) {
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
        intakemotor = hardwareMap.get(DcMotor.class, "intakemotor");
        intakeservo = hardwareMap.get(CRServo.class, "intakeservo");
        intakeservo.setDirection(DcMotorSimple.Direction.REVERSE);

        // Pinpoint, using the tuned settings from Constants.java
        localizer = new PinpointLocalizer(hardwareMap, Constants.localizerConfig);
        localizer.setPose(START);

        huskyLens = hardwareMap.get(HuskyLens.class, "huskylens");
        boolean cameraFound = huskyLens.knock();
        huskyLens.selectAlgorithm(HuskyLens.Algorithm.COLOR_RECOGNITION);

        // ---------- Pre-match settings (Marrow Prompter) ----------
        prompter.prompt("max_pollen", new ValuePrompt<>("How many pollen to collect?", Integer.class, 1, 4, 4, 1))
                    .label("Max pollen")
                .prompt("delay", new ValuePrompt<>("Start delay (seconds)", Double.class, 0.0, 10.0, 0.0, 0.5))
                    .label("Start delay")
                .prompt("use_timer", new BooleanPrompt("Go to SCORE before auto ends?", true))
                    .label("Use match timer")
                .prompt(new MessagePrompt(cameraFound
                        ? "HuskyLens ready. Place robot at the START coordinate!"
                        : "WARNING: HuskyLens NOT FOUND - check wiring/config"))
                .showSummary()
                .onComplete(() -> {
                    maxPollen  = prompter.get("max_pollen");
                    startDelay = prompter.get("delay");
                    useTimer   = prompter.get("use_timer");
                });

        while (opModeInInit()) {
            prompter.run();
            localizer.update();
        }
        if (isStopRequested()) return;

        // ---------- Autonomous ----------
        matchTimer.start();
        waitSeconds(startDelay);

        for (Pose spot : SEARCH_SPOTS) {
            if (!opModeIsActive() || pollenCollected >= maxPollen || timeToReturn()) break;

            status = "Driving to search spot";
            driveTo(spot);

            // Keep grabbing pollen at this spot until none are left in view
            while (opModeIsActive() && pollenCollected < maxPollen && !timeToReturn()) {
                if (!findAndGrabPollen()) break;
            }
        }

        status = "Driving to SCORE";
        intakeOff();
        driveTo(SCORE);

        stopDrive();
        status = "Done - collected " + pollenCollected;
        showTelemetry(null);
        while (opModeIsActive()) {
            localizer.update();
            showTelemetry(null);
        }
    }

    // =====================================================================
    //  P CONTROLLER: drive to a field coordinate
    // =====================================================================
    private void driveTo(Pose target) {
        ElapsedTime timer = new ElapsedTime();

        while (opModeIsActive() && timer.seconds() < DRIVE_TIMEOUT) {
            localizer.update();
            Pose now = localizer.pose();

            // 1. Error = target - current (field coordinates)
            double errX = target.x() - now.x();
            double errY = target.y() - now.y();
            double errH = angleWrap(target.heading() - now.heading());

            // 2. Done?
            if (Math.hypot(errX, errY) < POS_TOL && Math.abs(errH) < HEAD_TOL) break;

            // 3. Turn the field error into the robot's point of view
            double h = now.heading();
            double forwardErr =  errX * Math.cos(h) + errY * Math.sin(h);
            double leftErr    = -errX * Math.sin(h) + errY * Math.cos(h);

            // 4. P control: power = kP * error
            double forward = KP_DRIVE  * forwardErr;
            double left    = KP_STRAFE * leftErr;
            double turn    = KP_TURN   * errH;    // + = turn left (counter-clockwise)

            setMecanum(forward, left, turn, MAX_POWER);
            showTelemetry(target);
        }
        stopDrive();
    }

    // =====================================================================
    //  Vision: P-steer onto the closest pollen and intake it
    //  Returns false if no pollen was found at this spot.
    // =====================================================================
    private boolean findAndGrabPollen() {
        ElapsedTime searchTimer = new ElapsedTime();

        while (opModeIsActive() && !timeToReturn()) {
            localizer.update();
            HuskyLens.Block pollen = findClosestPollen();

            if (pollen == null) {
                if (searchTimer.seconds() > SEARCH_TIMEOUT) return false;  // nothing here
                status = "Searching...";
                setMecanum(0, 0, SEARCH_TURN, 1.0);                        // slow spin left
            } else if (pollen.width >= CLOSE_WIDTH) {
                grabPollen();
                return true;
            } else {
                // P control on the pixel error: off to the right -> turn right
                status = "Driving to pollen";
                double pixelErr = pollen.x - SCREEN_CENTER;
                double turn = -KP_VISION_TURN * pixelErr;
                setMecanum(APPROACH_POWER, 0, turn, 1.0);
                searchTimer.reset();
            }
            showTelemetry(null);
        }
        return false;
    }

    private void grabPollen() {
        status = "Intaking";
        intakeOn();
        ElapsedTime timer = new ElapsedTime();
        while (opModeIsActive() && timer.seconds() < INTAKE_TIME) {
            localizer.update();
            setMecanum(0.25, 0, 0, 1.0);   // creep forward into it
            showTelemetry(null);
        }
        stopDrive();
        waitSeconds(0.3);                   // let the intake finish pulling it in
        intakeOff();
        pollenCollected++;
    }

    private HuskyLens.Block findClosestPollen() {
        HuskyLens.Block[] blocks = huskyLens.blocks();
        if (blocks == null) return null;
        HuskyLens.Block best = null;
        for (HuskyLens.Block b : blocks) {
            if (b.id == POLLEN_ID && (best == null || b.width > best.width)) best = b;
        }
        return best;
    }

    // =====================================================================
    //  Helpers
    // =====================================================================

    /** forward, left (strafe), turn (+ = counter-clockwise). Same wheel math as your TeleOp. */
    private void setMecanum(double forward, double left, double turn, double maxPower) {
        double y  = Range.clip(forward, -maxPower, maxPower);
        double x  = Range.clip(-left,   -maxPower, maxPower);   // TeleOp uses + = right
        double rx = Range.clip(-turn,   -maxPower, maxPower);   // TeleOp uses + = clockwise

        double lf = y + x + rx;
        double lb = y - x + rx;
        double rf = y - x - rx;
        double rb = y + x - rx;

        // Scale down if any wheel is over 1 so the robot keeps its direction
        double biggest = Math.max(1.0, Math.max(Math.max(Math.abs(lf), Math.abs(lb)),
                                                Math.max(Math.abs(rf), Math.abs(rb))));
        flwheel.setPower(lf / biggest);
        blwheel.setPower(lb / biggest);
        frwheel.setPower(rf / biggest);
        brwheel.setPower(rb / biggest);
    }

    private void stopDrive() { setMecanum(0, 0, 0, 1.0); }

    private void intakeOn()  { intakemotor.setPower(1); intakeservo.setPower(1); }
    private void intakeOff() { intakemotor.setPower(0); intakeservo.setPower(0); }

    /** Keeps an angle between -180 and +180 degrees (in radians) so the robot turns the short way. */
    private double angleWrap(double radians) {
        return Math.atan2(Math.sin(radians), Math.cos(radians));
    }

    private boolean timeToReturn() {
        return useTimer && matchTimer.isLessThan(RETURN_TIME);
    }

    private void waitSeconds(double s) {
        ElapsedTime t = new ElapsedTime();
        while (opModeIsActive() && t.seconds() < s) {
            localizer.update();
            idle();
        }
    }

    private void showTelemetry(Pose target) {
        Pose now = localizer.pose();
        telemetry.addData("Status", status);
        telemetry.addData("Pollen", pollenCollected + " / " + maxPollen);
        telemetry.addData("Time left", "%.1f s", matchTimer.getRemaining());
        telemetry.addData("Robot", "x %.1f  y %.1f  h %.0f°",
                now.x(), now.y(), Math.toDegrees(now.heading()));
        if (target != null) {
            telemetry.addData("Target", "x %.1f  y %.1f  h %.0f°",
                    target.x(), target.y(), Math.toDegrees(target.heading()));
        }
        telemetry.update();
    }
}
