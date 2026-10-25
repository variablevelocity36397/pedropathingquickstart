package org.firstinspires.ftc.teamcode.vision;

import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

// Marrow library (https://marrow.skeletonarmyftc.com)
import com.skeletonarmy.marrow.TimerEx;
import com.skeletonarmy.marrow.prompts.BooleanPrompt;
import com.skeletonarmy.marrow.prompts.MessagePrompt;
import com.skeletonarmy.marrow.prompts.Prompter;
import com.skeletonarmy.marrow.prompts.ValuePrompt;

/*
  Pollen Collector - goBILDA FTC StarterBot (BIOBUZZ 2026-2027) + HuskyLens + Marrow
  ---------------------------------------------------------------------------------
  1. During INIT, the driver picks settings on the Driver Hub with the gamepad (Marrow Prompter)
  2. Spins slowly to SEARCH for the pollen
  3. When HuskyLens sees it, STEERS toward it and drives forward
  4. When the pollen is close, runs the Gecko wheel INTAKE to pull it in
  5. Repeats until it has enough pollen OR autonomous is almost over (Marrow TimerEx)

  SETUP - add Marrow to TeamCode/build.gradle (needs Android Studio):
      dependencies {
          implementation 'com.skeletonarmyftc.marrow:core:1.2.2'
      }

  Prompter controls during INIT:
      D-PAD = change value     A = confirm     B = go back

  Robot configuration names (change to match your Driver Station config):
    "huskylens"   -> HuskyLens plugged into an I2C port on the Control Hub
    "left_drive"  -> left drive motor
    "right_drive" -> right drive motor
    "intake"      -> intake motor

  HuskyLens setup:
    - Mount it on solid U-channel at the front, in front of the top polycarbonate plate
    - Use COLOR RECOGNITION mode
    - Point at a pollen ball and press the learn button so it becomes ID 1
*/

@Disabled  // hidden: uses left_drive/right_drive names this mecanum robot does not have. Use "Pollen Collector (P)".
@Autonomous(name = "Pollen Collector (StarterBot)", group = "HuskyLens")
public class HukyLensPollen_GB_starter_bot extends LinearOpMode {

    // ---------- Tuning (adjust these!) ----------
    private static final int    POLLEN_ID     = 1;     // ID HuskyLens learned for pollen
    private static final int    SCREEN_CENTER = 160;   // HuskyLens screen is 320 wide
    private static final int    CENTER_ZONE   = 25;    // pixels that count as "straight ahead"
    private static final double BASE_SPEED    = 0.4;   // forward power (0 to 1)
    private static final double SEARCH_SPEED  = 0.25;  // spin power while searching
    private static final double TURN_GAIN     = 0.003; // how hard it steers
    private static final int    CLOSE_WIDTH   = 90;    // box this wide = pollen is close
    private static final double INTAKE_POWER  = 1.0;
    private static final double INTAKE_TIME   = 1.5;   // seconds to run intake

    private static final double AUTO_LENGTH   = 30.0;  // autonomous length in seconds (check the BIOBUZZ manual)
    private static final double STOP_TIME     = 2.0;   // stop collecting when this many seconds are left

    // ---------- Marrow ----------
    private final Prompter prompter = new Prompter(this);
    private final TimerEx matchTimer = new TimerEx(AUTO_LENGTH);

    // Settings the driver picks during INIT (these defaults are used if they skip the prompts)
    private int     maxPollen  = 4;      // StarterBot holds up to 4
    private double  startDelay = 0.0;
    private boolean useTimer   = true;

    private int pollenCollected = 0;

    private HuskyLens huskyLens;
    private DcMotor leftDrive, rightDrive, intake;

    @Override
    public void runOpMode() {
        huskyLens  = hardwareMap.get(HuskyLens.class, "huskylens");
        leftDrive  = hardwareMap.get(DcMotor.class, "left_drive");
        rightDrive = hardwareMap.get(DcMotor.class, "right_drive");
        intake     = hardwareMap.get(DcMotor.class, "intake");

        // One side is usually mounted backwards - flip if robot spins instead of driving
        leftDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        rightDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        boolean cameraFound = huskyLens.knock();
        huskyLens.selectAlgorithm(HuskyLens.Algorithm.COLOR_RECOGNITION);

        // ---------- Pre-match settings (Marrow Prompter) ----------
        prompter.prompt("max_pollen", new ValuePrompt<>("How many pollen to collect?", Integer.class, 1, 4, 4, 1))
                .label("Max pollen")
                .prompt("delay", new ValuePrompt<>("Start delay (seconds)", Double.class, 0.0, 10.0, 0.0, 0.5))
                .label("Start delay")
                .prompt("use_timer", new BooleanPrompt("Stop before auto ends?", true))
                .label("Use match timer")
                .prompt(new MessagePrompt(cameraFound
                        ? "HuskyLens ready. Make sure pollen is taught as ID 1!"
                        : "WARNING: HuskyLens NOT FOUND - check wiring/config"))
                .showSummary()
                .onComplete(() -> {
                    maxPollen  = prompter.get("max_pollen");
                    startDelay = prompter.get("delay");
                    useTimer   = prompter.get("use_timer");
                });

        // Show the prompts on the Driver Hub until START is pressed
        while (opModeInInit()) {
            prompter.run();
        }

        if (isStopRequested()) return;

        // ---------- Autonomous starts ----------
        matchTimer.start();

        if (startDelay > 0) {
            sleep((long) (startDelay * 1000));
        }

        while (opModeIsActive() && pollenCollected < maxPollen && !timeIsUp()) {
            HuskyLens.Block pollen = findClosestPollen();

            telemetry.addData("Pollen collected", pollenCollected + " / " + maxPollen);
            telemetry.addData("Time left", "%.1f s", matchTimer.getRemaining());

            if (pollen == null) {
                searchForPollen();
                telemetry.addData("Status", "Searching...");
            } else if (pollen.width >= CLOSE_WIDTH) {
                telemetry.addData("Status", "Close! Intaking");
                telemetry.update();
                grabPollen();
            } else {
                driveToward(pollen.x);
                telemetry.addData("Status", "Driving to pollen");
                telemetry.addData("x", pollen.x);
                telemetry.addData("width", pollen.width);
            }
            telemetry.update();
        }

        stopAll();
        telemetry.addData("Done", "Collected " + pollenCollected + " pollen");
        if (timeIsUp()) telemetry.addData("Reason", "Ran out of time");
        telemetry.update();
    }

    /** True when the match timer is on and almost out of time. */
    private boolean timeIsUp() {
        return useTimer && matchTimer.isLessThan(STOP_TIME);
    }

    /** Returns the biggest (closest) block with the pollen ID, or null if none. */
    private HuskyLens.Block findClosestPollen() {
        HuskyLens.Block[] blocks = huskyLens.blocks();
        HuskyLens.Block best = null;
        for (HuskyLens.Block b : blocks) {
            if (b.id == POLLEN_ID && (best == null || b.width > best.width)) {
                best = b;
            }
        }
        return best;
    }

    private void searchForPollen() {
        intake.setPower(0);
        setDrive(-SEARCH_SPEED, SEARCH_SPEED);  // spin left
        sleep(60);
        setDrive(0, 0);                         // pause so the camera image is clear
        sleep(40);
    }

    private void driveToward(int x) {
        int error = x - SCREEN_CENTER;          // negative = left, positive = right
        if (Math.abs(error) < CENTER_ZONE) error = 0;

        double turn  = error * TURN_GAIN;
        double left  = Range.clip(BASE_SPEED + turn, -1, 1);
        double right = Range.clip(BASE_SPEED - turn, -1, 1);
        setDrive(left, right);
    }

    private void grabPollen() {
        intake.setPower(INTAKE_POWER);
        setDrive(0.25, 0.25);                   // creep forward into it

        ElapsedTime timer = new ElapsedTime();
        while (opModeIsActive() && timer.seconds() < INTAKE_TIME && !timeIsUp()) {
            idle();
        }

        setDrive(0, 0);
        sleep(300);                             // let intake finish pulling it in
        intake.setPower(0);
        pollenCollected++;
        sleep(300);
    }

    private void setDrive(double left, double right) {
        leftDrive.setPower(left);
        rightDrive.setPower(right);
    }

    private void stopAll() {
        setDrive(0, 0);
        intake.setPower(0);
    }
}