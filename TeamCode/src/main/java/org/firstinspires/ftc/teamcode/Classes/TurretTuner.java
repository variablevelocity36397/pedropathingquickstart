package org.firstinspires.ftc.teamcode.Classes;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import java.util.List;

@TeleOp(name = "TurretTuner", group = "Tuning")
public class TurretTuner extends OpMode {

    private DcMotorEx turretMotor;
    private Limelight3A limelight;

    private static final int APRILTAG_PIPELINE = 0;
    private static final double MANUAL_JOG_POWER = 0.25;

    // live-tunable values -- mirror the constants in Turret.java, copy the final numbers back
    // into that file once they're dialed in
    double kP = 0.02;
    double holdKP = 0.01;
    double ticksPerDegree = (8192.0 * 8) / 360.0;
    double maxPower = 0.6;
    double txDeadband = 1.75;
    double unwrapHysteresis = 10;

    // soft limits -- NOT live-tunable here, must be kept in sync with Turret.java by hand
    // (same MIN_ANGLE_DEG/MAX_ANGLE_DEG convention as the real class)
    private static final double MIN_ANGLE_DEG = -195;
    private static final double MAX_ANGLE_DEG = 195;

    double[] stepSizes = {10.0, 1.0, 0.1, 0.01, 0.001};
    int stepIndex = 2;

    String[] paramNames = {"kP", "HOLD_kP", "ticksPerDegree", "maxPower", "txDeadband", "unwrapHysteresis"};
    int paramIndex = 0;

    Turret.TargetAlliance targetAlliance = Turret.TargetAlliance.BLUE;
    boolean trackingEnabled = false;

    double lastKnownAngleDeg = 0;
    boolean hasTarget = false;
    boolean locked = false;

    boolean isUnwrapping = false;
    double unwrapTargetAngleDeg = 0;

    @Override
    public void init() {
        turretMotor = hardwareMap.get(DcMotorEx.class, "turretmtr");
        turretMotor.setDirection(DcMotor.Direction.FORWARD);
        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(APRILTAG_PIPELINE);
        limelight.start();

        telemetry.addLine("Turret centered at home (facing forward) before pressing start?");
        telemetry.addLine("Init complete.");
        telemetry.update();
    }

    @Override
    public void loop() {
        handleControls();

        LLResult result = limelight.getLatestResult();
        LLResultTypes.FiducialResult target = findTarget(result);

        double currentAngle = turretMotor.getCurrentPosition() / ticksPerDegree;
        double tx = 0;
        int seenId = -1;

        if (target != null) {
            tx = target.getTargetXDegrees();
            seenId = target.getFiducialId();
            hasTarget = true;
            lastKnownAngleDeg = currentAngle;
        } else {
            hasTarget = false;
            locked = false;
        }

        if (trackingEnabled) {
            double power;
            if (hasTarget) {
                double rawTarget = currentAngle + tx;
                double chosenTarget = chooseReachableTargetAngle(rawTarget, currentAngle);
                locked = !isUnwrapping && Math.abs(tx) < txDeadband;
                power = locked ? 0 : clamp(kP * (chosenTarget - currentAngle), -maxPower, maxPower);
            } else {
                double seekTarget = isUnwrapping ? unwrapTargetAngleDeg : lastKnownAngleDeg;
                double seekKp = isUnwrapping ? kP : holdKP;
                power = clamp(seekKp * (seekTarget - currentAngle), -maxPower, maxPower);
            }
            turretMotor.setPower(clampAtSoftLimits(power, currentAngle));
        }
        // manual jog (bumpers) only runs while tracking is OFF -- see handleControls()

        printTelemetry(currentAngle, tx, seenId);
    }

    private void handleControls() {
        if (gamepad1.xWasPressed()) {
            paramIndex = (paramIndex + 1) % paramNames.length;
        }
        if (gamepad1.bWasPressed()) {
            stepIndex = (stepIndex + 1) % stepSizes.length;
        }

        double delta = 0;
        if (gamepad1.dpadUpWasPressed()) delta = stepSizes[stepIndex];
        if (gamepad1.dpadDownWasPressed()) delta = -stepSizes[stepIndex];

        if (delta != 0) {
            switch (paramIndex) {
                case 0: kP += delta; break;
                case 1: holdKP += delta; break;
                case 2: ticksPerDegree += delta; break;
                case 3: maxPower += delta; break;
                case 4: txDeadband += delta; break;
                case 5: unwrapHysteresis += delta; break;
            }
        }

        if (gamepad1.aWasPressed()) {
            trackingEnabled = !trackingEnabled;
            turretMotor.setPower(0);
        }
        if (gamepad1.yWasPressed()) {
            targetAlliance = (targetAlliance == Turret.TargetAlliance.BLUE)
                    ? Turret.TargetAlliance.RED : Turret.TargetAlliance.BLUE;
        }

        if (!trackingEnabled) {
            if (gamepad1.left_bumper) {
                turretMotor.setPower(-MANUAL_JOG_POWER);
            } else if (gamepad1.right_bumper) {
                turretMotor.setPower(MANUAL_JOG_POWER);
            } else {
                turretMotor.setPower(0);
            }
        }
    }

    private LLResultTypes.FiducialResult findTarget(LLResult result) {
        if (result == null || !result.isValid()) {
            return null;
        }
        List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
        for (LLResultTypes.FiducialResult fr : fiducials) {
            if (fr.getFiducialId() == targetAlliance.fiducialId) {
                return fr;
            }
        }
        return null;
    }

    private static double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }

    // Mirrors Turret.chooseReachableTargetAngle() -- see that method for the full explanation.
    private double chooseReachableTargetAngle(double rawTarget, double currentAngle) {
        if (isUnwrapping) {
            boolean backInBoundsWithMargin = rawTarget >= MIN_ANGLE_DEG + unwrapHysteresis
                    && rawTarget <= MAX_ANGLE_DEG - unwrapHysteresis;
            if (backInBoundsWithMargin) {
                isUnwrapping = false;
                return rawTarget;
            }
            if (isInBounds(rawTarget)) {
                return unwrapTargetAngleDeg;
            }
            double rewrapped = rawTarget > MAX_ANGLE_DEG ? rawTarget - 360.0 : rawTarget + 360.0;
            if (isInBounds(rewrapped)) {
                unwrapTargetAngleDeg = rewrapped;
            }
            return unwrapTargetAngleDeg;
        }

        if (isInBounds(rawTarget)) {
            return rawTarget;
        }

        double wrapped = rawTarget > MAX_ANGLE_DEG ? rawTarget - 360.0 : rawTarget + 360.0;
        if (isInBounds(wrapped)) {
            isUnwrapping = true;
            unwrapTargetAngleDeg = wrapped;
            return wrapped;
        }

        return clamp(rawTarget, MIN_ANGLE_DEG, MAX_ANGLE_DEG);
    }

    private boolean isInBounds(double angle) {
        return angle >= MIN_ANGLE_DEG && angle <= MAX_ANGLE_DEG;
    }

    // Mirrors Turret.clampAtSoftLimits() -- final backstop so the tuner can't be driven past
    // the same soft limits Turret.java enforces.
    private double clampAtSoftLimits(double power, double currentAngle) {
        if (currentAngle >= MAX_ANGLE_DEG && power > 0) {
            return 0;
        }
        if (currentAngle <= MIN_ANGLE_DEG && power < 0) {
            return 0;
        }
        return power;
    }

    private void printTelemetry(double currentAngle, double tx, int seenId) {
        telemetry.addLine("=== TURRET TUNER ===");
        telemetry.addData("Tracking", trackingEnabled ? "ON (A to stop)" : "OFF (A to start, bumpers to jog)");
        telemetry.addData("Alliance target", "%s (id %d) -- Y to switch", targetAlliance, targetAlliance.fiducialId);
        telemetry.addLine("--------------------------------");
        telemetry.addData("Selected param", "%s -- X to cycle", paramNames[paramIndex]);
        telemetry.addData("Step size", "%.4f -- B to cycle", stepSizes[stepIndex]);
        telemetry.addData("  kP (D-pad U/D)", "%.4f", kP);
        telemetry.addData("  HOLD_kP (D-pad U/D)", "%.4f", holdKP);
        telemetry.addData("  ticksPerDegree (D-pad U/D)", "%.4f", ticksPerDegree);
        telemetry.addData("  maxPower (D-pad U/D)", "%.4f", maxPower);
        telemetry.addData("  txDeadband deg (D-pad U/D)", "%.4f", txDeadband);
        telemetry.addData("  unwrapHysteresis deg (D-pad U/D)", "%.4f", unwrapHysteresis);
        telemetry.addLine("--------------------------------");
        telemetry.addData("Turret angle (deg)", "%.2f", currentAngle);
        telemetry.addData("Raw encoder ticks", turretMotor.getCurrentPosition());
        telemetry.addData("Fiducial seen", hasTarget ? seenId : "none");
        telemetry.addData("tx (deg)", "%.2f", tx);
        telemetry.addData("Locked (within deadband)", locked);
        telemetry.addData("Unwrapping (going the long way)", isUnwrapping ? String.format("YES -> %.2f deg", unwrapTargetAngleDeg) : "no");
        telemetry.addLine("--------------------------------");
        telemetry.addLine("HOW TO TUNE (select param w/ X, step w/ B, adjust w/ D-pad U/D):");
        telemetry.addLine("1) ticksPerDegree FIRST: tracking OFF, jog w/ bumpers to a known");
        telemetry.addLine("   angle (protractor/framing mark, e.g. 90deg from home), adjust");
        telemetry.addLine("   until 'Turret angle' matches. Check at 2 far-apart angles.");
        telemetry.addLine("2) kP: A to enable tracking, show it a tag, raise kP until it");
        telemetry.addLine("   snaps to center fast w/o overshoot/oscillation, back off ~30%.");
        telemetry.addLine("3) txDeadband: with tracking on + tag steady, lower until you can");
        telemetry.addLine("   just barely see/hear the motor buzz at center, then raise 1 step.");
        telemetry.addLine("4) HOLD_kP: keep tracking on, cover the Limelight, nudge turret by");
        telemetry.addLine("   hand -- raise until it resists firmly w/o buzzing when left alone.");
        telemetry.addLine("5) maxPower: lowest value that still tracks fast enough for you.");
        telemetry.addLine("6) SAFETY CHECK: tracking OFF, jog to each real hard stop, confirm");
        telemetry.addLine("   'Turret angle' reads beyond +-195 there (i.e. the +-195 soft");
        telemetry.addLine("   limit trips BEFORE the real stop). If not, STOP and fix");
        telemetry.addLine("   ticksPerDegree/homing before ever enabling tracking.");
        telemetry.addLine("7) UNWRAP TEST (do this LAST, robot secured/propped up, low");
        telemetry.addLine("   maxPower first): tracking ON, slowly walk the tag toward one");
        telemetry.addLine("   soft limit until 'Unwrapping' shows YES -- confirm it spins the");
        telemetry.addLine("   long way around smoothly (no stutter at the boundary) and");
        telemetry.addLine("   re-locks near the target angle. unwrapHysteresis: raise if it");
        telemetry.addLine("   flickers in/out of unwrap right at the edge.");
        telemetry.addLine("Copy final numbers into Turret.java's constants when satisfied.");
        telemetry.update();
    }
}
