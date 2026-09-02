package org.firstinspires.ftc.teamcode.Classes;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.List;

// Turret must be mechanically centered (0 deg) when the robot initializes -- there is no
// absolute encoder to home against. The Limelight's pipeline 0 must be configured via its
// web UI for the tag36h11 AprilTag family before this class will detect anything.
public class Turret {
    public enum TargetAlliance {
        BLUE(20),
        RED(24);

        public final int fiducialId;

        TargetAlliance(int fiducialId) {
            this.fiducialId = fiducialId;
        }
    }

    private final DcMotorEx turretMotor;
    private final Limelight3A limelight;

    private static final int APRILTAG_PIPELINE = 0;

    // REV Through Bore Encoder = 8192 ticks/rev, assumed coupled 1:1 to the turret shaft.
    // If it's geared (e.g. a pinion on the encoder driving a larger turret ring gear),
    // multiply by that gear ratio (encoder shaft revs per turret rev). Verify with TurretTuner.
    private static final double TICKS_PER_DEGREE = (8192.0 * 8) / 360.0;

    // Soft limits: wiring physically allows ~400 deg of total travel before over-winding.
    // Measured real hard stops are around +-210 deg; +-195 deg leaves ~15 deg of margin for
    // motor coast/overshoot on each side. Sweep (390 deg) is slightly over a full circle, which
    // is fine -- see chooseReachableTargetAngle().
    private static final double MIN_ANGLE_DEG = -195;
    private static final double MAX_ANGLE_DEG = 195;

    private static final double TX_DEADBAND_DEGREES = 1.75;
    private static final double MAX_TURRET_POWER = 0.6;

    // proportional-only visual servo on Limelight tx -- kI/kD left for future tuning
    private static final double kP = 0.02;
    private static final double HOLD_kP = 0.01;

    // Once a wrapped (long-way-around) route is committed to because the direct route would
    // exceed a soft limit, the direct route must be back in bounds by this much margin (not
    // just barely) before the wrap is abandoned -- prevents rapid direction reversals when tx
    // noise makes the raw target hover right at MIN/MAX_ANGLE_DEG.
    private static final double UNWRAP_HYSTERESIS_DEG = 10;

    private TargetAlliance targetAlliance = TargetAlliance.BLUE;
    private double lastKnownAngleDeg = 0;
    private boolean hasTarget = false;
    private boolean isLocked = false;

    // Unwrap state: once a wrapped route is chosen, stay committed to it -- both to avoid
    // chatter right at the boundary, and because the tag will be out of the camera's FOV for
    // most of the ~300 deg transit, so there's no fresh tx to react to until it reappears near
    // the destination.
    private boolean isUnwrapping = false;
    private double unwrapTargetAngleDeg = 0;

    public Turret(HardwareMap hardwareMap) {
        turretMotor = hardwareMap.get(DcMotorEx.class, "turretmtr");
        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        turretMotor.setDirection(DcMotor.Direction.FORWARD);
        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        limelight.pipelineSwitch(APRILTAG_PIPELINE);
        limelight.start();
    }

    public void setTargetAlliance(TargetAlliance alliance) {
        targetAlliance = alliance;
    }

    public void update() {
        LLResult result = limelight.getLatestResult();
        LLResultTypes.FiducialResult target = findTarget(result);
        double currentAngle = getTurretAngleDegrees();

        if (target != null) {
            double tx = target.getTargetXDegrees();
            double rawTarget = currentAngle + tx;
            double chosenTarget = chooseReachableTargetAngle(rawTarget);

            isLocked = !isUnwrapping && Math.abs(tx) < TX_DEADBAND_DEGREES;

            double power = isLocked ? 0 : clamp(kP * (chosenTarget - currentAngle), -MAX_TURRET_POWER, MAX_TURRET_POWER);
            power = clampAtSoftLimits(power);

            turretMotor.setPower(power);
            lastKnownAngleDeg = currentAngle;
            hasTarget = true;
        } else {
            double seekTarget = isUnwrapping ? unwrapTargetAngleDeg : lastKnownAngleDeg;
            double seekKp = isUnwrapping ? kP : HOLD_kP;

            double power = clamp(seekKp * (seekTarget - currentAngle), -MAX_TURRET_POWER, MAX_TURRET_POWER);
            turretMotor.setPower(clampAtSoftLimits(power));
            hasTarget = false;
            isLocked = false;
        }
    }

    // Returns the absolute turret-encoder angle to drive toward for a given raw (unwrapped)
    // target angle, substituting a wrapped (+-360 deg) alternative when the raw target would
    // exceed a soft limit and the wrapped one is reachable instead. Persists the choice via
    // isUnwrapping/unwrapTargetAngleDeg so the route doesn't flip back and forth right at the
    // boundary, and so update()'s no-target branch can keep steering toward the same absolute
    // angle while the tag is out of frame mid-transit.
    private double chooseReachableTargetAngle(double rawTarget) {
        if (isUnwrapping) {
            boolean backInBoundsWithMargin = rawTarget >= MIN_ANGLE_DEG + UNWRAP_HYSTERESIS_DEG
                    && rawTarget <= MAX_ANGLE_DEG - UNWRAP_HYSTERESIS_DEG;
            if (backInBoundsWithMargin) {
                isUnwrapping = false;
                return rawTarget;
            }
            if (isInBounds(rawTarget)) {
                // in bounds but not yet by enough margin -- stay committed to the previous target
                return unwrapTargetAngleDeg;
            }
            double rewrapped = rawTarget > MAX_ANGLE_DEG ? rawTarget - 360.0 : rawTarget + 360.0;
            if (isInBounds(rewrapped)) {
                unwrapTargetAngleDeg = rewrapped; // refresh for accuracy while still visible
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

        // Combined sweep is too small to reach this bearing from either side -- clamp so the
        // motor eases toward the limit instead of being commanded toward an unreachable target.
        return clamp(rawTarget, MIN_ANGLE_DEG, MAX_ANGLE_DEG);
    }

    private boolean isInBounds(double angle) {
        return angle >= MIN_ANGLE_DEG && angle <= MAX_ANGLE_DEG;
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

    private double clampAtSoftLimits(double power) {
        double angle = getTurretAngleDegrees();
        if (angle >= MAX_ANGLE_DEG && power > 0) {
            return 0;
        }
        if (angle <= MIN_ANGLE_DEG && power < 0) {
            return 0;
        }
        return power;
    }

    private static double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }

    public double getTurretAngleDegrees() {
        return turretMotor.getCurrentPosition() / TICKS_PER_DEGREE;
    }

    public boolean hasTarget() {
        return hasTarget;
    }

    public boolean isLocked() {
        return isLocked;
    }

    public boolean isUnwrapping() {
        return isUnwrapping;
    }

    public void stop() {
        turretMotor.setPower(0);
    }
}
