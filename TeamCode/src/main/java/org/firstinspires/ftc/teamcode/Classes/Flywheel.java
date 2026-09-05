package org.firstinspires.ftc.teamcode.Classes;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.DcMotor;

public class Flywheel {
    private final DcMotorEx leftMotor;
    private final DcMotorEx rightMotor;

    private static final double TICKS_PER_REV = 28.0;

    private static final PIDFCoefficients LEFT_PIDF = new PIDFCoefficients(0.0, 0.0, 0.0, 0.0);
    private static final PIDFCoefficients RIGHT_PIDF = new PIDFCoefficients(0.0, 0.0, 0.0, 0.0);

    public static final double CLOSE_TARGET_VELOCITY = 1000;
    public static final double FAR_TARGET_VELOCITY = 1600;

    private double currentTargetVelocity = FAR_TARGET_VELOCITY;
    private boolean isSpinning = false;

    public Flywheel(HardwareMap hardwareMap) {
        leftMotor = hardwareMap.get(DcMotorEx.class, "leftshooter");
        rightMotor = hardwareMap.get(DcMotorEx.class, "rightshooter");

        leftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        rightMotor.setDirection(DcMotorSimple.Direction.REVERSE); // same direction -- shared shaft

        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, LEFT_PIDF);
        rightMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, RIGHT_PIDF);
    }

    public static double rpmToTicksPerSec(double rpm) {
        return (rpm / 60.0) * TICKS_PER_REV;
    }

    public void setCloseZone() {
        currentTargetVelocity = CLOSE_TARGET_VELOCITY;
        if (isSpinning) {
            leftMotor.setVelocity(currentTargetVelocity);
            rightMotor.setVelocity(currentTargetVelocity);
        }
    }

    public void setFarZone() {
        currentTargetVelocity = FAR_TARGET_VELOCITY;
        if (isSpinning) {
            leftMotor.setVelocity(currentTargetVelocity);
            rightMotor.setVelocity(currentTargetVelocity);
        }
    }

    public void spinUp() {
        leftMotor.setVelocity(currentTargetVelocity);
        rightMotor.setVelocity(currentTargetVelocity);
        isSpinning = true;
    }

    // gentle coast-down via FLOAT, not active PIDF braking -- see stop-behavior note below
    public void stop() {
        leftMotor.setPower(0);
        rightMotor.setPower(0);
        isSpinning = false;
    }

    public double getLeftVelocity() { return leftMotor.getVelocity(); }
    public double getRightVelocity() { return rightMotor.getVelocity(); }

    public boolean isAtSpeed() {
        return Math.abs(leftMotor.getVelocity() - currentTargetVelocity) < 50
                && Math.abs(rightMotor.getVelocity() - currentTargetVelocity) < 50;
    }
}