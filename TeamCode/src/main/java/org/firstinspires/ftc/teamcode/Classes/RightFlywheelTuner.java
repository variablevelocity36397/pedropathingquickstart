package org.firstinspires.ftc.teamcode.Classes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

@TeleOp
public class RightFlywheelTuner extends OpMode {

    public DcMotorEx rightshooter;
    double highVelocity = 1600;
    double lowVelocity = 1000;
    double curTargetVelocity =  highVelocity;
    double P = 0;
    double F = 0;
    double [] stepSizes = {10.0, 1.0, 0.1, 0.001, 0.0001};
    int stepIndex = 1;


    @Override
    public void init() {
        rightshooter = hardwareMap.get(DcMotorEx.class, "rightshooter");
        rightshooter.setDirection(DcMotorSimple.Direction.FORWARD);
        rightshooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P, 0, 0, F);
        rightshooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        telemetry.addLine("Init Complete");
    }

    @Override
    public void loop() {

        if (gamepad1.yWasPressed()) {
            if (curTargetVelocity == highVelocity) {
                curTargetVelocity = lowVelocity;
            } else {
                curTargetVelocity = highVelocity;
            }
        }

        if (gamepad1.bWasPressed()) {
            stepIndex = (stepIndex + 1) % stepSizes.length;
        }

        if (gamepad1.dpadLeftWasPressed()) {
            F -= stepSizes[stepIndex];
        }
        if (gamepad1.dpadRightWasPressed()) {
            F += stepSizes[stepIndex];
        }

        if (gamepad1.dpadDownWasPressed()) {
            P += stepSizes[stepIndex];
        }
        if (gamepad1.dpadUpWasPressed()) {
            P-= stepSizes[stepIndex];
        }

        // set new PIDF coefficients
        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P, 0, 0, F);
        rightshooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);

        // set velocity
        rightshooter.setVelocity(curTargetVelocity);

        double curVelocity = rightshooter.getVelocity();
        double error = curTargetVelocity - curVelocity;

        // telemetry
        telemetry.addData("Taget velocity", curTargetVelocity);
        telemetry.addData("Cur velocity", "%.2f", curVelocity);
        telemetry.addData("error", "%.2f", error);
        telemetry.addLine("--------------------------------");
        telemetry.addData("Tuning P", "%.4f (D-pad U/D)", P);
        telemetry.addData("Tuning F", "%.4f (D-pad L/R)", F);
        telemetry.addData("Step Size", "%.4f (B Button)", stepSizes[stepIndex]);
        telemetry.update();
    }
}
