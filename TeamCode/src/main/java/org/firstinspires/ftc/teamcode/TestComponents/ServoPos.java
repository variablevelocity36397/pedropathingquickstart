package org.firstinspires.ftc.teamcode.TestComponents;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "ServoPos", group = "Test")
public class ServoPos extends LinearOpMode {

    private Servo gate;
    double newPos = 0.5;

    @Override
    public void runOpMode() {

        // Map the servo from the hardware configuration
        gate = hardwareMap.get(Servo.class, "gate");

        telemetry.addLine("--------  GAMEPAD1 CONTROLS --------");
        telemetry.addLine("Press Dpad Up to increase servo position by 0.05");
        telemetry.addLine("Press Dpad Down to decrease servo position by 0.05");
        telemetry.addLine();
        telemetry.addLine("Initialized. Press PLAY to start.");
        telemetry.update();

        gate.setPosition(newPos);

        waitForStart();

        while (opModeIsActive()) {

            gate.setPosition(newPos);
            if (gamepad1.dpadUpWasPressed()) {
                newPos = newPos + 0.05;
            }
            if (gamepad1.dpadDownWasPressed()) {
                newPos = newPos - 0.05;
            }

            // Read current position every loop
            double servopos = gate.getPosition();

            // Push to Driver Hub
            telemetry.addLine("--------  GAMEPAD1 CONTROLS --------");
            telemetry.addLine("Dpad Up: +0.05  |  Dpad Down: -0.05");
            telemetry.addLine();
            telemetry.addData("Servo Name", "gate");
            telemetry.addData("Servo Position", "%.3f", servopos);
            telemetry.addData("Loop Time (ms)", "%.1f", getRuntime() * 1000 % 1000);
            telemetry.addData("Servo Position: ", newPos);
            telemetry.update();
        }
    }
}