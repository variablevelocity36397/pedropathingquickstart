package org.firstinspires.ftc.teamcode.TestComponents;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "ServoPos", group = "Test")
public class ServoPos extends LinearOpMode {

    private Servo gate;

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

        waitForStart();

        while (opModeIsActive()) {

            // Optional: let dpad up/down nudge the servo so you can watch it move
            if (gamepad1.dpadUpWasPressed()) {
                double newPos = Math.min(1.0, gate.getPosition() + 0.05);
                gate.setPosition(newPos);
            }
            if (gamepad1.dpadDownWasPressed()) {
                double newPos = Math.max(0.0, gate.getPosition() - 0.05);
                gate.setPosition(newPos);
            }

            // Read current position every loop
            double servopos = gate.getPosition();

            // Push to Driver Hub
            telemetry.addLine("--------  GAMEPAD1 CONTROLS --------");
            telemetry.addLine("Dpad Up: +0.05  |  Dpad Down: -0.05");
            telemetry.addLine();
            telemetry.addData("Servo Name", "servokicker");
            telemetry.addData("Servo Position", "%.3f", servopos);
            telemetry.addData("Loop Time (ms)", "%.1f", getRuntime() * 1000 % 1000);
            telemetry.update();
        }
    }
}