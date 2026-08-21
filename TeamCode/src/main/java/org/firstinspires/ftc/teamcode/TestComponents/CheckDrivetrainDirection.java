package org.firstinspires.ftc.teamcode.TestComponents;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "CheckDrivetrainDirection")
public class CheckDrivetrainDirection extends LinearOpMode {

    private DcMotor frwheel;
    private DcMotor flwheel;
    private DcMotor brwheel;
    private DcMotor blwheel;

    // Power to apply while a button is held
    private static final double TEST_POWER = 0.4;

    @Override
    public void runOpMode() {

        // Map hardware names (must match your robot configuration on the Driver Hub)
        frwheel = hardwareMap.get(DcMotor.class, "frwheel");
        flwheel = hardwareMap.get(DcMotor.class, "flwheel");
        brwheel = hardwareMap.get(DcMotor.class, "brwheel");
        blwheel = hardwareMap.get(DcMotor.class, "blwheel");

        // Make sure motors stop when not pressed
        frwheel.setPower(0);
        flwheel.setPower(0);
        brwheel.setPower(0);
        blwheel.setPower(0);

        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            // Each button only runs its own motor, only while held
            frwheel.setPower(gamepad1.y ? TEST_POWER : 0);   // Y -> front right
            flwheel.setPower(gamepad1.x ? TEST_POWER : 0);   // X -> front left
            brwheel.setPower(gamepad1.b ? TEST_POWER : 0);   // B -> back right
            blwheel.setPower(gamepad1.a ? TEST_POWER : 0);   // A -> back left

            // Telemetry showing controls and live motor status
            telemetry.addLine("=== Gamepad1 Controls ===");
            telemetry.addLine("Y button -> frwheel (front right)");
            telemetry.addLine("X button -> flwheel (front left)");
            telemetry.addLine("B button -> brwheel (back right)");
            telemetry.addLine("A button -> blwheel (back left)");
            telemetry.addLine("Hold a button to spin that motor forward. Release to stop.");
            telemetry.addLine("If a wheel spins backward, reverse that motor in your code.");

            telemetry.update();
        }

        // Safety stop
        frwheel.setPower(0);
        flwheel.setPower(0);
        brwheel.setPower(0);
        blwheel.setPower(0);
    }
}