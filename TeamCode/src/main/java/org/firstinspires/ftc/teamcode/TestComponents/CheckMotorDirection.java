package org.firstinspires.ftc.teamcode.TestComponents;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "CheckMotorDirection", group = "Test")
public class CheckMotorDirection extends LinearOpMode {

    private DcMotor shooter1;
    private DcMotor shooter2;

    // Slow test power -- adjust if too fast/slow to visually judge
    private static final double TEST_POWER = 0.15;

    @Override
    public void runOpMode() {

        shooter1 = hardwareMap.get(DcMotor.class, "shooter1");
        shooter2 = hardwareMap.get(DcMotor.class, "shooter2");

        // Both motors fixed to FORWARD
        shooter1.setDirection(DcMotorSimple.Direction.FORWARD);
        shooter2.setDirection(DcMotorSimple.Direction.FORWARD);

        shooter1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooter2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addLine("After clicking play, hold A to check shooter1, hold B to check shooter2");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            if (gamepad1.a) {
                shooter1.setPower(TEST_POWER);
            } else {
                shooter1.setPower(0);
            }

            if (gamepad1.b) {
                shooter2.setPower(TEST_POWER);
            } else {
                shooter2.setPower(0);
            }

            // Telemetry
            telemetry.addLine("When holding A, if the motor is spinning the wrong direction, REVERSE motor direction");
            telemetry.addLine("Hold onto this piece of information");
            telemetry.addLine();

            telemetry.addLine("When holding B, if the motor is spinning the wrong direction, REVERSE motor direction");
            telemetry.addLine("Hold onto this piece of information");
            telemetry.addLine();
            telemetry.addLine();
            telemetry.addLine("Once you know which motors needs to be reversed, move onto the teststuff.java file");

            telemetry.update();
        }
    }
}