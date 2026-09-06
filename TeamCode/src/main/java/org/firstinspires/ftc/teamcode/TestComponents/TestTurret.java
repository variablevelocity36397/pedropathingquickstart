package org.firstinspires.ftc.teamcode.TestComponents;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "TestTurret", group = "Testing")
public class TestTurret extends LinearOpMode {
    private DcMotor turretmtr;

    @Override
    public void runOpMode() {
        turretmtr = hardwareMap.get(DcMotor.class, "turretmtr");
        turretmtr.setDirection(DcMotorSimple.Direction.FORWARD);
        turretmtr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        double power = 0.65;

        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.left_trigger >= 0.1) {
                turretmtr.setPower(power);
            } else {
                turretmtr.setPower(0);
            }

            if (gamepad1.right_trigger >= 0.1) {
                turretmtr.setPower(-power);
            } else {
                turretmtr.setPower(0);
            }
        }
    }

}
