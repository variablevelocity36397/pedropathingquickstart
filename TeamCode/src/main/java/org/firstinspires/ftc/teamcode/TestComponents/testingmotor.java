package org.firstinspires.ftc.teamcode.TestComponents;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "testingmotor", group = "Linear OpMode")

public class testingmotor extends LinearOpMode {

    private DcMotor intakemotor;   // intake motor
    private DcMotor frwheel; // front right wheel
    private DcMotor flwheel; // front left wheel
    private DcMotor brwheel; // back right wheel
    private DcMotor blwheel; // back left wheel
    //private DcMotor shooter1; // one of the 2 shooter motors
    //private DcMotor shooter2; // the second of the 2 shooter motors
    //private DcMotor turretmtr; // the motor that turns the turntable/turret
    private CRServo intakeservo; // the middle intake powered via servo
    private Servo servokicker; //servo kicker

    @Override
    public void runOpMode() {

        intakemotor = hardwareMap.get(DcMotor.class, "intakemotor");
        intakeservo = hardwareMap.get(CRServo.class, "intakeservo");
        frwheel = hardwareMap.get(DcMotor.class, "frwheel");
        flwheel = hardwareMap.get(DcMotor.class, "flwheel");
        brwheel = hardwareMap.get(DcMotor.class, "brwheel");
        blwheel = hardwareMap.get(DcMotor.class, "blwheel");
        //shooter1 = hardwareMap.get(DcMotor.class, "shooter1");
        //shooter2 = hardwareMap.get(DcMotor.class, "shooter2");
        //turretmtr = hardwareMap.get(DcMotor.class, "turretmtr");
        servokicker = hardwareMap.get(Servo.class, "servokicker");

        servokicker.setPosition(0.456);

        waitForStart();

        boolean flag = true;
        boolean flag2 = true;

        while (opModeIsActive()) {

            double y = gamepad1.left_stick_y ;       // left joystick control forward/back movement
            double x = -gamepad1.left_stick_x ;     // right joystick control right/left turning
            double rx = -gamepad1.right_stick_x * 0.8;
            double speed = 1;
            if (gamepad1.right_trigger > 0.1) {
                speed = 0.2;
            }

            double leftFrontPower = (y + x + rx) ;
            double leftBackPower = (y - x + rx) ;
            double rightFrontPower = (y - x - rx) ;
            double rightBackPower = (y + x - rx) ;

            flwheel.setPower(leftFrontPower * speed);
            blwheel.setPower(leftBackPower * speed);
            frwheel.setPower(rightFrontPower * speed);
            brwheel.setPower(rightBackPower * speed);


            if (gamepad1.aWasPressed()) {
                if (flag) {
                    intakemotor.setPower(1);
                    intakeservo.setPower(1);
                } else {
                    intakeservo.setPower(0);
                    intakemotor.setPower(0);
                }
                flag = !flag;
            } // intake toggle
            if (gamepad1.bWasPressed()) {
                if (flag2) {
                    servokicker.setPosition(0.05);
                } else {
                    servokicker.setPosition(0.456);
                }
                flag2 = !flag2;
            }



        }
    }
}
