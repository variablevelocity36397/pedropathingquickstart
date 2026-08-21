package org.firstinspires.ftc.teamcode.TestComponents;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "teststuff", group = "Linear OpMode")

public class teststuff extends LinearOpMode {

    private DcMotor intakemotor;   // intake motor
    private DcMotor frwheel; // front right wheel
    private DcMotor flwheel; // front left wheel
    private DcMotor brwheel; // back right wheel
    private DcMotor blwheel; // back left wheel
    private DcMotor shooter1; // one of the 2 shooter motors
    private DcMotor shooter2; // the second of the 2 shooter motors
    //private DcMotor turretmtr; // the motor that turns the turntable/turret
    private CRServo intakeservo; // the middle intake powered via servo
    private Servo servokicker; // the servo kicker

    @Override
    public void runOpMode() {

        intakemotor = hardwareMap.get(DcMotor.class, "intakemotor");
        intakeservo = hardwareMap.get(CRServo.class, "intakeservo");
        frwheel = hardwareMap.get(DcMotor.class, "frwheel");
        flwheel = hardwareMap.get(DcMotor.class, "flwheel");
        flwheel.setDirection(DcMotorSimple.Direction.REVERSE);
        brwheel = hardwareMap.get(DcMotor.class, "brwheel");
        blwheel = hardwareMap.get(DcMotor.class, "blwheel");
        blwheel.setDirection(DcMotorSimple.Direction.REVERSE);
        shooter1 = hardwareMap.get(DcMotor.class, "shooter1");
        shooter2 = hardwareMap.get(DcMotor.class, "shooter2");
        //turretmtr = hardwareMap.get(DcMotor.class, "turretmtr");
        servokicker = hardwareMap.get(Servo.class, "servokicker");
        shooter1.setDirection(DcMotorSimple.Direction.FORWARD);
        shooter2.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeservo.setDirection(DcMotorSimple.Direction.REVERSE);


        shooter2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT); // Dont touch
        shooter1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT); // Dont touch

        telemetry.addLine("--------  GAMEPAD CONTROLS --------");
        telemetry.addLine("Press A to toggle intake");
        telemetry.addLine("Press B to toggle the servo kicker");
        telemetry.addLine("Press X to toggle both flywheel motors");
        telemetry.addLine("Left joystick to move, Right joystick to turn");
        telemetry.addLine("Hold the Right Trigger to drive at 20% speed");


        telemetry.update();
        waitForStart();

        boolean flag = true;
        boolean flag2 = true;
        boolean flag3 = true;

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

            if (gamepad1.aWasPressed()) {  // click the button a to toggle the intake
                if (flag) {
                    intakemotor.setPower(1);
                    intakeservo.setPower(1);
                } else {
                    intakeservo.setPower(0);
                    intakemotor.setPower(0);
                }
                flag = !flag;
            }
            if (gamepad1.bWasPressed()) { // click the button b to toggle the servo kicker
                if (flag2) {
                    servokicker.setPosition(0.05);
                }
                else {
                    servokicker.setPosition(0.456);
                }
                flag2 = !flag2;
            }
            if (gamepad1.xWasPressed()) { // click the button x to toggle the shooter
                if (flag3) {
                    shooter1.setPower(0.5);
                    shooter2.setPower(0.5);
                }
                else {
                    shooter2.setPower(0);
                    shooter1.setPower(0);
                }
                flag3 = !flag3;
            }
        }
    }
}
