package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by gsbol on 11/20/17.
 */

@TeleOp(name="MecanumWheel", group="Wang")
public class MecanumWheel extends LinearOpMode {


    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftfront = null;
    private DcMotor rightfront = null;
    private DcMotor leftrear = null;
    private DcMotor rightrear = null;
    private DcMotor slide = null;
    private DcMotor belt = null;
    private Servo servo_1 = null;
    private Servo servo_2 = null;
    private Servo servo_3 = null;
    private Servo servo_4 = null;
    private Servo servo_5 = null;
    private Servo servo_6 = null;


    private static final double Up_SERVO = 0.3;
    private static final double Down_SERVO = 1;
    private static final double WRIST_START = 1;
    private static final double GRABBER_START = 1;
    private static final double    SERVO_SPEED      = 0.01 ;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        leftfront = hardwareMap.get(DcMotor.class, "motor_1");
        rightfront = hardwareMap.get(DcMotor.class, "motor_2");
        leftrear = hardwareMap.get(DcMotor.class, "motor_3");
        rightrear= hardwareMap.get(DcMotor.class, "motor_4");
        slide = hardwareMap.get(DcMotor.class,"motor_5");
        belt = hardwareMap.get(DcMotor.class, "motor_6");
        servo_1 = hardwareMap.get(Servo.class, "servo_1");
        servo_2 = hardwareMap.get(Servo.class, "servo_2");
        servo_3 = hardwareMap.get(Servo.class, "servo_3");
        servo_4 = hardwareMap.get(Servo.class, "servo_4");
        servo_5 = hardwareMap.get(Servo.class, "servo_5");
        servo_6 = hardwareMap.get(Servo.class, "servo_6");


        leftfront.setDirection(DcMotor.Direction.REVERSE);
        leftrear.setDirection(DcMotor.Direction.REVERSE);
        rightfront.setDirection(DcMotor.Direction.FORWARD);
        rightrear.setDirection(DcMotor.Direction.FORWARD);
        slide.setDirection(DcMotor.Direction.FORWARD);
        belt.setDirection(DcMotor.Direction.FORWARD);

        servo_2.setPosition(WRIST_START);
        servo_3.setPosition(GRABBER_START);
        servo_5.setPosition(WRIST_START);
        servo_6.setPosition(GRABBER_START);


        waitForStart();
        runtime.reset();


        while (opModeIsActive()) {


            if (gamepad1.y) {
                servo_1.setPosition(Up_SERVO);
            }
            else if (gamepad1.a) {
                servo_1.setPosition(Down_SERVO);
            }

            if (gamepad2.y) {
                servo_2.setPosition(servo_2.getPosition() + SERVO_SPEED);

            }

            else if (gamepad2.a) {
                servo_2.setPosition(servo_2.getPosition() - SERVO_SPEED);
            }

            if (gamepad2.x) {
                servo_3.setPosition(servo_3.getPosition() + SERVO_SPEED);;
            }
            else if (gamepad2.b) {
                servo_3.setPosition(servo_3.getPosition() - SERVO_SPEED);
            }

            if (gamepad1.y) {
                servo_1.setPosition(Up_SERVO);
            }
            else if (gamepad1.a) {
                servo_1.setPosition(Down_SERVO);
            }


            if (gamepad2.left_bumper) {
                servo_5.setPosition(servo_5.getPosition() + SERVO_SPEED);

            }

            else if (gamepad2.right_bumper) {
                servo_5.setPosition(servo_5.getPosition() - SERVO_SPEED);
            }

            if (gamepad2.left_bumper) {
                servo_6.setPosition(servo_6.getPosition() - SERVO_SPEED);;
            }
            else if (gamepad2.right_bumper) {
                servo_6.setPosition(servo_6.getPosition() + SERVO_SPEED);
            }




            double vertical;

            double horizontal;

            double turn;

            vertical = gamepad1.left_stick_y;
            horizontal = gamepad1.left_stick_x;
            turn = gamepad1.right_stick_x;

            leftfront.setPower(-vertical);
            leftrear.setPower(-vertical);
            rightfront.setPower(-vertical);
            rightrear.setPower(-vertical);

            leftfront.setPower(horizontal);
            leftrear.setPower(-horizontal);
            rightfront.setPower(-horizontal);
            rightrear.setPower(horizontal);

            leftfront.setPower(-turn);
            leftrear.setPower(-turn);
            rightfront.setPower(turn);
            rightrear.setPower(turn);

            double up;

            up = gamepad2.left_stick_y;

            slide.setPower(up);

            double raise;
            raise = gamepad2.right_stick_y;

            belt.setPower(raise);

            telemetry.addData("Status", "Run Time: " + runtime.toString());
            //  telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
            telemetry.update();
        }

    }
}

