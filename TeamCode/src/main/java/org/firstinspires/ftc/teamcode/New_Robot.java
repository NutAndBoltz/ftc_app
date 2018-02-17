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

@TeleOp(name="New_Robot", group="Wang")

public class New_Robot extends LinearOpMode {


    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftfront = null;
    private DcMotor rightfront = null;
    private DcMotor leftrear = null;
    private DcMotor rightrear = null;
    private DcMotor slide = null;
    private DcMotor glyph = null;
    private Servo servo_1 = null;
    private Servo servo_2 = null;
    private Servo servo_3 = null;
    private Servo servo_5 = null;
    private Servo servo_6 = null;
    private Servo servo_4 = null;

    private static final double Up_SERVO = 0.35;
    private static final double Down_SERVO = 1;
    private static final double WRIST_START = 0.8;
    private static final double GRABBER_START = 0.8;
    private static final double SERVO_5_START = 0.9;
    private static final double SERVO_6_START = 0.1;
    private static final double SERVO_SPEED      = 0.01 ;
    private static final double SERVO_SPEED2      = 0.02 ;
    private double number = 1;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        leftfront = hardwareMap.get(DcMotor.class, "motor_1");
        rightfront = hardwareMap.get(DcMotor.class, "motor_2");
        leftrear = hardwareMap.get(DcMotor.class, "motor_3");
        rightrear= hardwareMap.get(DcMotor.class, "motor_4");
        slide = hardwareMap.get(DcMotor.class,"motor_5");
        glyph = hardwareMap.get(DcMotor.class,"motor_6");
        servo_1 = hardwareMap.get(Servo.class, "servo_1");
        servo_2 = hardwareMap.get(Servo.class, "servo_2");
        servo_3 = hardwareMap.get(Servo.class, "servo_3");
        servo_4 = hardwareMap.get(Servo.class, "servo_4");



        leftfront.setDirection(DcMotor.Direction.REVERSE);
        leftrear.setDirection(DcMotor.Direction.REVERSE);
        rightfront.setDirection(DcMotor.Direction.FORWARD);
        rightrear.setDirection(DcMotor.Direction.FORWARD);
        slide.setDirection(DcMotor.Direction.FORWARD);
        glyph.setDirection(DcMotor.Direction.REVERSE);

        servo_2.setPosition(WRIST_START);
        servo_3.setPosition(GRABBER_START);


        waitForStart();
        runtime.reset();


        while (opModeIsActive()) {



            if (gamepad2.y && servo_2.getPosition() <1) {
                servo_2.setPosition(servo_2.getPosition() + SERVO_SPEED);
            }



            else if (gamepad2.a && servo_2.getPosition() >0) {
                servo_2.setPosition(servo_2.getPosition() - SERVO_SPEED);
            }

            if (gamepad2.x && servo_3.getPosition() <1) {
                servo_3.setPosition(servo_3.getPosition() + SERVO_SPEED);
            }

            else if (gamepad2.b && servo_3.getPosition() >0) {
                servo_3.setPosition(servo_3.getPosition() - SERVO_SPEED);
            }


            if (gamepad2.right_bumper) {
                servo_4.setPosition(1);
            }

            else if (gamepad2.left_bumper) {
                servo_4.setPosition(0);
            }


            double vertical;

            double horizontal;

            double turn;



            vertical = gamepad1.left_stick_y;
            horizontal = gamepad1.left_stick_x;
            turn = gamepad1.right_stick_x;

            if (gamepad1.left_bumper && number>0.25) {
                number=number/2;
            }

            else if (gamepad1.right_bumper && number<1) {
                number=number*2;
            }


            leftfront.setPower(-vertical*number);
            leftrear.setPower(-vertical * number);
            rightfront.setPower(-vertical * number);
            rightrear.setPower(-vertical * number);

            leftfront.setPower(horizontal * number);
            leftrear.setPower(-horizontal * number);
            rightfront.setPower(-horizontal * number);
            rightrear.setPower(horizontal * number);

            leftfront.setPower(-turn * number);
            leftrear.setPower(-turn * number);
            rightfront.setPower(turn * number);
            rightrear.setPower(turn * number);

            double up;

            up = gamepad2.left_stick_y;

            slide.setPower(up);

            double system;

            system = gamepad2.left_stick_y;

            glyph.setPower(system);


            telemetry.addData("Status", "Run Time: " + runtime.toString());
            //  telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
            telemetry.update();
        }

    }
}

