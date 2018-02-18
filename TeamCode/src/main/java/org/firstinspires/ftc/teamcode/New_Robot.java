package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
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
    private DcMotor glyphleft = null;
    private DcMotor glyphright = null;
    private DcMotor slide = null;

    private Servo colorArm1 = null;
    private Servo colorArm2 = null;
    private Servo glyphFlipper = null;
    private Servo glyphStopper = null;
    private Servo elbow = null;
    private Servo claw = null;

    private static final double Straight_colorArm1 = 0.35;
    private static final double Up_colorArm2 = .9;
    private static final double Down_colorArm2 = .35;
    private static final double Up_glyphFlipper = 1;
    private static final double Down_glyphFlipper = 0;
    private static final double Up_glyphStopper = 1;
    private static final double Down_glyphStoppper = 0;
    private static final double Up_elbow = 0;
    private static final double Down_elbow = 0.9;
    private static final double Open_claw = 1;
    private static final double Closed_claw = 0.35;

    private double number = 1;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        leftfront = hardwareMap.get(DcMotor.class, "motor_1");
        rightfront = hardwareMap.get(DcMotor.class, "motor_2");
        leftrear = hardwareMap.get(DcMotor.class, "motor_3");
        rightrear= hardwareMap.get(DcMotor.class, "motor_4");
        glyphleft = hardwareMap.get(DcMotor.class,"motor_5");
        glyphright = hardwareMap.get(DcMotor.class,"motor_6");
        slide = hardwareMap.get(DcMotor.class,"motor_7");

        colorArm1 = hardwareMap.get(Servo.class, "servo_1");
        colorArm2 = hardwareMap.get(Servo.class, "servo_2");
        glyphFlipper = hardwareMap.get(Servo.class, "servo_3");
        glyphStopper = hardwareMap.get(Servo.class, "servo_4");
        elbow = hardwareMap.get(Servo.class, "servo_5");
        claw = hardwareMap.get(Servo.class, "servo_6");


        leftfront.setDirection(DcMotor.Direction.REVERSE);
        leftrear.setDirection(DcMotor.Direction.REVERSE);
        rightfront.setDirection(DcMotor.Direction.FORWARD);
        rightrear.setDirection(DcMotor.Direction.FORWARD);
        glyphleft.setDirection(DcMotor.Direction.FORWARD);
        glyphright.setDirection(DcMotor.Direction.REVERSE);

        //initialize servo positions
        colorArm1.setPosition(Straight_colorArm1);
        colorArm2.setPosition(Up_colorArm2);
        glyphFlipper.setPosition(Down_glyphFlipper);
        glyphStopper.setPosition(Down_glyphStoppper);
        elbow.setPosition(Up_elbow);
        claw.setPosition(Closed_claw);


        waitForStart();
        runtime.reset();


        while (opModeIsActive()) {


            if (gamepad1.y) {
                colorArm2.setPosition(Up_colorArm2);
            }
            else if (gamepad1.a) {
                colorArm2.setPosition(Down_colorArm2);
            }

            if (gamepad2.y) {
                glyphFlipper.setPosition(Up_glyphFlipper);
            }
            else if (gamepad2.a) {
                glyphFlipper.setPosition(Down_glyphFlipper);
            }

            if (gamepad2.b) {
                glyphStopper.setPosition(Up_glyphStopper);
            }
            else if (gamepad2.x) {
                glyphStopper.setPosition(Down_glyphStoppper);
            }

            if (gamepad2.dpad_right) {
                elbow.setPosition(elbow.getPosition() + 0.2);
            }
            else if (gamepad2.dpad_left) {
                elbow.setPosition(elbow.getPosition() - 0.2);
            }

            if (gamepad2.dpad_up) {
                claw.setPosition(Open_claw);
            }
            else if (gamepad2.dpad_down) {
                claw.setPosition(Closed_claw);
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


            leftfront.setPower(vertical*number);
            leftrear.setPower(vertical * number);
            rightfront.setPower(vertical * number);
            rightrear.setPower(vertical * number);

            leftfront.setPower(-horizontal * number);
            leftrear.setPower(horizontal * number*0.8);
            rightfront.setPower(horizontal * number);
            rightrear.setPower(-horizontal * number*0.8);

            leftfront.setPower(turn * number);
            leftrear.setPower(turn * number);
            rightfront.setPower(-turn * number);
            rightrear.setPower(-turn * number);

            double up;
            up = gamepad2.left_stick_y;
            glyphleft.setPower(up);
            glyphright.setPower(up);

            double slidePower;
            slidePower = gamepad2.right_stick_y;
            slide.setPower(slidePower);


            telemetry.addData("Status", "Run Time: " + runtime.toString());
            //  telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
            telemetry.update();
        }

    }
}

