package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by gsbol on 11/20/17.
 */

@TeleOp(name="Awesome_Double_Relic ", group="Wang")
@Disabled
public class Awesome_Double_Relic extends LinearOpMode {

    static final double COUNTS_PER_MOTOR_REV = 2240;    // eg: TETRIX Motor Encoder
    // static final double     DRIVE_GEAR_REDUCTION    = 2.0 ;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    static final double COUNTS_PER_INCH = COUNTS_PER_MOTOR_REV / (WHEEL_DIAMETER_INCHES * 3.1415);

    static final double Counts_PER_DegreeTurn = 24;
    public static final double DRIVE_SPEED = 0.5;

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

    private static final double Up_SERVO = 0.65;
    private static final double Down_SERVO = 0;
    private static final double WRIST_START = 0.8;
    private static final double GRABBER_START = 0.8;
    private static final double SERVO_5_START = 0.8;
    private static final double SERVO_6_START = 0.1;
    private static final double SERVO_SPEED = 0.01;
    private static final double SERVO_SPEED2 = 0.02;
    private double number = 1;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        leftfront = hardwareMap.get(DcMotor.class, "motor_1");
        rightfront = hardwareMap.get(DcMotor.class, "motor_2");
        leftrear = hardwareMap.get(DcMotor.class, "motor_3");
        rightrear = hardwareMap.get(DcMotor.class, "motor_4");
        slide = hardwareMap.get(DcMotor.class, "motor_5");
        glyph = hardwareMap.get(DcMotor.class, "motor_6");
        servo_1 = hardwareMap.get(Servo.class, "servo_1");
        servo_2 = hardwareMap.get(Servo.class, "servo_2");
        servo_3 = hardwareMap.get(Servo.class, "servo_3");
        servo_5 = hardwareMap.get(Servo.class, "servo_5");
        servo_6 = hardwareMap.get(Servo.class, "servo_6");


        leftfront.setDirection(DcMotor.Direction.REVERSE);
        leftrear.setDirection(DcMotor.Direction.REVERSE);
        rightfront.setDirection(DcMotor.Direction.FORWARD);
        rightrear.setDirection(DcMotor.Direction.FORWARD);
        slide.setDirection(DcMotor.Direction.FORWARD);
        glyph.setDirection(DcMotor.Direction.REVERSE);

        servo_2.setPosition(WRIST_START);
        servo_3.setPosition(GRABBER_START);
        servo_5.setPosition(SERVO_5_START);
        servo_6.setPosition(SERVO_6_START);

        waitForStart();
        runtime.reset();


        while (opModeIsActive()) {


            if (gamepad1.y) {
                servo_1.setPosition(Up_SERVO);
            } else if (gamepad1.a) {
                servo_1.setPosition(Down_SERVO);
            }

            if (gamepad2.y && servo_2.getPosition() < 1) {
                servo_2.setPosition(servo_2.getPosition() + SERVO_SPEED);
            } else if (gamepad2.a && servo_2.getPosition() > 0) {
                servo_2.setPosition(servo_2.getPosition() - SERVO_SPEED);
            }

            if (gamepad2.x && servo_3.getPosition() < 1) {
                servo_3.setPosition(servo_3.getPosition() + SERVO_SPEED);
            } else if (gamepad2.b && servo_3.getPosition() > 0) {
                servo_3.setPosition(servo_3.getPosition() - SERVO_SPEED);
            }

            if (gamepad2.left_bumper && servo_5.getPosition() > 0 && servo_6.getPosition() < 1) {
                servo_5.setPosition(servo_5.getPosition() - SERVO_SPEED2);
                servo_6.setPosition(servo_6.getPosition() + SERVO_SPEED2);
            } else if (gamepad2.right_bumper && servo_5.getPosition() < 1 && servo_6.getPosition() > 0) {
                servo_5.setPosition(servo_5.getPosition() + SERVO_SPEED2);
                servo_6.setPosition(servo_6.getPosition() - SERVO_SPEED2);
            }


            double vertical;

            double horizontal;

            double turn;


            vertical = gamepad1.left_stick_y;
            horizontal = gamepad1.left_stick_x;
            turn = gamepad1.right_stick_x;

            if (gamepad1.left_bumper && number > 0.25) {
                number = number / 2;
            } else if (gamepad1.right_bumper && number < 1) {
                number = number * 2;
            }


            leftfront.setPower(-vertical * number);
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

            system = gamepad2.right_stick_y;

            glyph.setPower(system);

            if (gamepad1.x) {
                driveBackward(4);

                stopRobot();
                runtime.reset();
                while (opModeIsActive() && (runtime.seconds() < 0.3)) ;

                //wrist
                servo_3.setPosition(0);

                stopRobot();
                runtime.reset();
                while (opModeIsActive() && (runtime.seconds() < 0.3)) ;

                rotateRightDegrees(63);

                stopRobot();
                runtime.reset();
                while (opModeIsActive() && (runtime.seconds() < 0.3)) ;

                driveBackward(2);
                stopRobot();
                runtime.reset();
                while (opModeIsActive() && (runtime.seconds() < 0.3)) ;

                //wrist
                servo_3.setPosition(0.4);

                stopRobot();
                runtime.reset();
                while (opModeIsActive() && (runtime.seconds() < 0.3)) ;


                //grabber
                servo_2.setPosition(1);

                stopRobot();
                runtime.reset();
                while (opModeIsActive() && (runtime.seconds() < 0.3)) ;

                rotateLeftDegrees(4);
                stopRobot();
                runtime.reset();
                while (opModeIsActive() && (runtime.seconds() < 0.3)) ;


                driveForward(6);

                stopRobot();
                runtime.reset();
                while (opModeIsActive() && (runtime.seconds() < 0.3)) ;

                //grabber
                servo_2.setPosition(0);

                stopRobot();
                runtime.reset();
                while (opModeIsActive() && (runtime.seconds() < 0.3)) ;


                driveBackward(7);

                stopRobot();
                runtime.reset();
                while (opModeIsActive() && (runtime.seconds() < 0.3)) ;

                //wrist
                servo_3.setPosition(0);

                stopRobot();
                runtime.reset();
                while (opModeIsActive() && (runtime.seconds() < 0.3)) ;

                rotateLeftDegrees(45);

                stopRobot();
                runtime.reset();
                while (opModeIsActive() && (runtime.seconds() < 0.3)) ;

                driveForward(5);

                stopRobot();
                runtime.reset();
                while (opModeIsActive() && (runtime.seconds() < 1)) ;

                //wrist
                servo_3.setPosition(0.4);

                stopRobot();
                runtime.reset();
                while (opModeIsActive() && (runtime.seconds() < 1)) ;

                //grabber
                servo_2.setPosition(1);

                stopRobot();
                runtime.reset();
                while (opModeIsActive() && (runtime.seconds() < 1)) ;

                driveBackward(4);

                stopRobot();
                runtime.reset();
                while (opModeIsActive() && (runtime.seconds() < 0.3)) ;


            }

            else if (gamepad1.b) {
                driveBackward(4);

                stopRobot();
                runtime.reset();
                while (opModeIsActive() && (runtime.seconds() < 0.3)) ;

                //wrist
                servo_3.setPosition(0);

                stopRobot();
                runtime.reset();
                while (opModeIsActive() && (runtime.seconds() < 0.3)) ;

                rotateLeftDegrees(63);

                stopRobot();
                runtime.reset();
                while (opModeIsActive() && (runtime.seconds() < 0.3)) ;

                driveBackward(2);
                stopRobot();
                runtime.reset();
                while (opModeIsActive() && (runtime.seconds() < 0.3)) ;

                //wrist
                servo_3.setPosition(0.4);

                stopRobot();
                runtime.reset();
                while (opModeIsActive() && (runtime.seconds() < 0.3)) ;


                //grabber
                servo_2.setPosition(1);

                stopRobot();
                runtime.reset();
                while (opModeIsActive() && (runtime.seconds() < 0.3)) ;

                rotateRightDegrees(2);
                stopRobot();
                runtime.reset();
                while (opModeIsActive() && (runtime.seconds() < 0.3)) ;


                driveForward(6);

                stopRobot();
                runtime.reset();
                while (opModeIsActive() && (runtime.seconds() < 0.3)) ;

                //grabber
                servo_2.setPosition(0);

                stopRobot();
                runtime.reset();
                while (opModeIsActive() && (runtime.seconds() < 0.3)) ;


                driveBackward(7);

                stopRobot();
                runtime.reset();
                while (opModeIsActive() && (runtime.seconds() < 0.3)) ;

                //wrist
                servo_3.setPosition(0);

                stopRobot();
                runtime.reset();
                while (opModeIsActive() && (runtime.seconds() < 0.3)) ;

                rotateRightDegrees(40);

                stopRobot();
                runtime.reset();
                while (opModeIsActive() && (runtime.seconds() < 0.3)) ;

                driveForward(4);

                stopRobot();
                runtime.reset();
                while (opModeIsActive() && (runtime.seconds() < 1)) ;

                //wrist
                servo_3.setPosition(0.4);

                stopRobot();
                runtime.reset();
                while (opModeIsActive() && (runtime.seconds() < 1)) ;

                //grabber
                servo_2.setPosition(1);

                stopRobot();
                runtime.reset();
                while (opModeIsActive() && (runtime.seconds() < 1)) ;

                driveBackward(4);

                stopRobot();
                runtime.reset();
                while (opModeIsActive() && (runtime.seconds() < 0.3)) ;


            }
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            //  telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
            telemetry.update();


        }


    }

    public void stopRobot() {
        leftfront.setPower(0);
        rightfront.setPower(0);
        leftrear.setPower(0);
        rightrear.setPower(0);
    }

    public void rotateRightDegrees(int degrees) {
        int newLeftFrontTarget;
        int newRightFrontTarget;
        int newLeftRearTarget;
        int newRightRearTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftFrontTarget = leftfront.getCurrentPosition() + (int) (degrees * Counts_PER_DegreeTurn);
            newRightFrontTarget = rightfront.getCurrentPosition() + (int) (degrees * -Counts_PER_DegreeTurn);
            newLeftRearTarget = leftrear.getCurrentPosition() + (int) (degrees * Counts_PER_DegreeTurn);
            newRightRearTarget = rightrear.getCurrentPosition() + (int) (degrees * -Counts_PER_DegreeTurn);
            leftfront.setTargetPosition(newLeftFrontTarget);
            rightfront.setTargetPosition(newRightFrontTarget);
            leftrear.setTargetPosition(newLeftRearTarget);
            rightrear.setTargetPosition(newRightRearTarget);

            // Turn On RUN_TO_POSITION
            leftfront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightfront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftrear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightrear.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            leftfront.setPower(Math.abs(DRIVE_SPEED));
            rightfront.setPower(Math.abs(DRIVE_SPEED));
            leftrear.setPower(Math.abs(DRIVE_SPEED));
            rightrear.setPower(Math.abs(DRIVE_SPEED));
            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (leftfront.isBusy() && rightfront.isBusy() && leftrear.isBusy() && rightrear.isBusy())) {

                // Display it for the driver.


                telemetry.addData("Leftfront", "%7d", leftfront.getCurrentPosition());
                telemetry.addData("Rightfront", "%7d", rightfront.getCurrentPosition());
                telemetry.addData("Leftrear", "%7d", leftrear.getCurrentPosition());
                telemetry.addData("Rightrear", "%7d", rightrear.getCurrentPosition());
                telemetry.update();

            }

            // Stop all motion;
            stopRobot();

            // Turn off RUN_TO_POSITION
            leftfront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightfront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftrear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightrear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }
    public void rotateLeftDegrees(int degrees) {
        int newLeftFrontTarget;
        int newRightFrontTarget;
        int newLeftRearTarget;
        int newRightRearTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftFrontTarget = leftfront.getCurrentPosition() + (int)(degrees * -Counts_PER_DegreeTurn);
            newRightFrontTarget = rightfront.getCurrentPosition() + (int)(degrees * Counts_PER_DegreeTurn);
            newLeftRearTarget = leftrear.getCurrentPosition() + (int)(degrees * -Counts_PER_DegreeTurn);
            newRightRearTarget = rightrear.getCurrentPosition() + (int)(degrees * Counts_PER_DegreeTurn);
            leftfront.setTargetPosition(newLeftFrontTarget);
            rightfront.setTargetPosition(newRightFrontTarget);
            leftrear.setTargetPosition(newLeftRearTarget);
            rightrear.setTargetPosition(newRightRearTarget);

            // Turn On RUN_TO_POSITION
            leftfront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightfront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftrear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightrear.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            leftfront.setPower(Math.abs(DRIVE_SPEED));
            rightfront.setPower(Math.abs(DRIVE_SPEED));
            leftrear.setPower(Math.abs(DRIVE_SPEED));
            rightrear.setPower(Math.abs(DRIVE_SPEED));
            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (leftfront.isBusy() && rightfront.isBusy() && leftrear.isBusy() && rightrear.isBusy())) {

                // Display it for the driver.


                telemetry.addData("Leftfront", "%7d", leftfront.getCurrentPosition());
                telemetry.addData("Rightfront", "%7d", rightfront.getCurrentPosition() );
                telemetry.addData("Leftrear", "%7d", leftrear.getCurrentPosition());
                telemetry.addData("Rightrear", "%7d", rightrear.getCurrentPosition());
                telemetry.update();

            }

            // Stop all motion;
            stopRobot();

            // Turn off RUN_TO_POSITION
            leftfront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightfront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftrear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightrear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        }
    }

    public void driveForward(int inches) {
        int newLeftFrontTarget;
        int newRightFrontTarget;
        int newLeftRearTarget;
        int newRightRearTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftFrontTarget = leftfront.getCurrentPosition() + (int)(inches * COUNTS_PER_INCH);
            newRightFrontTarget = rightfront.getCurrentPosition() + (int)(inches * COUNTS_PER_INCH);
            newLeftRearTarget = leftrear.getCurrentPosition() + (int)(inches * COUNTS_PER_INCH);
            newRightRearTarget = rightrear.getCurrentPosition() + (int)(inches * COUNTS_PER_INCH);
            leftfront.setTargetPosition(newLeftFrontTarget);
            rightfront.setTargetPosition(newRightFrontTarget);
            leftrear.setTargetPosition(newLeftRearTarget);
            rightrear.setTargetPosition(newRightRearTarget);

            // Turn On RUN_TO_POSITION
            leftfront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightfront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftrear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightrear.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            leftfront.setPower(Math.abs(DRIVE_SPEED));
            rightfront.setPower(Math.abs(DRIVE_SPEED));
            leftrear.setPower(Math.abs(DRIVE_SPEED));
            rightrear.setPower(Math.abs(DRIVE_SPEED));
            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (leftfront.isBusy() && rightfront.isBusy() && leftrear.isBusy() && rightrear.isBusy())) {

                // Display it for the driver.


                telemetry.addData("Leftfront", "%7d", leftfront.getCurrentPosition());
                telemetry.addData("Rightfront", "%7d", rightfront.getCurrentPosition() );
                telemetry.addData("Leftrear", "%7d", leftrear.getCurrentPosition());
                telemetry.addData("Rightrear", "%7d", rightrear.getCurrentPosition());
                telemetry.update();

            }

            // Stop all motion;
            stopRobot();

            // Turn off RUN_TO_POSITION
            leftfront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightfront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftrear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightrear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        }
    }


    public void driveBackward(int inches) {
        int newLeftFrontTarget;
        int newRightFrontTarget;
        int newLeftRearTarget;
        int newRightRearTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftFrontTarget = leftfront.getCurrentPosition() + (int)(inches * -COUNTS_PER_INCH);
            newRightFrontTarget = rightfront.getCurrentPosition() + (int)(inches * -COUNTS_PER_INCH);
            newLeftRearTarget = leftrear.getCurrentPosition() + (int)(inches * -COUNTS_PER_INCH);
            newRightRearTarget = rightrear.getCurrentPosition() + (int)(inches * -COUNTS_PER_INCH);
            leftfront.setTargetPosition(newLeftFrontTarget);
            rightfront.setTargetPosition(newRightFrontTarget);
            leftrear.setTargetPosition(newLeftRearTarget);
            rightrear.setTargetPosition(newRightRearTarget);

            // Turn On RUN_TO_POSITION
            leftfront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightfront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftrear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightrear.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            leftfront.setPower(Math.abs(DRIVE_SPEED));
            rightfront.setPower(Math.abs(DRIVE_SPEED));
            leftrear.setPower(Math.abs(DRIVE_SPEED));
            rightrear.setPower(Math.abs(DRIVE_SPEED));
            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (leftfront.isBusy() && rightfront.isBusy() && leftrear.isBusy() && rightrear.isBusy())) {

                // Display it for the driver.


                telemetry.addData("Leftfront", "%7d", leftfront.getCurrentPosition());
                telemetry.addData("Rightfront", "%7d", rightfront.getCurrentPosition() );
                telemetry.addData("Leftrear", "%7d", leftrear.getCurrentPosition());
                telemetry.addData("Rightrear", "%7d", rightrear.getCurrentPosition());
                telemetry.update();

            }

            // Stop all motion;
            stopRobot();

            // Turn off RUN_TO_POSITION
            leftfront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightfront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftrear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightrear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        }
    }

}


