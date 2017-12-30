/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcontroller.external.samples.ConceptVuforiaNavigation;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuMarkInstanceId;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

/**
 * This OpMode illustrates the basics of using the Vuforia engine to determine
 * the identity of Vuforia VuMarks encountered on the field. The code is structured as
 * a LinearOpMode. It shares much structure with {@link ConceptVuforiaNavigation}; we do not here
 * duplicate the core Vuforia documentation found there, but rather instead focus on the
 * differences between the use of Vuforia for navigation vs VuMark identification.
 *
 * @see ConceptVuforiaNavigation
 * @see VuforiaLocalizer
 * @see VuforiaTrackableDefaultListener
 * see  ftc_app/doc/tutorial/FTC_FieldCoordinateSystemDefinition.pdf
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list.
 *
 * IMPORTANT: In order to use this OpMode, you need to obtain your own Vuforia license key as
 * is explained in {@link ConceptVuforiaNavigation}.
 */

@Autonomous(name="Blue1_Autonomous", group ="Concept")
public class Blue1_Autonomous extends LinearOpMode {

    public static final String TAG = "Vuforia VuMark Sample";

    OpenGLMatrix lastLocation = null;

    VuforiaLocalizer vuforia;

    static final double COUNTS_PER_MOTOR_REV = 2240;    // eg: TETRIX Motor Encoder
    // static final double     DRIVE_GEAR_REDUCTION    = 2.0 ;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    static final double COUNTS_PER_INCH = COUNTS_PER_MOTOR_REV / (WHEEL_DIAMETER_INCHES * 3.1415);

    static final double Counts_PER_DegreeTurn = 50;

    ColorSensor color_sensor;
    DcMotor leftfront = null;
    DcMotor rightfront = null;
    DcMotor leftrear = null;
    DcMotor rightrear = null;
    Servo arm = null;
    Servo glyphArm = null;

    public static final double DRIVE_SPEED = 0.5;
    public static final double Down_SERVO = 1;
    public static final double Up_SERVO = 0.3;
    public static final double START_SERVO = 0.3;
    public static final int Degree_Turned_Jewel = 20;

    HardwareMap hwMap = null;
    private ElapsedTime runtime = new ElapsedTime();


    @Override
    public void runOpMode() {


        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = "AasdOY7/////AAAAmWVwx3oX3E9loWJNt07NJG80am5QdXfkxeve92eE1sLMQsmNNNkRsZyfC2kxic4bvjB36VlDNWoy3K62OvPZRaxo2bWJy9Nkg40o5el4/HYUF8C1u971N34RrHn5lXOnOC7SSQxy/rK2SkFMqFm1YiU/W5QmVxQyAgjp2o8kZD7QSjqke2vZfqMXLr40hl0xh1K5njXdqaD2x4mAQncm5rSOS0qgiiQjNPuMjt95jKqDQuvoa+2RA9cujfg0Ug52PRA2Un4jjpJGhV/upyxFdEworP7AkhTmCuOJihqDlyyvU+khdSOZc1sIq0NKpozR28j3L3jGEy47ntA9GSYgELUMjAOd9XL3DnQRHVlZlmKN";


        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);


        VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        VuforiaTrackable relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate");

        telemetry.addData(">", "Press Play to start");
        telemetry.update();


        leftfront = hardwareMap.get(DcMotor.class, "motor_1");
        rightfront = hardwareMap.get(DcMotor.class, "motor_2");
        leftrear = hardwareMap.get(DcMotor.class, "motor_3");
        rightrear = hardwareMap.get(DcMotor.class, "motor_4");

        leftfront.setDirection(DcMotor.Direction.FORWARD);
        leftrear.setDirection(DcMotor.Direction.FORWARD);
        rightfront.setDirection(DcMotor.Direction.REVERSE);
        rightrear.setDirection(DcMotor.Direction.REVERSE);

        color_sensor = hardwareMap.get(ColorSensor.class, "color");

        stopRobot();

        color_sensor.enableLed(false);

        leftfront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightfront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftrear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightrear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftfront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightfront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftrear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightrear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        arm = hardwareMap.get(Servo.class, "servo_1");
        glyphArm = hardwareMap.get(Servo.class, "servo_4");

        arm.setPosition(START_SERVO);
        glyphArm.setPosition(Down_SERVO);

        telemetry.addData("Status", "Ready to run");
        telemetry.update();


        waitForStart();

        relicTrackables.activate();

        while (opModeIsActive()) {
            knockJewel();


            RelicRecoveryVuMark vuMark=RelicRecoveryVuMark.from(relicTemplate);

            if(vuMark!=RelicRecoveryVuMark.LEFT){

                    parkRobotLeft();
                    telemetry.addData("I SEE LEFT","%s visible",vuMark);
                    telemetry.update();
                }


            else if(vuMark!=RelicRecoveryVuMark.CENTER){
                    parkRobotCenter();
                    telemetry.addData("I SEE CENTER","%s visible",vuMark);
                    telemetry.update();
                }


            else if(vuMark!=RelicRecoveryVuMark.RIGHT){
                    parkRobotRight();
                    telemetry.addData("I SEE Right","%s visible",vuMark);
                    telemetry.update();
                }



        }
    }


    public void knockJewel() {

        arm.setPosition(Down_SERVO);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 3.0)) {
            telemetry.addData("alpha", "%02x", color_sensor.alpha());
            telemetry.addData("red", "%02x", color_sensor.red());
            telemetry.addData("green", "%02x", color_sensor.green());
            telemetry.addData("blue", "%02x", color_sensor.blue());
            telemetry.update();
        }
        if (color_sensor.blue() > color_sensor.red() && color_sensor.blue() > 10) {

            rotateRightDegrees(Degree_Turned_Jewel);

            stopRobot();
            runtime.reset();
            while (opModeIsActive() && (runtime.seconds() < 1)) ;
            {
                telemetry.addData("Stop rotation", "Speed:", DRIVE_SPEED);
                telemetry.update();
            }

            arm.setPosition(Up_SERVO);
            runtime.reset();
            while (opModeIsActive() && (runtime.seconds() < 3)) ;
            {
                telemetry.addData("Lift arm", "Speed:", DRIVE_SPEED);
                telemetry.update();
            }

            rotateLeftDegrees(Degree_Turned_Jewel);

            stopRobot();
            runtime.reset();
            while (opModeIsActive() && (runtime.seconds() < 0.5)) ;
            {
                telemetry.addData("Stop rotation", "Speed:", DRIVE_SPEED);
                telemetry.update();
            }


            // See Red

        } else if (color_sensor.red() > color_sensor.blue() && color_sensor.red() > 10) {

            rotateLeftDegrees(Degree_Turned_Jewel);

            stopRobot();
            runtime.reset();
            while (opModeIsActive() && (runtime.seconds() < 1)) ;
            {
                telemetry.addData("Stop rotation", "Speed:", DRIVE_SPEED);
                telemetry.update();
            }

            arm.setPosition(Up_SERVO);
            runtime.reset();
            while (opModeIsActive() && (runtime.seconds() < 3)) ;
            {
                telemetry.addData("Lift arm", "Speed:", DRIVE_SPEED);
                telemetry.update();
            }

            rotateRightDegrees(Degree_Turned_Jewel);

            stopRobot();
            runtime.reset();
            while (opModeIsActive() && (runtime.seconds() < 0.5)) ;
            {
                telemetry.addData("Stop rotation", "Speed:", DRIVE_SPEED);
                telemetry.update();
            }


            // If doesn't see color, lift arm
        } else {
            arm.setPosition(Up_SERVO);
            runtime.reset();
            while (opModeIsActive() && (runtime.seconds() < 3)) ;
            {
                telemetry.addData("Lift arm", "Speed:", DRIVE_SPEED);
                telemetry.update();
            }

        }

        stopRobot();
        arm.setPosition(Up_SERVO);

        telemetry.addData("Congratulations!!", "Task Accomplished");
        telemetry.update();
        sleep(1000);
    }


    public void parkRobotLeft() {
        driveBackward(29);


        stopRobot();
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 1)) ;
        {
            telemetry.addData("Pause", "Speed:", DRIVE_SPEED);
            telemetry.update();
        }

        rotateRightDegrees(90);


        stopRobot();
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 1)) ;
        {
            telemetry.addData("Pause", "Speed:", DRIVE_SPEED);
            telemetry.update();
        }

        driveBackward(20);


        stopRobot();
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 0.5)) ;
        {
            telemetry.addData("Pause", "Speed:", DRIVE_SPEED);
            telemetry.update();
        }

        glyphArm.setPosition(Up_SERVO);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 3.0)) {
            telemetry.addData("Push glyph", "Speed:", DRIVE_SPEED);
            telemetry.update();
        }

        driveForward(10);


        stopRobot();
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 1)) ;
        {
            telemetry.addData("Pause", "Speed:", DRIVE_SPEED);
            telemetry.update();
        }

        driveBackward(5);
    }


    public void parkRobotCenter() {
        driveBackward(36);


        stopRobot();
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 1)) ;
        {
            telemetry.addData("Pause", "Speed:", DRIVE_SPEED);
            telemetry.update();
        }

        rotateRightDegrees(90);


        stopRobot();
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 1)) ;
        {
            telemetry.addData("Pause", "Speed:", DRIVE_SPEED);
            telemetry.update();
        }

        driveBackward(20);


        stopRobot();
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 0.5)) ;
        {
            telemetry.addData("Pause", "Speed:", DRIVE_SPEED);
            telemetry.update();
        }

        glyphArm.setPosition(Up_SERVO);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 3.0)) {
            telemetry.addData("Push glyph", "Speed:", DRIVE_SPEED);
            telemetry.update();
        }

        driveForward(10);


        stopRobot();
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 1)) ;
        {
            telemetry.addData("Pause", "Speed:", DRIVE_SPEED);
            telemetry.update();
        }

        driveBackward(5);

    }


    public void parkRobotRight() {
        driveBackward(43);


        stopRobot();
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 1)) ;
        {
            telemetry.addData("Pause", "Speed:", DRIVE_SPEED);
            telemetry.update();
        }

        rotateRightDegrees(90);


        stopRobot();
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 1)) ;
        {
            telemetry.addData("Pause", "Speed:", DRIVE_SPEED);
            telemetry.update();
        }

        driveBackward(20);


        stopRobot();
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 0.5)) ;
        {
            telemetry.addData("Pause", "Speed:", DRIVE_SPEED);
            telemetry.update();
        }

        glyphArm.setPosition(Up_SERVO);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 3.0)) {
            telemetry.addData("Push glyph", "Speed:", DRIVE_SPEED);
            telemetry.update();
        }

        driveForward(10);


        stopRobot();
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 1)) ;
        {
            telemetry.addData("Pause", "Speed:", DRIVE_SPEED);
            telemetry.update();
        }

        driveBackward(5);

    }

    public void stopRobot () {
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
            newLeftFrontTarget = leftfront.getCurrentPosition() + (int)(degrees * Counts_PER_DegreeTurn);
            newRightFrontTarget = rightfront.getCurrentPosition() + (int)(degrees * -Counts_PER_DegreeTurn);
            newLeftRearTarget = leftrear.getCurrentPosition() + (int)(degrees * Counts_PER_DegreeTurn);
            newRightRearTarget = rightrear.getCurrentPosition() + (int)(degrees * -Counts_PER_DegreeTurn);
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
