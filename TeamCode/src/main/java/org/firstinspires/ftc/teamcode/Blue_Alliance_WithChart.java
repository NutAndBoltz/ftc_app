package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by gsbol on 11/20/17.
 */
@Autonomous(name="Blue_Alliance_WithChart", group="Wang")
public class Blue_Alliance_WithChart extends LinearOpMode {


    ColorSensor color_sensor;
    DcMotor leftfront = null;
    DcMotor rightfront = null;
    DcMotor leftrear = null;
    DcMotor rightrear = null;
    Servo arm = null;

    public static final double DRIVE_SPEED = 0.5;
    public static final double Down_SERVO = 1;
    public static final double Up_SERVO = 0.3;
    public static final double START_SERVO = 0.3;

    HardwareMap hwMap = null;
    private ElapsedTime runtime = new ElapsedTime();
    // ...

    public void runOpMode() {


        leftfront = hardwareMap.get(DcMotor.class, "motor_1");
        rightfront = hardwareMap.get(DcMotor.class, "motor_2");
        leftrear = hardwareMap.get(DcMotor.class, "motor_3");
        rightrear = hardwareMap.get(DcMotor.class, "motor_4");

        leftfront.setDirection(DcMotor.Direction.FORWARD);
        leftrear.setDirection(DcMotor.Direction.FORWARD);
        rightfront.setDirection(DcMotor.Direction.REVERSE);
        rightrear.setDirection(DcMotor.Direction.REVERSE);



        color_sensor = hardwareMap.get(ColorSensor.class, "color");

        leftfront.setPower(0);
        rightfront.setPower(0);
        leftrear.setPower(0);
        rightrear.setPower(0);


        color_sensor.enableLed(false);

        leftfront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightfront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftrear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightrear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        arm = hardwareMap.get(Servo.class, "servo_1");

        arm.setPosition(START_SERVO);

        telemetry.addData("Status", "Ready to run");
        telemetry.update();

        waitForStart();

        arm.setPosition(Down_SERVO);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 3.0)) {
            telemetry.addData("a", "%02x", color_sensor.alpha());
            telemetry.addData("r", "%02x", color_sensor.red());
            telemetry.addData("r", "%02x", color_sensor.green());
            telemetry.addData("r", "%02x", color_sensor.blue());
            telemetry.update();
        }

        // See Color Blue as Blue Alliance, go backward
        if (color_sensor.blue() > color_sensor.red() && color_sensor.blue() >10) {
            leftfront.setPower(DRIVE_SPEED);
            rightfront.setPower(-DRIVE_SPEED);
            leftrear.setPower(DRIVE_SPEED);
            rightrear.setPower(-DRIVE_SPEED);
            runtime.reset();
            while (opModeIsActive() && (runtime.seconds() < 0.25)) ;
            {
                telemetry.addData("Turn Right Sir", "Speed:", DRIVE_SPEED);
                telemetry.update();
            }

            arm.setPosition(Up_SERVO);
            leftfront.setPower(0);
            rightfront.setPower(0);
            leftrear.setPower(0);
            rightrear.setPower(0);
            runtime.reset();
            while (opModeIsActive() && (runtime.seconds() < 3)) ;
            {
                telemetry.addData("Stop Sir", "Speed:", DRIVE_SPEED);
                telemetry.update();
            }

            leftfront.setPower(-DRIVE_SPEED);
            rightfront.setPower(DRIVE_SPEED);
            leftrear.setPower(-DRIVE_SPEED);
            rightrear.setPower(DRIVE_SPEED);
            runtime.reset();
            while (opModeIsActive() && (runtime.seconds() < 0.25)) ;
            {
                telemetry.addData("Turn Left Sir", "Speed:", DRIVE_SPEED);
                telemetry.update();
            }

            leftfront.setPower(0);
            rightfront.setPower(0);
            leftrear.setPower(0);
            rightrear.setPower(0);
            runtime.reset();
            while (opModeIsActive() && (runtime.seconds() < 0.5)) ;
            {
                telemetry.addData("Stop Sir", "Speed:", DRIVE_SPEED);
                telemetry.update();
            }



            leftfront.setPower(-DRIVE_SPEED);
            rightfront.setPower(-DRIVE_SPEED);
            leftrear.setPower(-DRIVE_SPEED);
            rightrear.setPower(-DRIVE_SPEED);
            runtime.reset();
            while (opModeIsActive() && (runtime.seconds() < 1)) ;
            {
                telemetry.addData("Moving Backward Sir", "Speed:", DRIVE_SPEED);
                telemetry.update();
            }

            leftfront.setPower(DRIVE_SPEED);
            rightfront.setPower(-DRIVE_SPEED);
            leftrear.setPower(-DRIVE_SPEED);
            rightrear.setPower(DRIVE_SPEED);
            runtime.reset();
            while (opModeIsActive() && (runtime.seconds() < 0.25)) ;
            {
                telemetry.addData("Sliding Right Sir", "Speed:", DRIVE_SPEED);
                telemetry.update();
            }

        // See Red

        } else if (color_sensor.red() > color_sensor.blue() && color_sensor.red() >10 ) {
            leftfront.setPower(-DRIVE_SPEED);
            rightfront.setPower(DRIVE_SPEED);
            leftrear.setPower(-DRIVE_SPEED);
            rightrear.setPower(DRIVE_SPEED);
            runtime.reset();
            while (opModeIsActive() && (runtime.seconds() < 0.25)) ;
            {
                telemetry.addData("Turn Left Sir", "Speed:", DRIVE_SPEED);
                telemetry.update();
            }

            arm.setPosition(Up_SERVO);
            leftfront.setPower(0);
            rightfront.setPower(0);
            leftrear.setPower(0);
            rightrear.setPower(0);
            runtime.reset();
            while (opModeIsActive() && (runtime.seconds() < 3)) ;
            {
                telemetry.addData("Stop Sir", "Speed:", DRIVE_SPEED);
                telemetry.update();
            }

            leftfront.setPower(DRIVE_SPEED);
            rightfront.setPower(-DRIVE_SPEED);
            leftrear.setPower(DRIVE_SPEED);
            rightrear.setPower(-DRIVE_SPEED);
            runtime.reset();
            while (opModeIsActive() && (runtime.seconds() < 0.25)) ;
            {
                telemetry.addData("Turn Right Sir", "Speed:", DRIVE_SPEED);
                telemetry.update();
            }

            leftfront.setPower(0);
            rightfront.setPower(0);
            leftrear.setPower(0);
            rightrear.setPower(0);
            runtime.reset();
            while (opModeIsActive() && (runtime.seconds() < 0.5)) ;
            {
                telemetry.addData("Stop Sir", "Speed:", DRIVE_SPEED);
                telemetry.update();
            }



            leftfront.setPower(-DRIVE_SPEED);
            rightfront.setPower(-DRIVE_SPEED);
            leftrear.setPower(-DRIVE_SPEED);
            rightrear.setPower(-DRIVE_SPEED);
            runtime.reset();
            while (opModeIsActive() && (runtime.seconds() < 1)) ;
            {
                telemetry.addData("Moving Backward Sir", "Speed:", DRIVE_SPEED);
                telemetry.update();
            }

            leftfront.setPower(DRIVE_SPEED);
            rightfront.setPower(-DRIVE_SPEED);
            leftrear.setPower(-DRIVE_SPEED);
            rightrear.setPower(DRIVE_SPEED);
            runtime.reset();
            while (opModeIsActive() && (runtime.seconds() < 0.25)) ;
            {
                telemetry.addData("Sliding Right Sir", "Speed:", DRIVE_SPEED);
                telemetry.update();
            }


        }

        leftfront.setPower(0);
        rightfront.setPower(0);
        leftrear.setPower(0);
        rightrear.setPower(0);
        arm.setPosition(Up_SERVO);

        telemetry.addData("Congratulations!!", "Task Accomplished");
        telemetry.update();
        sleep(1000);
    }
}








