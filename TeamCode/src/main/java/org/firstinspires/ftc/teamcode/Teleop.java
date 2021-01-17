package org.firstinspires.ftc.teamcode;

import android.app.Activity;
import android.graphics.Color;
import android.view.View;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.Locale;
//test again

@TeleOp(name = "TeleOp", group = "12806")
public class Teleop extends LinearOpMode {

    Robokenbot robot   = new Robokenbot();

    @Override
    public void runOpMode() throws InterruptedException {
        double speed_control = 0.5;
        double temp;

        double ServoPosition = 0.1;


        //double ArmSpeedControl = 0.6;  DELETE


        robot.init(hardwareMap,this);

        float hsvValues[] = {0F, 0F, 0F};
        final float values[] = hsvValues;
        final double SCALE_FACTOR = 255;



        int relativeLayoutId = hardwareMap.appContext.getResources().getIdentifier("RelativeLayout", "id", hardwareMap.appContext.getPackageName());
        final View relativeLayout = ((Activity) hardwareMap.appContext).findViewById(relativeLayoutId);

        robot.initRunWithoutEncoder();

        //for some reason having this is needed


        robot.motorFrontRight.setDirection(DcMotor.Direction.REVERSE);
        robot.motorRearLeft.setDirection(DcMotor.Direction.REVERSE);
        robot.motorFrontLeft.setDirection(DcMotor.Direction.FORWARD);
        robot.motorRearRight.setDirection(DcMotor.Direction.FORWARD);





        telemetry.addData("Status", "Ready to Go");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            Color.RGBToHSV((int) (robot.bottomSensorColor.red() * SCALE_FACTOR),
                    (int) (robot.bottomSensorColor.green() * SCALE_FACTOR),
                    (int) (robot.bottomSensorColor.blue() * SCALE_FACTOR),
                    hsvValues);

            //telemetry.addData("Distance (cm)",
                    //String.format(Locale.US, "%.02f", robot.sensorDistance.getDistance(DistanceUnit.CM)));

            telemetry.addData("Arm Position",ServoPosition);
            telemetry.addData("Alpha", robot.sensorColor.alpha());
            telemetry.addData("Red  ", robot.sensorColor.red());
            telemetry.addData("Green", robot.sensorColor.green());
            telemetry.addData("Blue ", robot.sensorColor.blue());
            telemetry.addData("Hue", hsvValues[0]);

            relativeLayout.post(new Runnable() {
                public void run() {
                    relativeLayout.setBackgroundColor(Color.HSVToColor(0xff, values));
                }
            });



            double G1rightStickY = gamepad1.right_stick_y;
            double G1leftStickY = gamepad1.left_stick_y;
            float G1rightTrigger = gamepad1.right_trigger;
            float G1leftTrigger = gamepad1.left_trigger;



            //Speed control for drive wheels
            if (gamepad1.dpad_up) {
                speed_control = 1;
                telemetry.addData("Status", "Setting Speed to 1");
                telemetry.update();
            }
            if (gamepad1.dpad_down) {
                speed_control = 0.25f;
                telemetry.addData("Status", "Setting Speed to .25");
                telemetry.update();
            }
            if (gamepad1.dpad_left) {
                speed_control = 0.5f;
                telemetry.addData("Status", "Setting Speed to .5");
                telemetry.update();
            }
            if (gamepad1.dpad_right) {
                speed_control = 0.5f;
                telemetry.addData("Status", "Setting Speed to .5");
                telemetry.update();
            }


            //motion
            if (G1rightTrigger > 0 && G1leftTrigger == 0) {  // Right Strafe

                robot.motorFrontLeft.setPower(-G1rightTrigger * speed_control);
                robot.motorRearLeft.setPower(G1rightTrigger * speed_control);
                robot.motorFrontRight.setPower(G1rightTrigger * speed_control);
                robot.motorRearRight.setPower(-G1rightTrigger * speed_control);

                telemetry.addData("Status", "Strafing Right");
                telemetry.update();

            } else if (G1leftTrigger > 0 && G1rightTrigger == 0) {     //Left Strafe
                robot.motorFrontLeft.setPower(G1leftTrigger * speed_control);
                robot.motorRearLeft.setPower(-G1leftTrigger * speed_control);
                robot.motorFrontRight.setPower(-G1leftTrigger * speed_control);
                robot.motorRearRight.setPower(G1leftTrigger * speed_control);

                telemetry.addData("Status", "Strafing Left");
                telemetry.update();

            } else {     //Tank Drive
                robot.motorFrontLeft.setPower(G1leftStickY * Math.abs(G1leftStickY) * speed_control);
                robot.motorRearLeft.setPower(G1leftStickY * Math.abs(G1leftStickY) * speed_control);
                robot.motorFrontRight.setPower(G1rightStickY * Math.abs(G1rightStickY)  * speed_control);
                robot.motorRearRight.setPower(G1rightStickY * Math.abs(G1rightStickY)  * speed_control);

                telemetry.addData("Status", "Moving");    //
                telemetry.update();
            }





            //arm contorl
            if(gamepad2.a){
                ServoPosition = 0.0;
            }

            if(gamepad2.b){
                ServoPosition = 0.85;
            }

            if(gamepad2.x){
                ServoPosition = 0.20;
            }


            if(gamepad2.dpad_up){
                ServoPosition += 0.01;
            }
            if(gamepad2.dpad_down){
                ServoPosition -= 0.01;
            }

    if(gamepad2.left_stick_y!=0){
            robot.armMotor.setPower(gamepad2.left_stick_y*-0.3*Math.abs(gamepad2.left_stick_y));}
    else {
        robot.armMotor.setPower(gamepad2.right_stick_y*-0.8 * Math.abs(gamepad2.right_stick_y));
    }


            if(gamepad2.left_bumper)
            {
                temp = speed_control;
                speed_control=0;
                robot.armMotor.setPower(0.4);
                sleep(750);
                robot.armMotor.setPower(0.0);
                speed_control=temp;


            }
            //these if statements do not work but we should probably have them
            /*
           if(ServoPosition > 1.0){
                ServoPosition = 1.0;
            }
            if(ServoPosition < 0.0);
            {
                ServoPosition = 0.0;
            }
            */


            robot.ClawServo.setPosition(ServoPosition);


            //Delete all
            /*

            if (gamepad2.left_bumper) {
                robot.claw.setPosition(0.0);
                telemetry.addData("Status", "Claw");
                telemetry.update();
            }

            if (gamepad2.right_bumper) {
                robot.claw.setPosition(1.0);
                telemetry.addData("Status", "Claw");
                telemetry.update();
                idle();

            }


            if (gamepad2.left_stick_y > 0 || gamepad2.left_stick_y < 0) {
                robot.arm.setPower(gamepad2.left_stick_y * ArmSpeedControl+0.2);
            }

            if (gamepad2.left_stick_y < 0) {
                robot.arm.setPower(gamepad2.left_stick_y *( ArmSpeedControl));
            }

            if (gamepad2.left_stick_y == 0) {
                robot.arm.setPower(0.0);
            }

            if (gamepad2.y)
            {
                ArmSpeedControl = 0.6;
            }


            if (gamepad2.a)
            {
                ArmSpeedControl = 0.4;
            }

            if (gamepad2.a)
            {
                ArmSpeedControl = 0.5;
            }


            if (gamepad2.dpad_up){
                robot.intakeServo1.setDirection(CRServo.Direction.FORWARD);
                robot.intakeServo2.setDirection(CRServo.Direction.REVERSE);
                robot.intakeServo1.setPower(0.2);
                robot.intakeServo2.setPower(0.2);

            }

            if (gamepad2.dpad_left) {
                robot.intakeServo1.setPower(0);
                robot.intakeServo2.setPower(0);
            }

            if (gamepad2.dpad_right) {
                robot.intakeServo1.setPower(0);
                robot.intakeServo2.setPower(0);
            }



            if (gamepad2.dpad_down){
                robot.intakeServo1.setDirection(CRServo.Direction.REVERSE);
                robot.intakeServo2.setDirection(CRServo.Direction.FORWARD);
                robot.intakeServo1.setPower(0.2);
                robot.intakeServo2.setPower(0.2);
            }

            if (gamepad2.x) {
                robot.intakeServo1.setDirection(CRServo.Direction.REVERSE);
                robot.intakeServo2.setDirection(CRServo.Direction.FORWARD);
                robot.intakeServo1.setPower(0.2);
                robot.intakeServo2.setPower(0.2);
                robot.driveForward(.17);
                sleep(1300);
                robot.intakeServo1.setPower(0);
                robot.intakeServo2.setPower(0);
                robot.stopDriving();

            }



            robot.capstoneServo.setPosition(1-0.6*gamepad2.right_trigger);


             */
            telemetry.update();
        }

        // Set the panel back to the default color
        relativeLayout.post(new Runnable() {
            public void run() {
                relativeLayout.setBackgroundColor(Color.WHITE);
            }
        });



    }
}
