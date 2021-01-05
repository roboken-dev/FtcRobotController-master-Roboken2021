package org.firstinspires.ftc.teamcode;

import android.app.Activity;
import android.graphics.Color;
import android.view.View;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import com.qualcomm.hardware.bosch.BNO055IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;



public class Robokenbot
{
    public DcMotor motorFrontLeft;  // motor1
    public DcMotor motorRearLeft;  // motor 2
    public DcMotor motorFrontRight; // motor 3
    public DcMotor motorRearRight; // motor 4
    public Servo claw;
    public CRServo arm;
    public DigitalChannel digitalTouch;  // Hardware Device Object
    public ColorSensor sensorColor;
    public DistanceSensor sensorDistance;
    public CRServo intakeServo1;
    public CRServo intakeServo2;
    public ColorSensor bottomSensorColor;
    public Servo capstoneServo;

    private ElapsedTime     runtime = new ElapsedTime();

    BNO055IMU               imu;  //Note: you must configure the IMU on I2C channel 0, port 0.
    Orientation             lastAngles = new Orientation();
    double                  globalAngle, power = .30, correction;



    static final double     SCALE_FACTOR = 75.0/75.0; //  if drive speed = .2 or .3 use 75.0/75.0;  .5 is 75.0/76.0 .4 is 75.0/75.5 if drive_speed = .1, use 1.0; if drive_speed = .3, use 75.0/77.0 note that .3 has hard time braking
    static final double     COUNTS_PER_MOTOR_REV    = 1440 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 0.5 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (SCALE_FACTOR * COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = 0.3;


    HardwareMap hwMap = null;
    private ElapsedTime period =new ElapsedTime();

    public Robokenbot(){ }


    public void init(HardwareMap ahwMap, LinearOpMode opmode) {

        hwMap=ahwMap;
        motorFrontLeft = hwMap.dcMotor.get("motorFrontLeft");
        motorRearLeft = hwMap.dcMotor.get("motorRearLeft");
        motorFrontRight = hwMap.dcMotor.get("motorFrontRight");
        motorRearRight = hwMap.dcMotor.get("motorRearRight");
        //       digitalTouch = hardwareMap.get(DigitalChannel.class, "sensor_digital");

        // get a reference to the color sensor.
        sensorColor = hwMap.get(ColorSensor.class, "sensor_color_distance");
        bottomSensorColor = hwMap.get(ColorSensor.class,"Bottom Color Sensor");

        // get a reference to the distance sensor that shares the same name.
        sensorDistance = hwMap.get(DistanceSensor.class, "sensor_color_distance");

        claw = hwMap.servo.get("servo");
        arm=hwMap.crservo.get("arm");
        intakeServo1=hwMap.crservo.get("intakeServo1");
        intakeServo2=hwMap.crservo.get("intakeServo2");
        capstoneServo=hwMap.servo.get("capstone");

        motorFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorRearLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorRearRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        
        // reset to default
        initRunWithoutEncoder();

        imu = hwMap.get(BNO055IMU.class, "imu");

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode                = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = false;

        imu.initialize(parameters);

        opmode.telemetry.addData("Status", "Calibrating IMU...");
        opmode.telemetry.update();

        // make sure the imu gyro is calibrated before continuing.
        while (!opmode.isStopRequested() && !imu.isGyroCalibrated())
        {
            opmode.sleep(50);
            opmode.idle();
        }

        resetAngle(); // reset robot IMU angle to current heading. Zero.

        opmode.telemetry.addData("Status", "Ready for Start");
        opmode.telemetry.addData("imu calib status", imu.getCalibrationStatus().toString());
        opmode.telemetry.update();

    }

    public void encoderDrive(double speed,
                             double leftInches, double rightInches,
                             double timeoutS, LinearOpMode opmode) {
        int newFrontLeftTarget;
        int newFrontRightTarget;
        int newRearLeftTarget;
        int newRearRightTarget;

        // Send telemetry message to signify robot waiting;
        opmode.telemetry.addData("Status", "Resetting Encoders");    //
        opmode.telemetry.update();

        initRunWithEncoder();

        // Send telemetry message to indicate successful Encoder reset
        opmode.telemetry.addData("Path0",  "Starting at %7d :%7d",
                motorFrontLeft.getCurrentPosition(),
                motorFrontRight.getCurrentPosition(),
                motorRearLeft.getCurrentPosition(),
                motorRearRight.getCurrentPosition());
        opmode.telemetry.update();

        // Ensure that the opmode is still active
        if (opmode.opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newFrontLeftTarget = (motorFrontLeft.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH));
            newFrontRightTarget = (motorFrontRight.getCurrentPosition() - (int)(rightInches * COUNTS_PER_INCH));
            newRearLeftTarget = -(motorRearLeft.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH));
            newRearRightTarget = -(motorRearRight.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH));

            motorFrontLeft.setTargetPosition(newFrontLeftTarget);
            motorFrontRight.setTargetPosition(newFrontRightTarget);
            motorRearLeft.setTargetPosition(newRearLeftTarget);
            motorRearRight.setTargetPosition(newRearRightTarget);

            // Turn On RUN_TO_POSITION
            motorFrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorFrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorRearLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorRearRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            // reset the timeout time and start motion.
            runtime.reset();

            motorFrontLeft.setPower(Math.abs(speed));
            motorFrontRight.setPower(Math.abs(speed));
            motorRearLeft.setPower(Math.abs(speed));
            motorRearRight.setPower(-Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opmode.opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (motorFrontLeft.isBusy() && motorFrontRight.isBusy() && motorRearLeft.isBusy() && motorRearRight.isBusy())) {

                // Display it for the driver.
                opmode.telemetry.addData("Path1",  "Running to %7d :%7d : %d : %d", newFrontLeftTarget,  newFrontRightTarget, newRearLeftTarget, newRearRightTarget);
                opmode.telemetry.addData("Path2",  "Running at %7d :%7d : %7d : %7d",
                        motorFrontLeft.getCurrentPosition(),
                        motorFrontRight.getCurrentPosition(),
                        motorRearLeft.getCurrentPosition(),
                        motorRearRight.getCurrentPosition());
                opmode.telemetry.update();

            }

            // Stop all motion;
            motorFrontLeft.setPower(0);
            motorFrontRight.setPower(0);
            motorRearLeft.setPower(0);
            motorRearRight.setPower(0);

            initRunWithoutEncoder();
            // Turn off RUN_TO_POSITION

        }
    }

    public void initRunWithEncoder()
    {
        motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRearLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRearRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        motorFrontLeft.setDirection(DcMotor.Direction.REVERSE);
        motorFrontRight.setDirection(DcMotor.Direction.REVERSE);
        motorRearRight.setDirection(DcMotor.Direction.FORWARD);
        motorRearLeft.setDirection(DcMotor.Direction.FORWARD);

        motorFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRearLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRearRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFrontRight.setDirection(DcMotor.Direction.REVERSE);
        motorRearLeft.setDirection(DcMotor.Direction.REVERSE);
    }

    public void initRunWithoutEncoder()
    {
        motorFrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorRearLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorRearRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        motorFrontLeft.setDirection(DcMotor.Direction.REVERSE);
        motorFrontRight.setDirection(DcMotor.Direction.FORWARD);
        motorRearRight.setDirection(DcMotor.Direction.REVERSE);
        motorRearLeft.setDirection(DcMotor.Direction.FORWARD);
    }



    public void driveForward(double power) {
        motorFrontLeft.setPower(power);
        motorFrontRight.setPower(power);
        motorRearLeft.setPower(power);
        motorRearRight.setPower(power);
    }

    public void driveBackward(double power) {
        motorFrontLeft.setPower(-power);
        motorFrontRight.setPower(-power);
        motorRearLeft.setPower(-power);
        motorRearRight.setPower(-power);
    }

    public void stopDriving() {
        driveForward(0);
    }

    public void turnLeft(double power) {
        motorFrontLeft.setPower(-power);
        motorFrontRight.setPower(power);
        motorRearLeft.setPower(-power);
        motorRearRight.setPower(power);
    }

    public void turnRight(double power) {
        turnLeft(-power);
    }

    public void driveForwardByTime(double power, long time) throws InterruptedException {
        driveForward(power);
        Thread.sleep(time);
        // don't we need to add stopDriving() ?
        //stopDriving();

    }

    public void turnLeftByTime(double power, long time) throws InterruptedException {
        turnLeft(power);
        Thread.sleep(time);
        // don't we need to add stopDriving() ?
        //stopDriving();

    }

    public void turnRightByTime(double power, long time) throws InterruptedException {
        turnRight(power);
        Thread.sleep(time);
        // don't we need to add stopDriving() ?
        //stopDriving();
    }


    public void driveTillTouched(double power) {
        while (digitalTouch.getState() == true) {
            driveForward(power);
        }
        // don't we need to add stopDriving() ?
        //stopDriving();
    }

    public void driveTillThisClose(double power, double distance) {
        double howfar;

        while (Double.isNaN(sensorDistance.getDistance(DistanceUnit.CM)) || sensorDistance.getDistance(DistanceUnit.CM) > distance) {
            driveForward(power);
        }
        // don't we need to add stopDriving() ?
        //stopDriving();
    }

    public void strafeRight (double power)
    {
        motorFrontLeft.setPower(power);
        motorRearLeft.setPower(-power);
        motorFrontRight.setPower(-power);
        motorRearRight.setPower(power);
    }

    public void strafeLeft(double power) {
        motorFrontLeft.setPower(-power);
        motorRearLeft.setPower(power);
        motorFrontRight.setPower(power);
        motorRearRight.setPower(-power);
    }

    public void strafeLeftByTime(double power, long time) throws InterruptedException {
        strafeLeft(power);
        Thread.sleep(time);
        // don't we need to add stopDriving() ?
        //stopDriving();
    }

    public void strafeRightByTime(double power, long time) throws InterruptedException {
        strafeRight(power);
        Thread.sleep(time);
        // don't we need to add stopDriving() ?
        //stopDriving();
    }

    // IMU sample from STEMRobotics educational example

    /*  for auto-correction while driving, using this pseudocode

            while (opModeIsActive())
        {
            // Use gyro to drive in a straight line.
            correction = checkDirection();

            telemetry.addData("1 imu heading", lastAngles.firstAngle);
            telemetry.addData("2 global heading", globalAngle);
            telemetry.addData("3 correction", correction);
            telemetry.update();

            leftMotor.setPower(power - correction);  // this is for regular motors. adapt for mech.
            rightMotor.setPower(power + correction);

     */




    /**
     * Resets the cumulative angle tracking to zero.
     */
    public void resetAngle()
    {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        globalAngle = 0;
    }

    /**
     * Get current cumulative angle rotation from last reset.
     * @return Angle in degrees. + = left, - = right.
     */
    private double getAngle()
    {
        // We experimentally determined the Z axis is the axis we want to use for heading angle.
        // We have to process the angle because the imu works in euler angles so the Z axis is
        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.

        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;

        lastAngles = angles;

        return globalAngle;
    }

    /**
     * See if we are moving in a straight line and if not return a power correction value.
     * @return Power adjustment, + is adjust left - is adjust right.
     */
    public double checkDirection()
    {
        // The gain value determines how sensitive the correction is to direction changes.
        // You will have to experiment with your robot to get small smooth direction changes
        // to stay on a straight line.
        double correction, angle, gain = .10;

        angle = getAngle();

        if (angle == 0)
            correction = 0;             // no adjustment.
        else
            correction = -angle;        // reverse sign of angle for correction.

        correction = correction * gain;

        return correction;
    }

    /**
     * Rotate left or right the number of degrees. Does not support turning more than 180 degrees.
     * @param degrees Degrees to turn, + is left - is right
     */
    public void rotate(int degrees, double power, boolean retainCurrentAngle, LinearOpMode opmode)
    {
        double  leftPower, rightPower;

        // restart imu movement tracking.
        if(retainCurrentAngle==false) {
            resetAngle();
        }

        // getAngle() returns + when rotating counter clockwise (left) and - when rotating
        // clockwise (right).

        if (degrees < 0)
        {   // turn right.
            turnRight(power);
        }
        else if (degrees > 0)
        {   // turn left.
            turnLeft(power);
        }
        else return;

        // rotate until turn is completed.
        if (degrees < 0)
        {
            // On right turn we have to get off zero first.
            while (opmode.opModeIsActive() && getAngle() == 0) {}

            while (opmode.opModeIsActive() && getAngle() > degrees) {
                double currentAngle = getAngle();

                opmode.telemetry.addData("angle is ",String.valueOf(currentAngle));
                opmode.telemetry.update();
            }
        }
        else    // left turn.
            while (opmode.opModeIsActive() && getAngle() < degrees) {
                double currentAngle = getAngle();

                opmode.telemetry.addData("angle is ",String.valueOf(currentAngle));
                opmode.telemetry.update();

            }

        // turn the motors off.

        stopDriving();

        // wait for rotation to stop.
        opmode.sleep(500);

        // reset angle tracking on new heading.
        resetAngle();
    }

}
