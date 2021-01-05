package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.Locale;


@Autonomous (name = "RedSeekSkystone", group = "12806") // drive to bridge via SkyStone path
//@Autonomous (name = "RedSeekSkystone_Wall", group = "12806") // drive to bridge via wall path

@Disabled
public class RedSeekSkystone extends LinearOpMode {

    Robokenbot robot   = new Robokenbot();

    public void runOpMode() throws InterruptedException {

        float hsvValues[] = {0F, 0F, 0F};

        // sometimes it helps to multiply the raw RGB values with a scale factor
        // to amplify/attentuate the measured values.
        final double SCALE_FACTOR = 255;

        robot.init(hardwareMap,this);

        waitForStart();

        // raise claw to default position
        robot.claw.setPosition((1.0));

        // step 1 - encoder drive toward the wall of stones
        robot.encoderDrive(0.4,-28,-28,8000,this);

        // step 2 - strafe left (robot is backwards, so use opposite) toward the wall to park in front of the 3rd stone from wall
        robot.strafeRightByTime(0.4,1100);


        // step 3 - seek the Skystone by strafing until the color sensor sees black
        robot.strafeLeft(0.25);
        do {
            // convert the RGB values to HSV values.
            // multiply by the SCALE_FACTOR.
            // then cast it back to int (SCALE_FACTOR is a double)
            Color.RGBToHSV((int) (robot.sensorColor.red() * SCALE_FACTOR),
                    (int) (robot.sensorColor.green() * SCALE_FACTOR),
                    (int) (robot.sensorColor.blue() * SCALE_FACTOR),
                    hsvValues);

            telemetry.addData("Distance (cm)",
                    String.format(Locale.US, "%.02f", robot.sensorDistance.getDistance(DistanceUnit.CM)));
            telemetry.addData("Hue", hsvValues[0]);
            telemetry.update();

        } while (hsvValues[0] < 101);

        robot.stopDriving();

        // we can add conditional logic here to skip all steps below except for step 6 if we don't detect any Skystones.
        // if hsvValues[0] is not within a range that resembles black, we immediately perform step 6 and park under the SkyBridge
        // the issue is that if this scenario occurs, we're going to lower the claw into the ground and find no SkyStone, then possible ruin the claw as we run thru the motions

        // step 4 - grab the Skystone. We may need to move forward a tad to position the robot.
//        robot.strafeLeftByTime(0.2,500); // try to center robot in front of Skystone.  power of .1 is too low.
        robot.encoderDrive(0.1,-4,-4,4000,this);
        robot.claw.setPosition((0.0));
        sleep(1000);

        // step 5 - back the robot away a tad from wall of stones, to avoid hitting the Skybridge pylon in the next step, then rotate
        robot.encoderDrive(0.2,6,6,5000,this);
        // for SeekSkystone_Wall version, we want to drive to bridge via wall path to make room for alliance partner, so we will need to move all the way back to the wall instead of just a few inches back

        robot.rotate(-90, .2, true, this);

        // step 6 - Seek SkyBridge - strafe toward the Skybridge using downward color sensor
        robot.driveForward(-0.2);
        do {
            // convert the RGB values to HSV values.
            // multiply by the SCALE_FACTOR.
            // then cast it back to int (SCALE_FACTOR is a double)
            Color.RGBToHSV((int) (robot.bottomSensorColor.red() * SCALE_FACTOR),
                    (int) (robot.bottomSensorColor.green() * SCALE_FACTOR),
                    (int) (robot.bottomSensorColor.blue() * SCALE_FACTOR),
                    hsvValues);

            telemetry.addData("Distance (cm)",
                    String.format(Locale.US, "%.02f", robot.sensorDistance.getDistance(DistanceUnit.CM)));
            telemetry.addData("Hue", hsvValues[0]);
            telemetry.update();

        } while (hsvValues[0] < 101);
        robot.stopDriving();

        // step 7 - Continue past Skybridge and into Building area
        robot.encoderDrive(0.5,-18,-18,5000,this);

        // step 8 - Release the SkyStone
        robot.claw.setPosition((1.0));

        // step 9 - Seek SkyBridge and park (alternately, we can can downward color sensor instead of distance)
        robot.strafeRightByTime(0.2,500);
        robot.encoderDrive(0.4,20,20,8000,this);

        telemetry.addData("Path", "Complete");
        telemetry.update();
    }

}
