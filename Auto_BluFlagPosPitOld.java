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

import android.app.Activity;
import android.graphics.Color;
import android.view.View;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import static java.lang.Double.NaN;

/**
 * This file illustrates the concept of driving a path based on time.
 * It uses the common Pushbot hardware class to define the drive on the robot.
 * The code is structured as a LinearOpMode
 *
 * The code assumes that you do NOT have encoders on the wheels,
 *   otherwise you would use: PushbotAutoDriveByEncoder;
 *
 *   The desired path in this example is:
 *   - Drive forward for 3 seconds
 *   - Spin right for 1.3 seconds
 *   - Drive Backwards for 1 Second
 *   - Stop and close the claw.
 *
 *  The code is written in a simple form with no optimizations.
 *  However, there are several ways that this type of sequence could be streamlined,
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name="Mineral pushing testing", group="Blu")
@Disabled
public class Auto_BluFlagPosPitOld extends LinearOpMode {

    /* Declare OpMode members. */
    HardwareDrive robot = new HardwareDrive();
    Varibles var = new Varibles();
    private ElapsedTime     runtime = new ElapsedTime();

    @Override
    public void runOpMode() {

        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);



        float hsvValues[] = {0F, 0F, 0F};

        // values is a reference to the hsvValues array.
        final float values[] = hsvValues;

        // sometimes it helps to multiply the raw RGB values with a scale factor
        // to amplify/attentuate the measured values.
        final double SCALE_FACTOR = 255;
        double timeValue = 8;

        // get a reference to the RelativeLayout so we can change the background
        // color of the Robot Controller app to match the hue detected by the RGB sensor.
        int relativeLayoutId = hardwareMap.appContext.getResources().getIdentifier("RelativeLayout", "id", hardwareMap.appContext.getPackageName());
        final View relativeLayout = ((Activity) hardwareMap.appContext).findViewById(relativeLayoutId);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Ready to run");    //
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        Color.RGBToHSV((int) (robot.sensorColor.red() * SCALE_FACTOR),
                (int) (robot.sensorColor.green() * SCALE_FACTOR),
                (int) (robot.sensorColor.blue() * SCALE_FACTOR),
                hsvValues);
//        while(robot.sensorDistance.getDistance(DistanceUnit.CM) > 7.0 || robot.sensorDistance.getDistance(DistanceUnit.CM) == 0.0){
//            robot.forward();
//        }
//        robot.stop();
        var.POWER = 0.4;
        robot.forward();
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() <= 1.68)) {
            telemetry.addData("Off Lander", "Lowering down legs: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }
        robot.stop();
        var.POWER = 0.8;

        var.POWER = 0.1;
        robot.spinLeft();
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() <= 0.28)) {
            telemetry.addData("Off Lander", "Lowering down legs: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }
        robot.stop();
        var.POWER = 0.8;

        var.POWER = 0.1;
        robot.left();
        runtime.reset();
        while (opModeIsActive() && robot.sensorColor.alpha() * 8 != NaN) {
            telemetry.addData("Alpha", robot.sensorColor.alpha() * 8);
            telemetry.update();
        }
        robot.stop();
        var.POWER = 0.8;


        // Step through each leg of the path, ensuring that the Auto mode has not been stopped along the way

        //Drop arm

        //block or ball
        // or

        //Drop team flag


//        // Step 1:  Drive forward for 3 seconds
//        robot.forward();
//        runtime.reset();
//        while (opModeIsActive() && (runtime.seconds() < 3.0)) {
//            telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
//            telemetry.update();
//        }
//
//        // Step 2:  Spin right for 1.3 seconds
//        robot.spinLeft();
//        runtime.reset();
//        while (opModeIsActive() && (runtime.seconds() < 1.3)) {
//            telemetry.addData("Path", "Leg 2: %2.5f S Elapsed", runtime.seconds());
//            telemetry.update();
//        }
//
//        // Step 3:  Drive Backwards for 1 Second
//        robot.back();
//        runtime.reset();
//        while (opModeIsActive() && (runtime.seconds() < 1.0)) {
//            telemetry.addData("Path", "Leg 3: %2.5f S Elapsed", runtime.seconds());
//            telemetry.update();
//        }

        // Step 4:  Stop and close the claw.
        robot.stop();

        telemetry.addData("Path", "Complete");
        telemetry.update();
        sleep(1000);
    }
}
