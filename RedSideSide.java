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
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;
//Built using Team 11654 (Plus or Minos)'s quick autonomous creator
@Autonomous(name="RedSideSide", group="Test")
//@Disabled
public class RedSideSide extends LinearOpMode {

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

            // Send telemetry message to signify robot waiting;
            telemetry.addData("Status", "Ready to run");    
            telemetry.update();

            // Wait for the game to start (driver presses PLAY)
            waitForStart();
            robot.platIn();
            runtime.reset();
            while (opModeIsActive() && runtime.seconds() <= 2.4){

            }
            robot.platStop();

            robot.encoderStrut(-1840, false, 0);
            robot.hook.setPosition(1);
            runtime.reset();
            while (opModeIsActive() && runtime.seconds() <= 3){

            }
            robot.encoderStrut(2040, true, 3);
            robot.hook.setPosition(0);
            runtime.reset();
            while (opModeIsActive() && runtime.seconds() <= 3){

            }
            //begin generated code
                               

             robot.encoderDrive(89,940);
             robot.encoderDrive(164,560);
             robot.encoderDrive(343,590);
             robot.encoderDrive(268,450);
   
        }
    }
//160, 156 Angle: 225
//110, 183 Angle: 225
//162, 155 Angle: 225
//193, 188 Angle: 225