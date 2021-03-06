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

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="Main Drive", group="Opmode")
//@Disabled
public class DriveMain extends LinearOpMode {

    // Declare OpMode members.
    HardwareDrive robot = new HardwareDrive();
    Varibles var = new Varibles();
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        var.armPosition = 0;
        var.grabLock = true;
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Setup a variable for each drive wheel to save power level for telemetry
            double forwardPower;
            double sidePower;
            double spinPower;
            double arm;
            double lowerArm;
            double strut;
            double slideToggle;
            boolean stopMoveDown;
            double armGrabberControl;
            boolean unlockGrabber;
            boolean armKnocker;
            boolean armLockForward;
            boolean armLockBackward;
            boolean panelForward;
            boolean panelBackward;
            boolean flagDroppping;
            boolean flagDroppeRaise;
            boolean slowdownTrigger;
            boolean grabTrigger;
            boolean strutUp;
            boolean strutDown;

            // POV Mode uses left stick to go forward, and right stick to turn.
            // - This uses basic math to combine motions and is easier to drive straight.
            forwardPower = gamepad1.left_stick_y;
            sidePower    = gamepad1.left_stick_x;
            spinPower    = gamepad1.right_stick_x;
            lowerArm     = -gamepad2.right_stick_y;
            slowdownTrigger = gamepad1.right_bumper;
            //strut
            strutUp     = gamepad1.dpad_up;
            strutDown   = gamepad1.dpad_down;
            slideToggle = gamepad1.right_trigger;
            stopMoveDown = gamepad1.left_bumper;
            //panel
            panelForward = gamepad2.y;
            panelBackward = gamepad2.x;
            //flag
            flagDroppping = gamepad1.dpad_down;
            flagDroppeRaise = gamepad1.dpad_up;
            //arm
            armKnocker = gamepad2.a;
            grabTrigger = gamepad2.right_bumper;
            armLockForward = gamepad2.dpad_up;
            armLockBackward = gamepad2.dpad_down;
            armGrabberControl = gamepad2.right_trigger;
            arm = gamepad2.right_stick_y;
            unlockGrabber = gamepad2.left_bumper;


            robot.drive(forwardPower, sidePower, spinPower, arm, slideToggle, panelForward, panelBackward, flagDroppping, flagDroppeRaise, slowdownTrigger, grabTrigger, strutUp, strutDown, lowerArm, armLockForward, armLockBackward, armGrabberControl, stopMoveDown, armKnocker, unlockGrabber);

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Motors", "left (%.2f), right (%.2f)");
            telemetry.addData("Status", "slideToggle"+slideToggle);
            telemetry.addData("Arm Motor Position", "Position: "+var.armPosition);
        }
    }
}


