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

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This is NOT an opmode.
 *
 * This class can be used to define all the specific hardware for a single robot.
 * In this case that robot is a Pushbot.
 * See PushbotTeleopTank_Iterative and others classes starting with "Pushbot" for usage examples.
 *
 * This hardware class assumes the following device names have been configured on the robot:
 * Note:  All names are lower case and some have single spaces between words.
 *
 * Motor channel:  Left  drive motor:        "left_drive"
 * Motor channel:  Right drive motor:        "right_drive"
 * Motor channel:  Manipulator drive motor:  "left_arm"
 * Servo channel:  Servo to open left claw:  "left_hand"
 * Servo channel:  Servo to open right claw: "right_hand"
 */
public class HardwareDrive
{

    Varibles var = new Varibles();

    /* Public OpMode members. */
    public DcMotor  frontLeftMotor   = null;
    public DcMotor  frontRightMotor  = null;
    public DcMotor  backLeftMotor = null;
    public DcMotor  backRightMotor = null;
    public DcMotor  lift = null;
    public Servo    smite = null;
    public Servo    grabL = null;
    public Servo    grabR = null;

    /* local OpMode members. */
    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    public HardwareDrive(){

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors
        frontLeftMotor  = hwMap.get(DcMotor.class, "fL");
        frontRightMotor = hwMap.get(DcMotor.class, "fR");
        backLeftMotor    = hwMap.get(DcMotor.class, "bL");
        backRightMotor    = hwMap.get(DcMotor.class, "bR");
        lift            = hwMap.get(DcMotor.class,  "lift");

        // Set all motors to zero power
        frontLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backLeftMotor.setPower(0);
        backRightMotor.setPower(0);
        lift.setPower(0);

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Define and initialize ALL installed servo
        smite = hwMap.get(Servo.class, "slap");
        grabL = hwMap.get(Servo.class, "gL");
        grabR = hwMap.get(Servo.class, "gR");

    }
    public void drive(double forward, double side, double spin, double up, boolean grab){

        double frontLeftPower = forward*var.POWER -side*var.POWER + spin*var.POWER;
        double frontRightPower = -forward*var.POWER -side*var.POWER + spin*var.POWER;
        double backLeftPower = forward*var.POWER +side*var.POWER + spin*var.POWER;
        double backRightPower = -forward*var.POWER +side*var.POWER + spin*var.POWER;

        if(frontLeftPower > 1.0)
            frontLeftPower = 1.0;
        if(frontLeftPower < -1.0)
            frontLeftPower = -1.0;
        if(frontRightPower > 1.0)
            frontRightPower = 1.0;
        if(frontRightPower < -1.0)
            frontRightPower = -1.0;
        if(backLeftPower > 1.0)
            backLeftPower = 1.0;
        if(backLeftPower < -1.0)
            backLeftPower = -1.0;
        if(backRightPower > 1.0)
            backRightPower = 1.0;
        if(backRightPower < -1.0)
            backRightPower = -1.0;

        //for drive dircation
        frontLeftMotor.setPower(frontLeftPower);
        frontRightMotor.setPower(frontRightPower);
        backLeftMotor.setPower(backLeftPower);
        backRightMotor.setPower(backRightPower);

        //lift
        lift.setPower(up);

        //hand open or close
        if(grab == true){
            grabL.setPosition(-.2);
            grabR.setPosition(.35);
            smite.setPosition(1.1);

        }
        if(grab == false){
            grabL.setPosition(.2);
            grabR.setPosition(0);
            smite.setPosition(1.1);
        }                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                            
    }
    public void spinLeft(){

        frontLeftMotor.setPower(var.POWER);
        frontRightMotor.setPower(var.POWER);
        backLeftMotor.setPower(var.POWER);
        backRightMotor.setPower(var.POWER);

    }
    public void spinRight(){

        frontLeftMotor.setPower(-var.POWER);
        frontRightMotor.setPower(-var.POWER);
        backLeftMotor.setPower(-var.POWER);
        backRightMotor.setPower(-var.POWER);

    }
    public void back(){

        frontLeftMotor.setPower(var.POWER);
        frontRightMotor.setPower(-var.POWER);
        backLeftMotor.setPower(var.POWER);
        backRightMotor.setPower(-var.POWER);

    }
    public void forward(){

        frontLeftMotor.setPower(-var.POWER);
        frontRightMotor.setPower(var.POWER);
        backLeftMotor.setPower(-var.POWER);
        backRightMotor.setPower(var.POWER);

    }
    public void Left(){

        frontLeftMotor.setPower(-var.POWER);
        frontRightMotor.setPower(-var.POWER);
        backLeftMotor.setPower(var.POWER);
        backRightMotor.setPower(var.POWER);

    }
    public void Right(){

        frontLeftMotor.setPower(var.POWER);
        frontRightMotor.setPower(var.POWER);
        backLeftMotor.setPower(-var.POWER);
        backRightMotor.setPower(-var.POWER);

    }
    public void Stop(){
        frontLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backLeftMotor.setPower(0);
        backRightMotor.setPower(0);

    }
    public void Smite(){
        smite.setPosition(0);
    }
    public void Smote(){
        smite.setPosition(1.1);
    }
 }

