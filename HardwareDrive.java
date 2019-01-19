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

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
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
    public DcMotor  armLeft = null;
    public DcMotor  armRight = null;
    //public DcMotor  armString = null;
    public DcMotor  strutLeft = null;
    public DcMotor  strutRight = null;
    public DcMotor midArm = null;
    public Servo hook = null;
    public Servo backHook = null;
    public Servo panelPush = null;
    public ColorSensor sensorColor = null;
    public DistanceSensor sensorDistance = null;

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
        armLeft    = hwMap.get(DcMotor.class, "aL");
        armRight    = hwMap.get(DcMotor.class, "aR");
        //armString    = hwMap.get(DcMotor.class, "aS");
        strutLeft = hwMap.get(DcMotor.class, "sL");
        strutRight = hwMap.get(DcMotor.class, "sR");
        //midArm = hwMap.get(DcMotor.class, "mA");
        hook = hwMap.get(Servo.class, "hook");
        backHook = hwMap.get(Servo.class, "backHook");
        panelPush = hwMap.get(Servo.class, "panelPush");
        sensorColor = hwMap.get(ColorSensor.class, "sensor_color_distance");
        sensorDistance = hwMap.get(DistanceSensor.class, "sensor_color_distance");

        // Set all motors to zero power
        frontLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backLeftMotor.setPower(0);
        backRightMotor.setPower(0);
        armLeft.setPower(0);
        armRight.setPower(0);
        //armString.setPower(0);
        strutRight.setPower(0);
        strutLeft.setPower(0);

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        armLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        armRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //armString.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        strutLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        strutRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        // Define and initialize ALL installed servo
        /*smite = hwMap.get(Servo.class, "slap");
        grabL = hwMap.get(Servo.class, "gL");
        grabR = hwMap.get(Servo.class, "gR");*/

    }
    /*public void drive(double forward, double side, double spin, double up, boolean grab){

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
    }*/
    public void drive(double forward, double side, double spin, double arm, double strut, boolean hookF, boolean hookB, boolean panelForward, boolean panelBackward){

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


        if(arm > var.ARMP)
            arm = var.ARMP;
        if(arm < -var.ARMP)
            arm = -var.ARMP;

        if (hookF){
            hook.setPosition(1);
        }
        else if (hookB){
            hook.setPosition(0);
        }
        else {
            hook.setPosition(0.5);
        }

        if (panelForward){
            panelPush.setPosition(1);
        }
        else if (panelBackward){
            panelPush.setPosition(0);
        }
        else {
            panelPush.setPosition(0.5);
        }

        //for drive direction
        frontLeftMotor.setPower(frontLeftPower);
        frontRightMotor.setPower(frontRightPower);
        backLeftMotor.setPower(backLeftPower);
        backRightMotor.setPower(backRightPower);
        armLeft.setPower(arm);
        armRight.setPower(arm);
        //armString.setPower(arm);
        strutLeft.setPower(strut);
        strutRight.setPower(strut);
    }
    public void spinRight(){

        frontLeftMotor.setPower(var.POWER);
        frontRightMotor.setPower(var.POWER);
        backLeftMotor.setPower(var.POWER);
        backRightMotor.setPower(var.POWER);

    }
    public void spinLeft(){

        frontLeftMotor.setPower(-var.POWER);
        frontRightMotor.setPower(-var.POWER);
        backLeftMotor.setPower(-var.POWER);
        backRightMotor.setPower(-var.POWER);

    }
    public void forward(){

        frontLeftMotor.setPower(var.POWER);
        frontRightMotor.setPower(-var.POWER);
        backLeftMotor.setPower(var.POWER);
        backRightMotor.setPower(-var.POWER);

    }
    public void back(){

        frontLeftMotor.setPower(-var.POWER);
        frontRightMotor.setPower(var.POWER);
        backLeftMotor.setPower(-var.POWER);
        backRightMotor.setPower(var.POWER);

    }
    public void left(){
        frontLeftMotor.setPower(-var.POWER);
        frontRightMotor.setPower(-var.POWER);
        backLeftMotor.setPower(var.POWER);
        backRightMotor.setPower(var.POWER);
    }
    public void right(){
        frontLeftMotor.setPower(var.POWER);
        frontRightMotor.setPower(var.POWER);
        backLeftMotor.setPower(-var.POWER);
        backRightMotor.setPower(-var.POWER);
    }
    public void stop(){
        frontLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backLeftMotor.setPower(0);
        backRightMotor.setPower(0);
        strutRight.setPower(0);
        strutLeft.setPower(0);

    }
    public void strutLiftUp(){
        strutLeft.setPower(var.POWER);
        strutRight.setPower(var.POWER);
    }
    public void strutLowerDown(){
        strutRight.setPower(-var.POWER);
        strutLeft.setPower(-var.POWER);
    }
    public void landerHookClose(){
        hook.setPosition(1);
    }
    public void landerHookRest(){
        hook.setPosition(.5);
    }
    public void landerHookOpen(){
        hook.setPosition(0);
    }
    public void backHookOpen(){
        backHook.setPosition(.1);
    }
    public void backhookClose(){
        backHook.setPosition(0);
    }
 }

 