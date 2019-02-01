/* Copyright (c) 2018 FIRST. All rights reserved.
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
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

/**
 * This 2018-2019 OpMode illustrates the basics of using the TensorFlow Object Detection API to
 * determine the position of the gold and silver minerals.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list.
 *
 * IMPORTANT: In order to use this OpMode, you need to obtain your own Vuforia license key as
 * is explained below.
 */
@Autonomous(name = "AutoBlueDepo")
@Disabled
public class AutoBlueDepo extends LinearOpMode {
    private static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    private static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    private static final String LABEL_SILVER_MINERAL = "Silver Mineral";
    private ElapsedTime mLandTime = new ElapsedTime();
    /*
     * IMPORTANT: You need to obtain your own license key to use Vuforia. The string below with which
     * 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and will not function.
     * A Vuforia 'Development' license key, can be obtained free of charge from the Vuforia developer
     * web site at https://developer.vuforia.com/license-manager.
     *
     * Vuforia license keys are always 380 characters long, and look as if they contain mostly
     * random data. As an example, here is a example of a fragment of a valid key:
     *      ... yIgIzTqZ4mWjk9wd3cZO9T1axEqzuhxoGlfOOI2dRzKS4T0hQ8kT ...
     * Once you've obtained a license key, copy the string from the Vuforia web site
     * and paste it in to your code on the next line, between the double quotes.
     */
    private static final String VUFORIA_KEY = "ATOrXR//////AAABmYBLizU5fEcvglm7irJ3z1ho4e/qAjqNcxmQRzrp4L/hQj+/fnCtmUnv+wI0th5S2ilk+oJIj5QgTJyllrTI1eog37UhiUkZhUIkitUV1Ysk5QUSAfFJ9K6Qjs1lkXoNqQKTsBSlFPWq2SxUE7NuXJdFODgAcEu+GxKbt4jD7O+DpUPrfcnfWFlDpQB+QeRXoVnA5CoxecgiJlFFwfVLv1enWNibPo+71MRljLon67ehX3X/bH+hdqUoD4yCqZwHgl7jDWOpM3xzdKvo2v7Jw4IR8hhxqQ+HhjGofi1DUWbR6ZGSwyLC6r37vDnLvhqeZ6Acsu0Dz4hu2mGItCpXe3wo5duzxKdqtAef5dSJBjGT";

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    private VuforiaLocalizer vuforia;

    /**
     * {@link #tfod} is the variable we will use to store our instance of the Tensor Flow Object
     * Detection engine.
     */
    private TFObjectDetector tfod;

    HardwareDrive robot = new HardwareDrive();

    @Override
    public void runOpMode() {
        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
        // first.
        initVuforia();
//        robot.init(hardwareMap, this, true, false, false);
//        robot.getDriveTrain().driveZeroPowerBrake();
//
//        telemetry.addData("Status", "Resetting Encoders");
//        telemetry.update();
//
//        robot.getDriveTrain().driveModeStopResetEncoders();
//        idle();
//
//        robot.getDriveTrain().driveModeWithEncoders();
//        robot.getDriveTrain().showDriveTelemetry();
//        telemetry.update();
//
//        robot.getGyro().init(hardwareMap, this, true, true, false);

        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            initTfod();
        } else {
            telemetry.addData("Sorry!", "This device is not compatible with TFOD");
        }

        /** Wait for the game to begin */
        telemetry.addData(">", "Press Play to start tracking");
        telemetry.update();
        waitForStart();

        mLandTime.reset();
        if (opModeIsActive()) {
            /** Activate Tensor Flow Object Detection. */
            if (tfod != null) {
                tfod.activate();
            }

            while (opModeIsActive()) {
                    if (mLandTime.time() < 10) {
                        //robot.getmLiftArm().up();
                    }else {
                        //robot.getmLiftArm().stop();
                        //robot.getGyro().start();
                        LocateTFMineral();
                    }
                }
            }

        if (tfod != null) {
            tfod.shutdown();
        }
    }

    /**
     * Initialize the Vuforia localization engine.
     */
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = CameraDirection.FRONT;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the Tensor Flow Object Detection engine.
    }

    /**
     * Initialize the Tensor Flow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
            "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_GOLD_MINERAL, LABEL_SILVER_MINERAL);
    }

    private void LocateTFMineral() {
        for (int x = 0; x < 1; x++) {
            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
            if (updatedRecognitions != null) {
                telemetry.addData("% Object Detected", updatedRecognitions);
                int goldMineralX = 0;
                if (updatedRecognitions.size() == 2) {
                    for (Recognition recognition : updatedRecognitions) {
                        if ((recognition.getLabel().equals(LABEL_GOLD_MINERAL)) && recognition.getTop() < 500) {
                            goldMineralX = (int) recognition.getLeft();
                        }
                        telemetry.addData("Gold Mineral Position", "%d", goldMineralX);
                    }
                    if ((goldMineralX >= 1) && (goldMineralX < 300)) {
                        //done!
                        telemetry.addData("Mineral Location", "Left");
                    } else if (goldMineralX > 300) {
                        //done!
                        telemetry.addData("Mineral Location", "Center");
                    } else {
                        //done!
                        telemetry.addData("Mineral Location", "Right");
                    }
                    telemetry.addData("X Position", goldMineralX);
                    telemetry.update();
                }
            }else {
                telemetry.addData("No", "TFOD");
            }
        }
    }
}