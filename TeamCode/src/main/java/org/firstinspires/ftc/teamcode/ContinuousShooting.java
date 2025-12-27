/* Copyright (c) 2021 FIRST. All rights reserved.
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
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.concurrent.TimeUnit;

/*
 * This file contains an example of a Linear "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When a selection is made from the menu, the corresponding OpMode is executed.
 *
 * This particular OpMode illustrates driving a 4-motor Omni-Directional (or Holonomic) robot.
 * This code will work with either a Mecanum-Drive or an X-Drive train.
 * Both of these drives are illustrated at https://gm0.org/en/latest/docs/robot-design/drivetrains/holonomic.html
 * Note that a Mecanum drive must display an X roller-pattern when viewed from above.
 *
 * Also note that it is critical to set the correct rotation direction for each motor.  See details below.
 *
 * Holonomic drives provide the ability for the robot to move in three axes (directions) simultaneously.
 * Each motion axis is controlled by one Joystick axis.
 *
 * 1) Axial:    Driving forward and backward               Left-joystick Forward/Backward
 * 2) Lateral:  Strafing right and left                     Left-joystick Right and Left
 * 3) Yaw:      Rotating Clockwise and counter clockwise    Right-joystick Right and Left
 *
 * This code is written assuming that the right-side motors need to be reversed for the robot to drive forward.
 * When you first test your robot, if it moves backward when you push the left stick forward, then you must flip
 * the direction of all 4 motors (see code below).
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@TeleOp(name="ContinuousShooting", group="Linear OpMode")
//@Disabled
public class ContinuousShooting extends LinearOpMode {

    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;

    private Servo clawServo = null;

    private int state = 0; // 0: stop_walking 1: start_walking
    private DcMotor intake = null;

    private final ElapsedTime wiggleTimer = new ElapsedTime();
    private final ElapsedTime shotTimer = new ElapsedTime();
    private final ElapsedTime feederTimer = new ElapsedTime();
    private final ElapsedTime driveTimer = new ElapsedTime();
    private final ElapsedTime correctingHeadingTimer = new ElapsedTime();
    private DcMotorEx launcher;
    private CRServo rightFeeder;
    private CRServo leftFeeder;
    private Servo blocker;
    private Servo angle;

    private enum LaunchState {
        IDLE,
        PREPARE,
        LAUNCH,
    }

    private enum WiggleState {
        WIGGLE_BACK,
        WIGGLE_FRONT
    }
    private WiggleState wiggleState;
    private enum WiggleControl {
        WIGGLE_LISTEN,
        WIGGLING
    }
    private WiggleControl wiggleControl;
    private enum LaunchControl {
        LISTEN,
        CORRECTING_HEADING,
        LAUNCHING,
    }
    private LaunchControl launchControl;
    private LaunchState launchState;
    final double WIGGLE_TIME = 1.0;
    final int LAUNCHER_TARGET_VELOCITY = 1175;
    final int LAUNCHER_MIN_VELOCITY = 1160;
    final double FEED_TIME = 0.125; //was 0.20
    final double TIME_BETWEEN_SHOTS = 2;
    private int targetVelocity = 1110;
    final double GOAL_ANGLE = 0.6;
    final double FAR_ANGLE = 1;
    final double BLOCKER_UP = 0;
    final double BLOCKER_DOWN = 1;


    final double K_RANGE = 3.06;
    final double K_PCT = 1080;
    double pctShoot = 1;
    final int TARGET_VELOCITY_GOAL = 1110;
    final int TARGET_VELOCITY_FAR = 1750;

    int launcherVelocity = 0;

    private static final boolean USE_WEBCAM = true;  // Set true to use a webcam, or false for a phone camera
    private static final int DESIRED_TAG_ID = -1;     // Choose the tag you want to approach or set to -1 for ANY tag.
    private VisionPortal visionPortal;               // Used to manage the video source.
    private AprilTagProcessor aprilTag;              // Used for managing the AprilTag detection process.
    private AprilTagDetection desiredTag = null;     // Used to hold the data for a detected AprilTag
    final double DESIRED_DISTANCE = 48.0; //  this is how close the camera should get to the target (inches)

    //  Set the GAIN constants to control the relationship between the measured position error, and how much power is
    //  applied to the drive motors to correct the error.
    //  Drive = Error * Gain    Make these values smaller for smoother control, or larger for a more aggressive response.
    final double SPEED_GAIN  =  0.02  ;   //  Forward Speed Control "Gain". e.g. Ramp up to 50% power at a 25 inch error.   (0.50 / 25.0)
    final double STRAFE_GAIN =  0.015 ;   //  Strafe Speed Control "Gain".  e.g. Ramp up to 37% power at a 25 degree Yaw error.   (0.375 / 25.0)
    final double TURN_GAIN   =  0.01  ;   //  Turn Control "Gain".  e.g. Ramp up to 25% power at a 25 degree error. (0.25 / 25.0)

    final double MAX_AUTO_SPEED = 0.5;   //  Clip the approach speed to this max value (adjust for your robot)
    final double MAX_AUTO_STRAFE= 0.5;   //  Clip the strafing speed to this max value (adjust for your robot)
    final double MAX_AUTO_TURN  = 0.3;   //  Clip the turn speed to this max value (adjust for your robot)
    double leftFrontPower;
    double rightFrontPower;
    double leftBackPower;
    double rightBackPower;

    public void goForwardFor2Secs() {
        runtime.reset();
        while (runtime.seconds() < 2) {
            // move forward
            leftFrontDrive.setPower(.2);
        }
    }
    boolean launch(boolean shotRequested){
        switch (launchState) {
            case IDLE:
                if (shotRequested) {
                    launchState = LaunchState.PREPARE;
                    shotTimer.reset();
                }
                break;
            case PREPARE:
                launcherVelocity = targetVelocity;
                if (launcher.getVelocity() > (targetVelocity*0.98)){
                    launchState = LaunchState.LAUNCH;
                    leftFeeder.setPower(1);
                    rightFeeder.setPower(1);
                    feederTimer.reset();
                }
                break;
            case LAUNCH:
                if (feederTimer.seconds() > FEED_TIME) {
                    leftFeeder.setPower(0);
                    rightFeeder.setPower(0);

                    if(shotTimer.seconds() > TIME_BETWEEN_SHOTS){
                        launchState = LaunchState.IDLE;
                        //launcher.setVelocity(0);
                        return true;
                    }
                }
        }
        return false;
    }

    boolean wiggleMachine() {
        switch (wiggleState) {
            case WIGGLE_BACK:
                if (wiggleTimer.seconds() > WIGGLE_TIME) {
                    // Always go back when wiggling
                    launcherVelocity = -LAUNCHER_TARGET_VELOCITY;
                    wiggleState =  WiggleState.WIGGLE_FRONT;
                    wiggleTimer.reset();
                    if (gamepad1.back) {
                        return true;
                    }
                }
                break;
            case WIGGLE_FRONT:
                if (wiggleTimer.seconds() > WIGGLE_TIME) {
                    launcherVelocity = -LAUNCHER_TARGET_VELOCITY;

                    wiggleState = WiggleState.WIGGLE_BACK;
                    wiggleTimer.reset();
                    if (gamepad1.back) {
                        return true;
                    }

                }
                break;
        }
        return false;
    }

    public void moveRobot(double axial, double lateral, double yaw) {

        // Combine the joystick requests for each axis-motion to determine each wheel's power.
        // Set up a variable for each drive wheel to save the power level for telemetry.

        leftFrontPower  = axial + lateral + yaw;
        rightFrontPower = axial - lateral - yaw;
        leftBackPower   = axial - lateral + yaw;
        rightBackPower  = axial + lateral - yaw;

        // Normalize the values so no wheel power exceeds 100%
        // This ensures that the robot maintains the desired motion.
        double max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));

        if (max > 1.0) {
            leftFrontPower  /= max;
            rightFrontPower /= max;
            leftBackPower   /= max;
            rightBackPower  /= max;
        }

            /*if (gamepad1.start) {
                runtime.reset();
                state = 1;
            }

            if (state == 1) {
                if (runtime.seconds() < 2) {
                    leftFrontPower = 0.2;
                    leftBackPower = 0.2;
                    rightFrontPower = 0.2;
                    rightBackPower = 0.2;
                } else {
                    leftFrontPower = 0.0;
                    leftBackPower = 0.0;
                    rightFrontPower = 0.0;
                    rightBackPower = 0.0;
                    state = 0;
                }
            }*/


        // This is test code:
        //
        // Uncomment the following code to test your motor directions.
        // Each button should make the corresponding motor run FORWARD.
        //   1) First get all the motors to take to correct positions on the robot
        //      by adjusting your Robot Configuration if necessary.
        //   2) Then make sure they run in the correct direction by modifying the
        //      the setDirection() calls above.
        // Once the correct motors move in the correct direction re-comment this code.


        //leftFrontPower  = gamepad1.x ? 1.0 : 0.0;  // X gamepad
        //leftBackPower   = gamepad1.a ? 1.0 : 0.0;  // A gamepad
        //rightFrontPower = gamepad1.y ? 1.0 : 0.0;  // Y gamepad
        //rightBackPower  = gamepad1.b ? 1.0 : 0.0;  // B gamepadhttps://research.google.com/colaboratory/faq.html


        // Send calculated power to wheels
        leftFrontDrive.setPower(leftFrontPower);
        rightFrontDrive.setPower(rightFrontPower);
        leftBackDrive.setPower(leftBackPower);
        rightBackDrive.setPower(rightBackPower);
    }

    private void initAprilTag() {
        // Create the AprilTag processor by using a builder.
        aprilTag = new AprilTagProcessor.Builder().build();

        // Adjust Image Decimation to trade-off detection-range for detection-rate.
        // e.g. Some typical detection data using a Logitech C920 WebCam
        // Decimation = 1 ..  Detect 2" Tag from 10 feet away at 10 Frames per second
        // Decimation = 2 ..  Detect 2" Tag from 6  feet away at 22 Frames per second
        // Decimation = 3 ..  Detect 2" Tag from 4  feet away at 30 Frames Per Second
        // Decimation = 3 ..  Detect 5" Tag from 10 feet away at 30 Frames Per Second
        // Note: Decimation can be changed on-the-fly to adapt during a match.
        aprilTag.setDecimation(3);

        // Create the vision portal by using a builder.
        if (USE_WEBCAM) {
            visionPortal = new VisionPortal.Builder()
                    .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                    .addProcessor(aprilTag)
                    .build();
        } else {
            visionPortal = new VisionPortal.Builder()
                    .setCamera(BuiltinCameraDirection.BACK)
                    .addProcessor(aprilTag)
                    .build();
        }
    }

    private void    setManualExposure(int exposureMS, int gain) {
        // Wait for the camera to be open, then use the controls

        if (visionPortal == null) {
            return;
        }

        // Make sure camera is streaming before we try to set the exposure controls
        if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            telemetry.addData("Camera", "Waiting");
            telemetry.update();
            while (!isStopRequested() && (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING)) {
                sleep(20);
            }
            telemetry.addData("Camera", "Ready");
            telemetry.update();
        }

        // Set camera controls unless we are stopping.
        if (!isStopRequested())
        {
            ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
            if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
                exposureControl.setMode(ExposureControl.Mode.Manual);
                sleep(50);
            }
            exposureControl.setExposure((long)exposureMS, TimeUnit.MILLISECONDS);
            sleep(20);
            GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
            gainControl.setGain(gain);
            sleep(20);
        }
    }

    @Override
    public void runOpMode() {

        boolean targetFound     = false;    // Set to true when an AprilTag target is detected


        // Initialize the Apriltag Detection process
        initAprilTag();

        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        leftFrontDrive  = hardwareMap.get(DcMotor.class, "left_front_drive");
        leftBackDrive  = hardwareMap.get(DcMotor.class, "left_back_drive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "right_back_drive");
        intake = hardwareMap.get(DcMotor.class, "intake");
        launcher = hardwareMap.get(DcMotorEx.class, "launcher");
        rightFeeder = hardwareMap.get(CRServo.class, "right_feeder");
        leftFeeder = hardwareMap.get(CRServo.class, "left_feeder");
        blocker = hardwareMap.get(Servo.class, "blocker");
        angle = hardwareMap.get(Servo.class, "angle");

        if (USE_WEBCAM)
            setManualExposure(6, 250);  // Use low exposure time to reduce motion blur


        // ########################################################################################
        // !!!            IMPORTANT Drive Information. Test your motor directions.            !!!!!
        // ########################################################################################
        // Most robots need the motors on one side to be reversed to drive forward.
        // The motor reversals shown here are for a "direct drive" robot (the wheels turn the same direction as the motor shaft)
        // If your robot has additional gear reductions or uses a right-angled drive, it's important to ensure
        // that your motors are turning in the correct direction.  So, start out with the reversals here, BUT
        // when you first test your robot, push the left joystick forward and observe the direction the wheels turn.
        // Reverse the direction (flip FORWARD <-> REVERSE ) of any wheel that runs backward
        // Keep testing until ALL the wheels move the robot forward when you push the left joystick forward.
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        leftFeeder.setDirection(DcMotorSimple.Direction.REVERSE);

        launchState = LaunchState.IDLE;
        launchControl = LaunchControl.LISTEN;
        wiggleState = WiggleState.WIGGLE_FRONT;
        wiggleControl = WiggleControl.WIGGLE_LISTEN;

        // Wait for the game to start (driver presses START)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();
        wiggleTimer.reset();

        // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
        double axial   =  0;  // Note: pushing stick forward gives negative value
        double lateral =  0;
        double yaw     =  0;


        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            double max;

            double intakePower = (gamepad1.left_trigger - gamepad1.right_trigger);
            if (intakePower > 0){
                blocker.setPosition(BLOCKER_UP);
                launcherVelocity = -100;
            }

            double feederPower = 0.0;

            targetFound = false;
            desiredTag  = null;

            // Step through the list of detected tags and look for a matching tag
            List<AprilTagDetection> currentDetections = aprilTag.getDetections();
            for (AprilTagDetection detection : currentDetections) {
                // Look to see if we have size info on this tag.
                if (detection.metadata != null) {
                    //  Check to see if we want to track towards this tag.
                    if ((DESIRED_TAG_ID < 0) || (detection.id == DESIRED_TAG_ID)) {
                        // Yes, we want to use this tag.
                        targetFound = true;
                        desiredTag = detection;
                        break;  // don't look any further.
                    } else {
                        // This tag is in the library, but we do not want to track it right now.
                        telemetry.addData("Skipping", "Tag ID %d is not desired", detection.id);
                    }
                } else {
                    // This tag is NOT in the library, so we don't have enough information to track to it.
                    telemetry.addData("Unknown", "Tag ID %d is not in TagLibrary", detection.id);
                }
            }

            // Tell the driver what we see, and what to do.
            if (targetFound) {
                telemetry.addData("\n>","HOLD Left-Bumper to Drive to Target\n");
                telemetry.addData("Found", "ID %d (%s)", desiredTag.id, desiredTag.metadata.name);
                telemetry.addData("Range",  "%5.1f inches", desiredTag.ftcPose.range);
                telemetry.addData("Bearing","%3.0f degrees", desiredTag.ftcPose.bearing);
                telemetry.addData("Yaw","%3.0f degrees", desiredTag.ftcPose.yaw);
            } else {
                telemetry.addData("\n>","Drive using joysticks to find valid target\n");
            }

            // If Left Bumper is being pressed, AND we have found the desired target, Drive to target Automatically .
            if (gamepad1.left_bumper && targetFound) {

                // Determine heading, range and Yaw (tag image rotation) error so we can use them to control the robot automatically.
                double rangeError = (desiredTag.ftcPose.range - DESIRED_DISTANCE);
                double headingError = desiredTag.ftcPose.bearing;
                double yawError = desiredTag.ftcPose.yaw;

                // Use the speed and turn "gains" to calculate how we want the robot to move.
                axial = Range.clip(rangeError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
                yaw = Range.clip(-headingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN);
                lateral = Range.clip(yawError * STRAFE_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);

                telemetry.addData("Auto", "Drive %5.2f, Strafe %5.2f, Turn %5.2f ", axial, lateral, yaw);


            } else {
                axial   = -gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
                lateral =  gamepad1.left_stick_x;
                yaw     =  gamepad1.right_stick_x;

            }


            // wiggle launch
            switch (wiggleControl) {
                case WIGGLE_LISTEN:
                    if (gamepad1.start) {
                        wiggleControl = WiggleControl.WIGGLING;
                    }
                    break;
                case WIGGLING:
                    boolean is_done_wiggling = wiggleMachine();
                    if (is_done_wiggling) {
                        launcherVelocity = 0;
                        wiggleControl = WiggleControl.WIGGLE_LISTEN;
                    }
                    break;

            }

            if (gamepad1.dpad_left)
                feederPower += 0.4;
            if (gamepad1.dpad_right)
                feederPower -= 0.4;

            if (gamepad1.a)
                blocker.setPosition(BLOCKER_UP);
            if (gamepad1.b)
                blocker.setPosition(BLOCKER_DOWN);



            if (gamepad1.dpad_up) {
                pctShoot += 0.01;
                sleep(20);
            }
            if (gamepad1.dpad_down) {
                pctShoot -= 0.01;
                sleep(20);
            }
            if (pctShoot > 2)
                pctShoot = 2;

            //left bumper for normal, close to the goal, right bumper when farther away



            switch (launchControl) {
                case LISTEN:
                    leftFeeder.setPower(feederPower);
                    rightFeeder.setPower(feederPower);
                    targetVelocity = LAUNCHER_TARGET_VELOCITY;
                    if (desiredTag != null){
                        targetVelocity = (int)(K_RANGE*desiredTag.ftcPose.range+K_PCT*pctShoot);
                    }


                    if (gamepad1.y) {
                        blocker.setPosition(BLOCKER_DOWN);
                        angle.setPosition(GOAL_ANGLE);

                        launchControl = LaunchControl.CORRECTING_HEADING;
                        correctingHeadingTimer.reset();
                    }
                    break;
                case CORRECTING_HEADING:
                    intakePower = 0.5;
                    if (desiredTag != null) {
                        double bearing = desiredTag.ftcPose.bearing;
                        yaw = Range.clip(-bearing * 0.02, -MAX_AUTO_TURN, MAX_AUTO_TURN);
                        if (((bearing < 1) && (bearing > -1))) {
                            launch(true);
                            launchControl = LaunchControl.LAUNCHING;
                        } else {
                            axial = 0;
                            lateral = 0;
                        }
                    }
                    if (gamepad1.x || correctingHeadingTimer.seconds() > 0.2){
                        launch(true);
                        launchControl = LaunchControl.LAUNCHING;
                    }
                    break;
                case LAUNCHING:
                    if(launch(false)) {
                        launchControl = LaunchControl.LISTEN;
                    }
                    break;
            }

            if ((axial  != 0) || (lateral != 0)|| (yaw != 0)) {
                state = 0;
            }


            if (gamepad1.back){
                launcherVelocity = 0;
            }

            moveRobot(axial, lateral, yaw);

            intake.setPower(intakePower);

            launcher.setVelocity(launcherVelocity);



            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
            telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftBackPower, rightBackPower);
            telemetry.addData("intake", intakePower);
            telemetry.addData("launch_state", launchState);
            telemetry.addData("launch_control", launchControl);
            telemetry.addData("launch_velocity","%4.2f", launcher.getVelocity());
            telemetry.addData("wiggle_state", wiggleState);
            telemetry.addData("wiggle_control", wiggleControl);
            telemetry.addData("Status", "Wiggle_Time: " + wiggleTimer.toString());
            telemetry.addData("V/pct:", "%d/%.2f" , targetVelocity, pctShoot);
            telemetry.update();
        }
    }}