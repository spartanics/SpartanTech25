package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

public class LaunchPackage {
    final double K_RANGE = 3.06;
    final double K_PCT = 1080;
    private DcMotorEx launcher;
    private CRServo rightFeeder;
    private CRServo leftFeeder;
    private Servo blocker;
    private DcMotor intake = null;
    private enum LaunchControl {
        LISTEN,
        CORRECTING_HEADING,
        LAUNCHING,
    }
    private LaunchControl launchControl;
    private enum LaunchState {
        IDLE,
        PREPARE,
        LAUNCH,
    }
    private LaunchState launchState;

    double feederPower = 0.0;

    private Gamepad gamepad1 = null;
    private LinearOpMode opModeParent = null;
    int launcherVelocity = 0;

    final int LAUNCHER_TARGET_VELOCITY = 1250;
    final int LAUNCHER_MIN_VELOCITY = 1160;
    final double FEED_TIME = 0.115; //was 0.20
    final double TIME_BETWEEN_SHOTS = 2;
    final double TIME_BETWEEN_SHOTS_BLOCKER_UP = 0.5;
    private int targetVelocity = 1110;
    private final ElapsedTime shotTimer = new ElapsedTime();
    private final ElapsedTime feederTimer = new ElapsedTime();
    private final ElapsedTime correctingHeadingTimer = new ElapsedTime();

    final double BLOCKER_UP = 0;
    final double BLOCKER_DOWN = 1;
    double pctShoot = 1;


    public LaunchPackage(LinearOpMode opMode) {
        opModeParent = opMode;

        intake = opMode.hardwareMap.get(DcMotor.class, "intake");
        launcher = opMode.hardwareMap.get(DcMotorEx.class, "launcher");
        rightFeeder = opMode.hardwareMap.get(CRServo.class, "right_feeder");
        leftFeeder = opMode.hardwareMap.get(CRServo.class, "left_feeder");
        leftFeeder.setDirection(DcMotorSimple.Direction.REVERSE);

        blocker = opMode.hardwareMap.get(Servo.class, "blocker");

        gamepad1 = opMode.gamepad1;

        launchState = LaunchState.IDLE;
        launchControl = LaunchControl.LISTEN;

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
                if (launcher.getVelocity() > (targetVelocity*0.75)){
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

                    if(shotTimer.seconds() > TIME_BETWEEN_SHOTS_BLOCKER_UP){
                        blocker.setPosition(BLOCKER_UP);
                    }
                    if(shotTimer.seconds() > TIME_BETWEEN_SHOTS){
                        launchState = LaunchState.IDLE;
                        //launcher.setVelocity(0);
                        return true;
                    }
                }
        }
        return false;
    }

    public boolean isInCorrectingHeadingMode() {
        return launchControl == LaunchControl.CORRECTING_HEADING;
    }
    public void launchCheck(AprilTagDetection desiredTag) {
        if (gamepad1.dpad_up) {
            pctShoot += 0.01;
            opModeParent.sleep(20);
        }
        if (gamepad1.dpad_down) {
            pctShoot -= 0.01;
            opModeParent.sleep(20);
        }
        if (pctShoot > 2)
            pctShoot = 2;

        double intakePower = (gamepad1.left_trigger - gamepad1.right_trigger);

        if (gamepad1.dpad_left)
            feederPower += 0.4;
        if (gamepad1.dpad_right)
            feederPower -= 0.4;
        if (gamepad1.a)
            blocker.setPosition(BLOCKER_UP);
        if (gamepad1.b)
            blocker.setPosition(BLOCKER_DOWN);


        if (gamepad1.back){
            launcherVelocity = 0;
        }

        if (intakePower > 0){

            blocker.setPosition(BLOCKER_UP);
            if (launchState == LaunchState.IDLE){
                launcherVelocity = -100;
            }
        }

        switch (launchControl) {
            case LISTEN:
                leftFeeder.setPower(feederPower);
                rightFeeder.setPower(feederPower);
                if (desiredTag != null){
                    targetVelocity = (int)(1.15 * (K_RANGE * desiredTag.ftcPose.range + K_PCT * pctShoot)); //equation
                } else {
                    if (targetVelocity < LAUNCHER_TARGET_VELOCITY){
                        targetVelocity = LAUNCHER_TARGET_VELOCITY;
                    }
                }


                if (gamepad1.y) {
                    blocker.setPosition(BLOCKER_DOWN);

                    launchControl = LaunchControl.CORRECTING_HEADING;
                    correctingHeadingTimer.reset();
                }
                break;
            case CORRECTING_HEADING:
                intakePower = 0.5;
                if (desiredTag != null) {
                    double bearing = desiredTag.ftcPose.bearing;
                    double yawError = desiredTag.ftcPose.yaw;

                    if (((bearing < 1) && (bearing > -1)) && ((yawError < 1) && (yawError > -1))) {
                        launch(true);
                        launchControl = LaunchControl.LAUNCHING;
                    }
                }
                if (gamepad1.x || correctingHeadingTimer.seconds() > 0.3){
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

        intake.setPower(intakePower);
        launcher.setVelocity(launcherVelocity);

        opModeParent.telemetry.addData("launch_control", launchControl);
        opModeParent.telemetry.addData("launch_state", launchState);
        opModeParent.telemetry.addData("Vc/V/pct:", "%4.2f/%d/%.2f" , launcher.getVelocity(), targetVelocity, pctShoot);
        opModeParent.telemetry.addData("intake", intakePower);


    }
}
