package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

public class Robot {
    DcMotor frontLeftMotor;
    DcMotor backLeftMotor;
    DcMotor frontRightMotor;
    DcMotor backRightMotor;
    DcMotor intakeMotor;
    DcMotor leftShooterMotor;
    DcMotor rightShooterMotor;

    Servo leftKicker;
    // CRServo rightKicker;
    // Servo adjustHood;

    IMU imu;

    // final int TICKS_PER_INCH = 45; // 11.87 in per rev; 537.7 ticks per rev; 537.7/11.87 ticks per inch

    /*
    double adjustHoodPos;
    final double HOODNEAR = 0.0;
    final double HOODFAR = 0.8;
     */

    // double kickerPos;
    final double KICKERUP = 0.85;
    final double KICKERDOWN = 0.50;

    double intakeSpeed;
    final double INTAKERUN = 0.4;
    final double INTAKEREVERSE = -0.4;
    final double INTAKESHOOTER = 1.0;
    final double INTAKESTOP = 0.0;
    final double INTAKEFAST = 0.7;

    double shooterSpeed;
    final double SHOOTERRUN = 0.8;
    final double SHOOTERCLOSE = 0.55;
    final double SHOOTERSTOP = 0.0;
    final double SHOOTERREVERSE = -0.5;

    int leftFrontPos = 0;
    int leftBackPos = 0;
    int rightFrontPos = 0;
    int rightBackPos = 0;

    private OpMode myOpMode;
    public Robot(OpMode opMode){
        this.myOpMode = opMode;
    }

    public void initHardware(){
        // Declare our motors
        // Make sure your ID's match your configuration
        frontLeftMotor = myOpMode.hardwareMap.dcMotor.get("frontLeftMotor");
        backLeftMotor = myOpMode.hardwareMap.dcMotor.get("backLeftMotor");
        frontRightMotor = myOpMode.hardwareMap.dcMotor.get("frontRightMotor");
        backRightMotor = myOpMode.hardwareMap.dcMotor.get("backRightMotor");
        intakeMotor = myOpMode.hardwareMap.dcMotor.get("intakeMotor");
        leftShooterMotor = myOpMode.hardwareMap.dcMotor.get("leftShooterMotor");
        rightShooterMotor = myOpMode.hardwareMap.dcMotor.get("rightShooterMotor");

        try {
            leftKicker = myOpMode.hardwareMap.get(Servo.class, "leftKicker");
        } catch (IllegalArgumentException e) {
            leftKicker = null;
            myOpMode.telemetry.addData("WARN", "Servo 'leftKicker' not found in configuration");
            myOpMode.telemetry.update();
        }

        // adjustHood = myOpMode.hardwareMap.servo.get("adjustHood");

        /*
        frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
         */

        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftShooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightShooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Reverse the right side motors. This may be wrong for your setup.
        // If your robot moves backwards when commanded to go forwards,
        // reverse the left side instead.
        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        
        // Reverse both shooter motors
        leftShooterMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        rightShooterMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // initIMU(); no need as we use Two Wheel + IMU for PR
    }

    public void robotMove(double frontLeftPower, double backLeftPower, double frontRightPower, double backRightPower){
        frontLeftMotor.setPower(frontLeftPower);
        backLeftMotor.setPower(backLeftPower);
        frontRightMotor.setPower(frontRightPower);
        backRightMotor.setPower(backRightPower);
    }

    public void robotBrakeBehavior(){
        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftShooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightShooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void drive(int lF, int lB, int rF, int rB, double speed) {

        leftFrontPos -= lF;
        leftBackPos -= lB;
        rightFrontPos -= rF;
        rightBackPos -= rB;

        frontLeftMotor.setTargetPosition(leftFrontPos);
        backLeftMotor.setTargetPosition(leftBackPos);
        frontRightMotor.setTargetPosition(rightFrontPos);
        backRightMotor.setTargetPosition(rightBackPos);

        frontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        double driftFactor = 1.042;
        frontLeftMotor.setPower(speed);
        backLeftMotor.setPower(speed);
        frontRightMotor.setPower(speed);
        backRightMotor.setPower(speed);

        // waitDrive();
    }

    /*
    private void waitDrive() {
        while (frontLeftMotor.isBusy() && frontRightMotor.isBusy() && backLeftMotor.isBusy() && backRightMotor.isBusy() && myOpMode.opModeIsActive()) ;
    }
     */

    public void intakeOn() {
        intakeMotor.setPower(INTAKERUN);
    }
    public void intakeOff() {
        intakeMotor.setPower(INTAKESTOP);
    }
    public void intakeRev() {
        intakeMotor.setPower(INTAKEREVERSE);
    }
    public void intakeShoot() {intakeMotor.setPower(INTAKESHOOTER); }
    public void intakeFast() {intakeMotor.setPower(INTAKEFAST); }

    public void kickerUp() { leftKicker.setPosition(KICKERUP);
    }
    public void kickerDown() { leftKicker.setPosition(KICKERDOWN); }

    public void shooterOn() { 
        leftShooterMotor.setPower(SHOOTERRUN);
        rightShooterMotor.setPower(SHOOTERRUN);
    }
    public void shooterClose() { 
        leftShooterMotor.setPower(SHOOTERCLOSE);
        rightShooterMotor.setPower(SHOOTERCLOSE);
    }
    public void shooterOff() { 
        leftShooterMotor.setPower(SHOOTERSTOP);
        rightShooterMotor.setPower(SHOOTERSTOP);
    }
    public void shooterRev() { 
        leftShooterMotor.setPower(SHOOTERREVERSE);
        rightShooterMotor.setPower(SHOOTERREVERSE);
    }
    
    /**
     * Get robot heading (yaw) from IMU in degrees
     * @return Robot heading in degrees (-180 to 180)
     */
    public double getRobotHeading() {
        if (imu != null) {
            YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
            return orientation.getYaw(AngleUnit.DEGREES);
        }
        return 0.0;
    }
    
    /**
     * Get robot angular velocity (rotation rate) in degrees per second
     * @return Robot yaw rotation rate in degrees/second (positive = counter-clockwise)
     */
    public double getRobotAngularVelocity() {
        if (imu != null) {
            return imu.getRobotAngularVelocity(AngleUnit.DEGREES).zRotationRate;
        }
        return 0.0;
    }

    private void initIMU() {
        // Retrieve the IMU from the hardware map
        imu = myOpMode.hardwareMap.get(IMU.class, "imu");
        // Adjust the orientation parameters to match your robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters);
    }
}