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
    DcMotor turretMotor;

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
    final double KICKERUP = 0.5;
    final double KICKERDOWN = 0.15;

    double intakeSpeed;
    final double INTAKERUN = -0.4;
    final double INTAKEREVERSE = 0.4;
    final double INTAKESHOOTER = -0.6;
    final double INTAKESTOP = 0.0;
    final double INTAKEFAST = -1.0;
    final double INTAKESLOW = -0.2;

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
        turretMotor = myOpMode.hardwareMap.dcMotor.get("turretMotor");


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
        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Reverse the right side motors. This may be wrong for your setup.
        // If your robot moves backwards when commanded to go forwards,
        // reverse the left side instead.
        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        
        // Reverse both shooter motors
        leftShooterMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        rightShooterMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        initIMU();
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
    public void intakeSlow() {intakeMotor.setPower(INTAKESLOW); }

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

    public void turretTurnLeft() { turretMotor.setPower(-0.5); }
    public void turretTurnRight() { turretMotor.setPower(0.5); }
    public void turretStop() { turretMotor.setPower(0.0); }
    
    /**
     * Set turret power with proportional control
     * @param power Power value between -1.0 and 1.0
     */
    public void turretSetPower(double power) {
        turretMotor.setPower(power);
    }
    
    /**
     * Get turret position in degrees relative to robot
     * Uses encoder if available, otherwise returns 0
     * @return Turret position in degrees (0 = forward relative to robot)
     */
    public double getTurretPositionDegrees() {
        // Gear ratio: 121 teeth (turret) / 47 teeth (motor) = 2.5745 motor rotations per 360° turret rotation
        // Most FTC motors have 28 counts per revolution (REV HD Hex)
        // Adjust MOTOR_COUNTS_PER_REVOLUTION if using different motor
        final double MOTOR_COUNTS_PER_REVOLUTION = 28.0;
        final double TURRET_GEAR_TEETH = 121.0;
        final double MOTOR_GEAR_TEETH = 47.0;
        final double TURRET_GEAR_RATIO = TURRET_GEAR_TEETH / MOTOR_GEAR_TEETH; // 2.5745
        final double TURRET_DEGREES_PER_MOTOR_ROTATION = 360.0 / TURRET_GEAR_RATIO; // ~139.84 degrees
        final double TURRET_DEGREES_PER_ENCODER_TICK = TURRET_DEGREES_PER_MOTOR_ROTATION / MOTOR_COUNTS_PER_REVOLUTION; // ~4.994 degrees per tick
        
        int encoderTicks = turretMotor.getCurrentPosition();
        return encoderTicks * TURRET_DEGREES_PER_ENCODER_TICK;
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