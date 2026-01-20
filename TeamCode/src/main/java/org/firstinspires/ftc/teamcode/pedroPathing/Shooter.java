package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.math.MathFunctions;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class Shooter {
    public static final double RPM = 5400.0; // 6000 RPM peak
    public static final double TICKS_PER_REV = 28.0;
    public static final double TICKS_PER_SECOND = (RPM / 60) * TICKS_PER_REV;

    private DcMotorEx leftShooterMotor;
    //private DcMotorEx rightShooterMotor;
    // for PIDFs tuning
    private static final double F = -27.3;
    private static final double P = 5.803;
    private  static final  double LOW_VELOCITY = 900;
    private  static final  double HIGH_VELOCITY = 1850;
    private OpMode myOpMode;
    private boolean isRedAlliance = false; // default to Blue
    private double distanceToGoalOffset = 8.0; // offset the measurment with PetraPathing X/Y
    public Shooter(OpMode opMode){
        this.myOpMode = opMode;
    }

    public void init() {
        leftShooterMotor = myOpMode.hardwareMap.get(DcMotorEx.class, "leftShooterMotor");
        //rightShooterMotor = myOpMode.hardwareMap.get(DcMotorEx.class, "rightShooterMotor");

        leftShooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //rightShooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftShooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //rightShooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Reverse both shooter motors
        leftShooterMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        //rightShooterMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P, 0, 0, F);
        this.setPIDFCoefficients(pidfCoefficients);
    }

    public void setVelocity(double velocity) {
        leftShooterMotor.setVelocity(velocity);
        // rightShooterMotor.setVelocity(velocity);
    }

    public double getLeftMotorVelocity() {
        return leftShooterMotor.getVelocity();
    }

    //public double getRightMotorVelocity() {
        //return rightShooterMotor.getVelocity();
   //}

    public void setPIDFCoefficients(PIDFCoefficients pidfCoefficients) {
        leftShooterMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        // rightShooterMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
    }
    public void setAlliance(boolean isRedAlliance) {
        this.isRedAlliance = isRedAlliance;
    }

    public void shooterIdle() {
        this.leftShooterMotor.setVelocity(LOW_VELOCITY);
        myOpMode.telemetry.addData("shooterIdle Velocity: ", LOW_VELOCITY);
    }
    public void shooterOn(double x, double y) {
        double distance = getDistanceToGoal(x, y);
        double targetVelocity = calculateVelocity(distance);

        myOpMode.telemetry.addData("Shooting Velocity: ", targetVelocity);

        this.leftShooterMotor.setVelocity(targetVelocity);
    }
    public double calculateVelocity(double distance) {
        double y = MathFunctions.clamp(
                0.00877849 * Math.pow(distance, 2) + 3.87391 * distance + 1100.8993,
                LOW_VELOCITY,
                HIGH_VELOCITY);
        return y;
    }

    public double getDistanceToGoal(double x, double y) {
        double goal_x = 14.0;
        double goal_y = 134.0;

        if (this.isRedAlliance) {
            goal_x = 129.0;
            goal_y = 133.0;
        }

        double dx = goal_x - x;
        double dy = goal_y - y;
        double distance = Math.sqrt(Math.pow(dx, 2) + Math.pow(dy, 2));

        myOpMode.telemetry.addLine("SHOOTER");
        myOpMode.telemetry.addData("getDistanceToGoal X:", x);
        myOpMode.telemetry.addData("Goal X:", goal_x);
        myOpMode.telemetry.addData("getDistanceToGoal Y:", y);
        myOpMode.telemetry.addData("Goal Y:", goal_y);
        myOpMode.telemetry.addData("Distance to goal: ", distance);
        myOpMode.telemetry.addData("Distance to goal + offset: ", distance - distanceToGoalOffset);

        return distance - distanceToGoalOffset;
    }
}
