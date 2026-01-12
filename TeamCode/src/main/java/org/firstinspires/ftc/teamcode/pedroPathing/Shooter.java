package org.firstinspires.ftc.teamcode.pedroPathing;

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
    private DcMotorEx rightShooterMotor;

    private OpMode myOpMode;
    public Shooter(OpMode opMode){
        this.myOpMode = opMode;
    }

    public void init() {
        leftShooterMotor = myOpMode.hardwareMap.get(DcMotorEx.class, "leftShooterMotor");
        rightShooterMotor = myOpMode.hardwareMap.get(DcMotorEx.class, "rightShooterMotor");

        leftShooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightShooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftShooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightShooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Reverse both shooter motors
        leftShooterMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        rightShooterMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void setVelocity(double velocity) {
        leftShooterMotor.setVelocity(velocity);
        rightShooterMotor.setVelocity(velocity);
    }

    public double getLeftMotorVelocity() {
        return leftShooterMotor.getVelocity();
    }

    public double getRightMotorVelocity() {
        return rightShooterMotor.getVelocity();
    }

    public void setPIDFCoefficients(PIDFCoefficients pidfCoefficients) {
        leftShooterMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        rightShooterMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
    }
}
