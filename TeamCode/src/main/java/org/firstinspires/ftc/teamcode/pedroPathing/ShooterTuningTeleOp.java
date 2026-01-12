package org.firstinspires.ftc.teamcode.pedroPathing;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Configurable
@TeleOp(name = "Shooter")
public class ShooterTuningTeleOp extends OpMode {
    private Robot robot;
    private Shooter shooter;

    // public static fields should be configurable via Panels UI
    public static double highVelocity = 1500.0;
    public static double lowVelocity = 900.0;
    public static double curTargetVelocity = highVelocity; // best target velocity for certain distance to goal

    // for PIDFs tuning
    double F = 0;
    double P = 0;
    double[] stepSizes = {10.0, 1.0, 0.1, 0.01, 0.001};
    int stepIndex = 1;
    // end for PIDFs tuning

    @Override
    public void init() {
        robot = new Robot(this);
        robot.initHardware();

        shooter = new Shooter(this);
        shooter.init();

        // Uncomment when ready for PIDFs tuning
        // PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P, 0, 0, F);
        // shooter.setPIDFCoefficients(pidfCoefficients);
    }

    @Override
    public void loop() {
        telemetry.addData("Current Left Velocity", "%.2f", shooter.getLeftMotorVelocity());
        telemetry.addData("Current Right Velocity", "%.2f", shooter.getRightMotorVelocity());

        // Begin PIDFs tuning
        if (gamepad1.yWasPressed()) {
            if (curTargetVelocity == highVelocity) {
                curTargetVelocity = lowVelocity;
            } else {
                curTargetVelocity = highVelocity;
            }
        }

        if (gamepad1.bWasPressed()) {
            stepIndex = (stepIndex + 1) % stepSizes.length;
        }

        if (gamepad1.dpadLeftWasPressed()) {
            F -= stepSizes[stepIndex];
        }

        if (gamepad1.dpadUpWasPressed()) {
            F += stepSizes[stepIndex];
        }

        if (gamepad1.dpadUpWasPressed()) {
            P += stepSizes[stepIndex];
        }

        if (gamepad1.dpadDownWasPressed()) {
            P -= stepSizes[stepIndex];
        }

        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P, 0, 0, F);
        shooter.setPIDFCoefficients(pidfCoefficients);
        // End PIDFs tuning

        // this is also configurable via Panels UI
        // used by both tuning PIDFs and finding best velocity for distances
        shooter.setVelocity(curTargetVelocity);

        double curVelocity = shooter.getLeftMotorVelocity();
        double error = curTargetVelocity - curVelocity;

        telemetry.addData("Target Velocity", curTargetVelocity);
        telemetry.addData("Current Velocity", "%.2f", curVelocity);
        telemetry.addData("Error", "%.2f", error);
        telemetry.addData("Tuning P", "%.4f DPad U/D", P);
        telemetry.addData("Tuning F", "%.4f DPad L/R", F);
        telemetry.addData("Step Size", "%.4f B button", stepSizes[stepIndex]);

        telemetry.update();
    }
}
