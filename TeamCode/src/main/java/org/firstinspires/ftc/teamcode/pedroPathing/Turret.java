package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

import java.util.List;

public class Turret {

    private static final int TARGET_TAG_ID = 24;

    // ----------------- HARDWARE -----------------
    private DcMotor turretMotor;

    // ----------------- VISION -------------------
    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;
    private AprilTagDetection targetTag;
    private boolean tagDetected = false;

    // ----------------- CONTROL TUNING -----------
    private static final double TURN_GAIN = 0.025;
    private static final double MAX_POWER = 0.45;
    private static final double CENTER_THRESHOLD = 1.2;
    private static final double SMOOTHING = 0.08;

    private double smoothedPower = 0;

    private OpMode myOpMode;

    public Turret(OpMode opMode) {
        this.myOpMode = opMode;
    }

    public void init() {
        // turret motor (NO ENCODER)
        turretMotor = myOpMode.hardwareMap.get(DcMotor.class, "turretMotor");
        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Vision setup
        aprilTag = new AprilTagProcessor.Builder()
                .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                .setDrawTagOutline(true)
                .build();
/*
        visionPortal = new VisionPortal.Builder()
                .setCamera(myOpMode.hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTag)
                .build();
 */
    }

    public void followTag() {
        this.detectTag();

        if (!tagDetected) {
            turretMotor.setPower(0);
            myOpMode.telemetry.addLine("Searching for Tag " + TARGET_TAG_ID + "...");
            myOpMode.telemetry.update();
            return;
        }

        double yawError = targetTag.ftcPose.yaw;
        boolean centered = Math.abs(yawError) < CENTER_THRESHOLD;

        double desiredPower = yawError * TURN_GAIN;
        desiredPower = Range.clip(desiredPower, -MAX_POWER, MAX_POWER);

        smoothedPower =
                smoothedPower * (1 - SMOOTHING) + desiredPower * SMOOTHING;

        if (centered) {
            turretMotor.setPower(0);
        } else {
            turretMotor.setPower(smoothedPower);
        }

        myOpMode.telemetry.addData("Tag Detected", tagDetected);
        myOpMode.telemetry.addData("Tag ID", targetTag.id);
        myOpMode.telemetry.addData("Yaw Error (deg)", yawError);
        myOpMode.telemetry.addData("Turret Power", smoothedPower);
        myOpMode.telemetry.addData("Centered", centered ? "YES" : "NO");
        myOpMode.telemetry.update();
    }

    public void turretTurnLeft() { turretMotor.setPower(-0.3); }
    public void turretTurnRight() { turretMotor.setPower(0.3); }
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
    private void detectTag() {
        tagDetected = false;
        targetTag = null;

        List<AprilTagDetection> detections = aprilTag.getDetections();
        for (AprilTagDetection d : detections) {
            if (d.id == TARGET_TAG_ID) {
                targetTag = d;
                tagDetected = true;
                return;
            }
        }
    }
}