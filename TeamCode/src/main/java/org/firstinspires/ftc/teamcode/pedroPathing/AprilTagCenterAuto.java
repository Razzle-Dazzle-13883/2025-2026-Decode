package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

import java.util.List;

@Autonomous(name = "AprilTagCenterAuto", group = "Turret")
public class AprilTagCenterAuto extends OpMode {

    private static final int TARGET_TAG_ID = 24;

    // ----------------- HARDWARE -----------------
    private DcMotor turret;

    // ----------------- VISION -------------------
    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;
    private AprilTagDetection targetTag;
    private boolean tagDetected = false;

    // ----------------- CONTROL TUNING -----------
    private static final double TURN_GAIN = 0.022;
    private static final double MAX_POWER = 0.45;
    private static final double CENTER_THRESHOLD = 1.2;
    private static final double SMOOTHING = 0.10;

    private double smoothedPower = 0;

    @Override
    public void init() {

        // turret motor (NO ENCODER)
        turret = hardwareMap.get(DcMotor.class, "turretMotor");
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Vision setup
        aprilTag = new AprilTagProcessor.Builder()
                .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                .setDrawTagOutline(true)
                .build();

        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTag)
                .build();

        telemetry.addLine("Turret tracker initialized.");
    }

    @Override
    public void loop() {

        detectTag();

        if (!tagDetected) {
            turret.setPower(0);
            telemetry.addLine("Searching for Tag " + TARGET_TAG_ID + "...");
            telemetry.update();
            return;
        }

        double yawError = targetTag.ftcPose.yaw;
        boolean centered = Math.abs(yawError) < CENTER_THRESHOLD;

        double desiredPower = yawError * TURN_GAIN;
        desiredPower = Range.clip(desiredPower, -MAX_POWER, MAX_POWER);

        smoothedPower =
                smoothedPower * (1 - SMOOTHING) + desiredPower * SMOOTHING;

        if (centered) {
            turret.setPower(0);
        } else {
            turret.setPower(smoothedPower);
        }

        telemetry.addData("Tag Detected", tagDetected);
        telemetry.addData("Tag ID", targetTag.id);
        telemetry.addData("Yaw Error (deg)", yawError);
        telemetry.addData("Turret Power", smoothedPower);
        telemetry.addData("Centered", centered ? "YES" : "NO");
        telemetry.update();
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