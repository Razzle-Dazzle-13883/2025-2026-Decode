package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;
import com.pedropathing.follower.Follower;

import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

import java.util.List;
import java.util.concurrent.TimeUnit;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;

public class Turret {

    private static final int TARGET_TAG_ID = 24;

    // ----------------- HARDWARE -----------------
    private DcMotor turret;
    private Robot robot;
    private Follower follower;

    // ----------------- VISION -------------------
    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;
    private AprilTagDetection targetTag;
    private boolean tagDetected = false;

    // ----------------- CONTROL TUNING -----------
    // GEAR RATIO: 121 (turret) : 47 (motor)
    private static final double TURN_GAIN = 0.02;  // Proportional gain for heading error
    private static final double MAX_POWER = 0.3;   // Maximum turret power
    private static final double CENTER_THRESHOLD = 1.0;  // Consider centered within this threshold (degrees)
    private static final double SMOOTHING = 0.1;   // Power smoothing factor (lower = smoother)
    
    // Velocity feedforward to compensate for robot rotation while moving
    private static final double VELOCITY_FEEDFORWARD_GAIN = 0.12;  // Compensate for robot rotation
    
    // State variables
    private double smoothedPower = 0.0;
    private double filteredRobotVelocity = 0.0;

    private OpMode myOpMode;

    public Turret(OpMode opMode) {
        this.myOpMode = opMode;
    }
    
    public void setRobot(Robot robot) {
        this.robot = robot;
    }
    
    public void setFollower(Follower follower) {
        this.follower = follower;
    }

    public void init() {
        // turret motor with encoder
        turret = myOpMode.hardwareMap.get(DcMotor.class, "turretMotor");
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // Stop using encoder mode as per user request
        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Vision setup with optimized settings
        aprilTag = new AprilTagProcessor.Builder()
                .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                .setDrawTagOutline(true)
                .build();
        
        // Set decimation for better performance (2 = good balance of range and speed)
        // Lower = better range but slower, Higher = faster but shorter range
        aprilTag.setDecimation(2);

        visionPortal = new VisionPortal.Builder()
                .setCamera(myOpMode.hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTag)
                .build();

        // Manual exposure settings to reduce motion blur (from standard FTC samples)
        setManualExposure(6, 250); 
    }

    public void followTag() {
        this.detectTag();

        if (!tagDetected) {
            // No tag detected - stop turret
            turret.setPower(0);
            smoothedPower = 0.0;
            myOpMode.telemetry.addLine("Searching for Tag " + TARGET_TAG_ID + "...");
            myOpMode.telemetry.update();
            return;
        }

        // CORRECTED: Use 'bearing' (angle to target), NOT 'yaw' (target orientation).
        // Bearing: Positive = Target is to the LEFT. Negative = Target is to the RIGHT.
        double headingError = targetTag.ftcPose.bearing;
        boolean centered = Math.abs(headingError) < CENTER_THRESHOLD;

        // Calculate proportional control
        // If bearing is Positive (Left), we want to turn Left.
        // IF Positive Power = Turn Right (common), then we need Negative Power.
        // CHECK THIS: If it goes away from target, flip this sign!
        double desiredPower = -headingError * TURN_GAIN;
        
        // Get robot angular velocity for feedforward compensation (when robot rotates while tracking)
        double velocityFeedforward = 0.0;
        if (robot != null) {
            double robotAngularVelocity = robot.getRobotAngularVelocity(); // deg/sec
            // Filter robot velocity to reduce noise
            filteredRobotVelocity = filteredRobotVelocity * 0.6 + robotAngularVelocity * 0.4;
            
            // Compensate for robot rotation
            velocityFeedforward = filteredRobotVelocity * VELOCITY_FEEDFORWARD_GAIN;
        }
        
        // Combine proportional control with velocity feedforward
        desiredPower = desiredPower + velocityFeedforward;
        
        // Clip to max power
        desiredPower = Range.clip(desiredPower, -MAX_POWER, MAX_POWER);

        // Apply smoothing for smooth movement (like samples)
        smoothedPower = smoothedPower * (1.0 - SMOOTHING) + desiredPower * SMOOTHING;

        // Stop if centered, otherwise apply power
        if (centered) {
            turret.setPower(0);
            smoothedPower = 0.0;
        } else {
            turret.setPower(smoothedPower);
        }

        // Simple telemetry
        myOpMode.telemetry.addData("Tag Detected", "YES (ID " + targetTag.id + ")");
        myOpMode.telemetry.addData("Bearing (Error)", "%.2f deg", headingError);
        myOpMode.telemetry.addData("Target Dir", headingError > 0 ? "LEFT" : "RIGHT");
        myOpMode.telemetry.addData("Turret Power", "%.3f", smoothedPower);
        myOpMode.telemetry.addData("Centered", centered ? "YES" : "NO");
        if (robot != null) {
            myOpMode.telemetry.addData("Robot Vel", "%.1f deg/s", filteredRobotVelocity);
        }
        myOpMode.telemetry.update();
    }

    private void detectTag() {
        tagDetected = false;
        targetTag = null;

        List<AprilTagDetection> detections = aprilTag.getDetections();
        for (AprilTagDetection d : detections) {
            if (d.metadata != null) { // Check if metadata is available (tag is in library)
                 if (d.id == TARGET_TAG_ID) {
                    targetTag = d;
                    tagDetected = true;
                    return;
                }
            }
        }
    }

    /*
     Manually set the camera gain and exposure.
     This can only be called AFTER calling initAprilTag(), and only works for Webcams;
    */
    private void setManualExposure(int exposureMS, int gain) {
        // Wait for the camera to be open, then use the controls

        if (visionPortal == null) {
            return;
        }

        // Make sure camera is streaming before we try to set the exposure controls
        if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            // Check quickly if streaming, otherwise just return to avoid blocking init
            // In a real opmode, you might want to wait, but for safety here we'll just try
             return;
        }

        // Set camera controls
        ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
        if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
            exposureControl.setMode(ExposureControl.Mode.Manual);
            try {
                Thread.sleep(50);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
        exposureControl.setExposure((long)exposureMS, TimeUnit.MILLISECONDS);
        try {
            Thread.sleep(20);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }   
        GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
        gainControl.setGain(gain);
        try {
            Thread.sleep(20);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }
}