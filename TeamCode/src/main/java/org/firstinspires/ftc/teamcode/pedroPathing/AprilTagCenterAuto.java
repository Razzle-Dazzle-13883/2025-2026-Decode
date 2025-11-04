package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.concurrent.TimeUnit;

import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This autonomous OpMode centers the robot on AprilTag ID 21 using PedroPathing.
 * The robot will continuously adjust its position to keep the AprilTag centered in the camera view.
 * 
 * Based on external samples but integrated with PedroPathing follower system.
 */
@Autonomous(name = "AprilTag Center Auto", group = "PedroPathing")
public class AprilTagCenterAuto extends OpMode {
    
    // AprilTag settings
    private static final boolean USE_WEBCAM = true;  // Set to false to use phone camera
    private static final int TARGET_TAG_ID = 21;     // The AprilTag ID we want to center on
    
    // Control gains - adjust these for smoother/more aggressive movement
    private static final double TURN_GAIN = 0.012;    // Increased for more responsive movement
    
    // Maximum movement speeds - NOT USED for translation, only rotation
    private static final double MAX_AUTO_TURN = 0.3;    // Increased maximum turn speed for better responsiveness
    
    // Smoothing variables for rotation
    private double smoothedTurnRate = 0.0;
    private double currentTurnRate = 0.0;
    private static final double TURN_SMOOTHING = 0.25; // Increased for more responsive but still smooth movement
    
    // Centering thresholds - how close to center before stopping movement
    private static final double CENTER_THRESHOLD_X = 0.1; // Inches from center horizontally
    private static final double CENTER_THRESHOLD_Y = 0.1; // Inches from center vertically
    private static final double CENTER_THRESHOLD_HEADING = 1.0; // Tightened threshold for more precise centering
    
    // PedroPathing components
    private Follower follower;
    private Robot robot;
    private Timer pathTimer, actionTimer, opmodeTimer;
    
    // Vision components
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;
    private AprilTagDetection targetTag = null;
    
    // Control variables
    private boolean tagDetected = false;
    private double targetX = 0.0;
    private double targetY = 0.0;
    private double targetHeading = 0.0;
    
    // State machine variables
    private int pathState;
    
    /**
     * Helper method to sleep for a specified number of milliseconds.
     * Only available in LinearOpMode, so we implement our own version.
     */
    private void sleep(long milliseconds) {
        ElapsedTime elapsedTime = new ElapsedTime();
        elapsedTime.reset();
        while (elapsedTime.milliseconds() < milliseconds) {
            // Just wait
        }
    }
    
    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();
        
        robot = new Robot(this);
        robot.initHardware();
        
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose()); // Start at origin
        
        // Initialize AprilTag detection
        initAprilTag();
        
        // Verify Robot hardware is properly configured
        verifyRobotConfiguration();
        
        pathState = 0;
    }
    
    @Override
    public void init_loop() {
        telemetry.addData("Status", "Initialized - Ready to center on AprilTag ID " + TARGET_TAG_ID);
        telemetry.addData("Camera", USE_WEBCAM ? "Webcam" : "Phone Camera");
        telemetry.addData(">", "Touch START to begin centering");
        telemetry.update();
        
        follower.update();
    }
    
    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
        
        // Ensure Robot hardware is properly configured for autonomous driving
        robot.robotBrakeBehavior();
    }
    
    @Override
    public void loop() {
        // These loop the movements of the robot, these must be called continuously in order to work
        follower.update();
        autonomousPathUpdate();
        
        // Feedback to Driver Hub for debugging
        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        
        // Robot status
        telemetry.addData("Robot Status", "Hardware Initialized");
        telemetry.addData("Follower Status", follower.isBusy() ? "Following Path" : "Idle");
        
        // Camera and AprilTag detection info
        if (visionPortal != null) {
            telemetry.addData("Camera State", visionPortal.getCameraState().toString());
        } else {
            telemetry.addData("Camera State", "NOT INITIALIZED");
        }
        
        telemetry.addData("AprilTag Detected", tagDetected ? "YES" : "NO");
        if (tagDetected && targetTag != null) {
            telemetry.addData("Tag ID", targetTag.id);
            telemetry.addData("Tag Yaw Error", "%.2f deg", targetHeading);
            telemetry.addData("Turn Rate", "%.3f", currentTurnRate);
            telemetry.addData("Centered", isCentered() ? "YES" : "NO");
        } else if (visionPortal != null && visionPortal.getCameraState() == VisionPortal.CameraState.STREAMING) {
            telemetry.addData("Status", "Camera ready - searching for AprilTag ID " + TARGET_TAG_ID);
        }
        
        telemetry.update();
    }
    
    @Override
    public void stop() {
        // Clean up vision portal
        if (visionPortal != null) {
            visionPortal.close();
        }
    }
    
    /**
     * Main autonomous path update method - uses continuous control like FieldTeleOp for smooth movement
     */
    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                // Start teleop-style continuous drive for smooth control
                follower.startTeleopDrive();
                setPathState(1);
                break;
            case 1:
                // Continuously center on AprilTag using smooth teleop-style control
                detectAprilTag();
                
                if (tagDetected && targetTag != null) {
                    calculateCenteringMovement();
                    
                    // Check if we're centered
                    if (isCentered()) {
                        telemetry.addData("Status", "CENTERED on AprilTag ID " + TARGET_TAG_ID);
                        follower.setTeleOpDrive(0, 0, 0, true);
                        // Stay in state 1 to continue tracking
                    } else {
                        // Apply smooth continuous movement
                        applyContinuousCentering();
                    }
                } else {
                    telemetry.addData("Status", "Searching for AprilTag ID " + TARGET_TAG_ID);
                    follower.setTeleOpDrive(0, 0, 0, true); // Stop until found
                    // Stay in state 1 to continue searching
                }
                break;
        }
    }
    
    /**
     * Apply smooth continuous movement to center on AprilTag - ONLY ROTATION
     */
    private void applyContinuousCentering() {
        // Calculate desired turn rate based on AprilTag yaw error
        // AprilTag ftcPose.yaw: positive = tag rotated LEFT relative to camera -> robot should turn RIGHT
        // So we need to negate the yaw to correct the heading
        // But if the robot is still turning away, we need to flip the sign again
        
        double desiredTurn = Range.clip(targetHeading * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN);
        
        // Apply exponential smoothing to the turn rate for smooth rotation
        // This creates a gradual ramp-up and ramp-down effect
        currentTurnRate = currentTurnRate * (1 - TURN_SMOOTHING) + desiredTurn * TURN_SMOOTHING;
        
        // Apply smoothed turn using PedroPathing teleop drive
        // Only rotation, no forward/back or strafe movement
        follower.setTeleOpDrive(0, 0, currentTurnRate, true);
    }
    /**
     * Verify that Robot hardware is properly configured for autonomous driving
     */
    private void verifyRobotConfiguration() {
        // Check if drive motors are properly initialized
        if (robot.frontLeftMotor == null || robot.backLeftMotor == null || 
            robot.frontRightMotor == null || robot.backRightMotor == null) {
            telemetry.addData("ERROR", "Drive motors not properly initialized!");
            telemetry.update();
        } else {
            telemetry.addData("Robot Config", "Drive motors initialized successfully");
        }
        
        // Check if IMU is properly initialized
        if (robot.imu == null) {
            telemetry.addData("ERROR", "IMU not properly initialized!");
            telemetry.update();
        } else {
            telemetry.addData("Robot Config", "IMU initialized successfully");
        }
    }
    
    /**
     * These change the states of the paths and actions. It will also reset the timers of the individual switches
     */
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }
    
    /**
     * Initialize the AprilTag processor and vision portal
     */
    private void initAprilTag() {
        try {
            // Create the AprilTag processor
            aprilTag = new AprilTagProcessor.Builder()
                    .setDrawAxes(false)
                    .setDrawCubeProjection(false)
                    .setDrawTagOutline(true)
                    .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                    .build();
            
            // Create the vision portal
            VisionPortal.Builder builder = new VisionPortal.Builder();
            
            // Set the camera with error checking
            if (USE_WEBCAM) {
                try {
                    WebcamName webcam = hardwareMap.get(WebcamName.class, "Webcam 1");
                    builder.setCamera(webcam);
                    telemetry.addData("Camera", "Webcam 1 initialized successfully");
                } catch (Exception e) {
                    telemetry.addData("ERROR", "Webcam 1 not found! Check hardware configuration.");
                    telemetry.addData("Camera", "Falling back to phone camera");
                    builder.setCamera(BuiltinCameraDirection.BACK);
                }
            } else {
                builder.setCamera(BuiltinCameraDirection.BACK);
                telemetry.addData("Camera", "Phone camera initialized");
            }
            
            // Add the processor and build
            builder.addProcessor(aprilTag);
            visionPortal = builder.build();
            
            // Wait for camera to be ready and optimize settings
            waitForCameraReady();
            
            // Optimize camera settings for better detection
            if (USE_WEBCAM) {
                setManualExposure(5, 250); // Low exposure, high gain for motion blur reduction
            }
            
            telemetry.addData("AprilTag", "Vision system initialized successfully");
            
        } catch (Exception e) {
            telemetry.addData("ERROR", "Failed to initialize AprilTag system: " + e.getMessage());
            aprilTag = null;
            visionPortal = null;
        }
    }
    
    /**
     * Wait for camera to be ready for detection
     */
    private void waitForCameraReady() {
        if (visionPortal == null) return;
        
        int timeout = 0;
        while (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING && timeout < 100) {
            sleep(50);
            timeout++;
        }
        
        if (visionPortal.getCameraState() == VisionPortal.CameraState.STREAMING) {
            telemetry.addData("Camera Status", "Ready for detection");
        } else {
            telemetry.addData("Camera Status", "Not ready - may affect detection");
        }
    }
    
    /**
     * Detect the target AprilTag
     */
    private void detectAprilTag() {
        // Check if camera system is properly initialized
        if (aprilTag == null || visionPortal == null) {
            tagDetected = false;
            targetTag = null;
            return;
        }
        
        // Check if camera is streaming
        if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            tagDetected = false;
            targetTag = null;
            return;
        }
        
        try {
            List<AprilTagDetection> currentDetections = aprilTag.getDetections();
            tagDetected = false;
            targetTag = null;
            
            // Look for our target tag
            for (AprilTagDetection detection : currentDetections) {
                if (detection.id == TARGET_TAG_ID) {
                    targetTag = detection;
                    tagDetected = true;
                    break;
                }
            }
        } catch (Exception e) {
            // If detection fails, reset state
            tagDetected = false;
            targetTag = null;
        }
    }
    
    /**
     * Calculate the movement needed to center the AprilTag
     */
    private void calculateCenteringMovement() {
        if (targetTag != null && targetTag.metadata != null) {
            // Get tag position relative to camera
            targetX = targetTag.ftcPose.x;      // Left/right offset
            targetY = targetTag.ftcPose.y;      // Forward/backward distance
            targetHeading = targetTag.ftcPose.yaw; // Rotation offset
        }
    }
    
    /**
     * Check if the robot is centered on the AprilTag (only checks rotation)
     */
    private boolean isCentered() {
        // Only check if the yaw error is within threshold since we're only rotating
        return Math.abs(targetHeading) < CENTER_THRESHOLD_HEADING;
    }
    
    /**
     * Set manual camera exposure for better AprilTag detection
     */
    private void setManualExposure(int exposureMS, int gain) {
        if (visionPortal == null) return;
        
        // Wait for camera to be ready
        if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            int timeout = 0;
            while ((visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) && timeout < 50) {
                sleep(20);
                timeout++;
            }
        }
        
        // Set exposure
        try {
            ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
            if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
                exposureControl.setMode(ExposureControl.Mode.Manual);
                sleep(50);
            }
            exposureControl.setExposure((long) exposureMS, TimeUnit.MILLISECONDS);
            sleep(20);
            
            // Set gain
            GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
            gainControl.setGain(gain);
            sleep(20);
        } catch (Exception e) {
            telemetry.addData("Camera Settings", "Failed to set exposure/gain: " + e.getMessage());
        }
    }
}
