package org.firstinspires.ftc.teamcode.pedroPathing;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.pedropathing.util.Timer;

import java.util.function.Supplier;

@Configurable
@TeleOp
public class FieldTeleOp extends OpMode {
    private Follower follower;
    public static Pose startingPose; //See ExampleAuto to understand how to use this
    private boolean automatedDrive;
    private Supplier<PathChain> pathChain;
    private TelemetryManager telemetryM;
    private boolean slowMode = false;
    private double slowModeMultiplier = 0.5;
    private double strafeCorrectionFactor = 0.0; // Start at 0, tune live
    private boolean lastGuide = false;
    private Robot robot;
    private Turret turret;
    private Shooter shooter;

    // we want to know what auto mode it ran before the teleop
    // so we know the robot's startPose
    private enum AutonomusOpMode {
        BLUE_NEAR,
        BLUE_FAR,
        RED_NEAR,
        RED_FAR
    }
    AutonomusOpMode autonomusOpMode = AutonomusOpMode.BLUE_NEAR;

    // Turn control variables for smooth turning
    private double currentTurnRate = 0.0;
    private double targetTurnRate = 0.0;
    private double turnAcceleration = 0.12; // How fast the turn rate changes (0.01 = very slow, 0.2 = very fast)
    private double turnExponent = 1.5; // Higher = more exponential curve (1.0 = linear, 2.0+ = exponential)
    private double maxTurnRate = 1.0; // Maximum turn rate
    private double minTurnThreshold = 0.05; // Minimum input to start turning
    private double maxPowerDelay = 0.3; // Time in seconds to hold full stick before reaching max power
    private double timeAtMaxInput = 0.0; // Time spent at maximum input
    private double currentMaxAllowedRate = 0.0; // Current maximum allowed turn rate
    private double smoothTurnRate = 0.0; // Current smoothed turn rate (accessible throughout loop)


    // B button sequence state machine
    private enum BSequenceState {
        IDLE, SHOOTER_ON_WAIT, KICKER_UP_WAIT_1, KICKER_DOWN_WAIT_1,
        INTAKE_SHOOT_WAIT, KICKER_UP_WAIT_2, KICKER_DOWN_WAIT_2,
        KICKER_UP_WAIT_3, KICKER_DOWN_WAIT_3, SHOOTER_OFF_WAIT
    }
    private BSequenceState sequenceState = BSequenceState.IDLE;
    private Timer pathTimer;
    
    // Edge detection for dpad buttons
    private boolean lastDpadUp = false;
    private boolean lastDpadDown = false;
    private boolean lastDpadLeft = false;
    
    // Turret control modes
    private enum TurretMode {
        MANUAL,           // Manual trigger control
        FIELD_RELATIVE,   // Field-relative lock mode
        APRILTAG          // AprilTag following mode
    }
    private TurretMode turretMode = TurretMode.MANUAL;
    
    // Field-relative turret control
    private boolean turretFieldRelativeLocked = false;
    private double lockedFieldDirection = 0.0; // Field-relative direction to maintain (degrees)
    private double turretPositionEstimate = 0.0; // Estimated turret position relative to robot (degrees)
    private double lastTurretPower = 0.0; // Last turret power applied (for position estimation)
    private boolean lastLeftBumper = false;
    private double lastLoopTime = 0.0;
    // Actual-movement-based control: Only uses actual robot movement, prediction only for smoothing
    private static final double TURRET_MAX_POWER = 0.4; // Maximum turret power
    private static final double POWER_SMOOTHING_ALPHA = 0.3; // Smoothing factor for power output (0.0-1.0, lower = smoother)
    private static final double IMU_VELOCITY_FILTER_ALPHA = 0.5; // Low-pass filter for IMU angular velocity (0.0-1.0, lower = more filtering)
    private static final double MIN_ACTUAL_VELOCITY_THRESHOLD = 2.0; // Only move turret if actual velocity is above this (deg/sec)
    private static final double PREDICTION_SMOOTHING_ALPHA = 0.2; // How much to blend prediction when actual movement exists (0.0-1.0)
    
    // Position-based feedback control using encoder
    private double turretPositionCorrectionGain = 0.02; // Proportional gain for position correction (adjustable with triggers)
    private static final double MIN_POSITION_CORRECTION_GAIN = 0.001; // Minimum gain
    private static final double MAX_POSITION_CORRECTION_GAIN = 0.1; // Maximum gain
    private static final double POSITION_GAIN_ADJUSTMENT_RATE = 0.005; // How much to change per trigger press
    private static final double TURRET_POSITION_DEADBAND = 1.0; // Deadband for position correction (degrees) - reduced for better accuracy
    
    // Robot angular velocity calibration: Maximum robot turn rate at full joystick input
    // This converts smoothTurnRate (-1.0 to 1.0) to predicted angular velocity (deg/sec)
    // Only used for smoothing when actual movement is happening
    private double maxRobotAngularVelocity = 180.0; // deg/sec at full turn input (adjustable with triggers)
    private static final double MIN_MAX_ROBOT_ANGULAR_VELOCITY = 50.0; // Minimum calibration value
    private static final double MAX_MAX_ROBOT_ANGULAR_VELOCITY = 360.0; // Maximum calibration value
    private static final double ROBOT_ANGULAR_VELOCITY_ADJUSTMENT_RATE = 10.0; // How much to change per trigger press
    
    // Turret velocity conversion: need to convert desired turret deg/sec to motor power
    // This is a calibration value - adjustable with triggers in field-relative mode
    private double turretDegPerSecPerPower = 50.0; // Approximate: at power=1.0, turret rotates at this many deg/sec
    private static final double MIN_TURRET_DEG_PER_SEC_PER_POWER = 0.0; // Minimum calibration value
    private static final double MAX_TURRET_DEG_PER_SEC_PER_POWER = 200.0; // Maximum calibration value
    private static final double TURRET_CALIBRATION_ADJUSTMENT_RATE = 2.0; // How much to change per trigger press
    
    // Filtered values for smooth control
    private double filteredActualAngularVelocity = 0.0; // Filtered IMU angular velocity
    private double smoothedTurretPower = 0.0; // Smoothed power output
    
    // Turret gear ratio: 121 teeth (turret) / 47 teeth (motor) = 2.5745 motor rotations per 360° turret rotation
    private static final double TURRET_GEAR_TEETH = 121.0; // Big turret gear teeth
    private static final double MOTOR_GEAR_TEETH = 47.0; // Motor gear teeth
    private static final double TURRET_GEAR_RATIO = TURRET_GEAR_TEETH / MOTOR_GEAR_TEETH; // 2.5745 motor rotations per 360° turret rotation
    private static final double TURRET_DEGREES_PER_MOTOR_ROTATION = 360.0 / TURRET_GEAR_RATIO; // ~139.84 degrees per motor rotation
    
    // Motor specs (adjust if using different motor)
    // REV HD Hex Motor: 28 counts per revolution
    // Other motors: 537.7 (Neverest), 1120 (TETRIX), etc.
    private static final double MOTOR_COUNTS_PER_REVOLUTION = 28.0; // Adjust based on your motor
    private static final double TURRET_DEGREES_PER_ENCODER_TICK = TURRET_DEGREES_PER_MOTOR_ROTATION / MOTOR_COUNTS_PER_REVOLUTION; // ~4.994 degrees per tick
    
    // Estimated turret speed (adjust based on your motor's max RPM at max power)
    // Example: If motor max RPM is ~6000 at power=1.0, then at power=0.4: 6000 * 0.4 = 2400 RPM
    // Motor rotations per second = 2400/60 = 40 rot/sec
    // Turret degrees per second = 40 * 139.84 = ~5593 deg/sec (too high, needs actual measurement)
    // More realistic: Estimate based on typical motor performance
    private static final double ESTIMATED_MOTOR_RPM_AT_MAX_POWER = 150.0; // Adjust based on actual motor performance
    private static final double TURRET_DEGREES_PER_SECOND = (ESTIMATED_MOTOR_RPM_AT_MAX_POWER / 60.0) * TURRET_DEGREES_PER_MOTOR_ROTATION; // Degrees/sec at max power
    
    // Trigger edge detection for calibration adjustment
    private boolean lastLeftTrigger = false;
    private boolean lastRightTrigger = false;

    @Override
    public void init() {
        robot = new Robot(this);
        robot.initHardware(); // Initialize all hardware components
        turret = new Turret(this);
        turret.init();
        shooter = new Shooter(this);
        shooter.init();;

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startingPose == null ? new Pose() : startingPose);
        follower.update();
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        pathTimer = new Timer();
        lastLoopTime = time;
        
        // Initialize turret position from encoder (encoder should be reset in Robot.initHardware())
        // This ensures we start with accurate position tracking
        turretPositionEstimate = turret.getTurretPositionDegrees();
        
        // Verify encoder is working (should be 0 or very close to 0 after reset)
        if (Math.abs(turretPositionEstimate) > 5.0) {
            telemetry.addLine("WARNING: Turret encoder may not be reset properly!");
            telemetry.addData("Initial Position", "%.1f deg", turretPositionEstimate);
            telemetry.update();
        }

        pathChain = () -> follower.pathBuilder() //Lazy Curve Generation
                .addPath(new Path(new BezierLine(follower::getPose, new Pose(45, 98))))
                .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading, Math.toRadians(45), 0.8))
                .build();
    }

    @Override
    public void init_loop() {
        startingPose = new Pose(21.408, 100.51); // default is from blue near
        telemetry.addData("Auto was", autonomusOpMode.toString());
        telemetry.addData("Starding Pose", startingPose.toString());
        telemetry.addLine("Press Left Bumper to choose which auto it ran...");
        telemetry.update();

        if (gamepad1.leftBumperWasPressed()) {
            if (autonomusOpMode == AutonomusOpMode.BLUE_NEAR) {
                autonomusOpMode = AutonomusOpMode.BLUE_FAR;
                startingPose = new Pose(58.883, 35.529);
            } else if (autonomusOpMode == AutonomusOpMode.BLUE_FAR) {
                autonomusOpMode = AutonomusOpMode.RED_NEAR;
                startingPose = new Pose(122.337, 101.112);
            } else if (autonomusOpMode == AutonomusOpMode.RED_NEAR) {
                autonomusOpMode = AutonomusOpMode.RED_FAR;
                startingPose = new Pose(122.337, 101.112);
            } else {
                autonomusOpMode = AutonomusOpMode.BLUE_NEAR;
                startingPose = new Pose(21.408, 100.51);
            }

            telemetry.addData("Auto was", autonomusOpMode.toString());
            telemetry.addData("Starding Pose", startingPose.toString());
            telemetry.update();
        }
    }

    @Override
    public void start() {
        //The parameter controls whether the Follower should use break mode on the motors (using it is recommended).
        //In order to use float mode, add .useBrakeModeInTeleOp(true); to your Drivetrain Constants in Constant.java (for Mecanum)
        //If you don't pass anything in, it uses the default (false)
        follower.startTeleopDrive();
        boolean isRedAlliance = false;
        isRedAlliance = (autonomusOpMode == AutonomusOpMode.RED_NEAR || autonomusOpMode == AutonomusOpMode.RED_FAR);
        turret.setAlliance(isRedAlliance);
        shooter.setAlliance(isRedAlliance);
    }
    
    /**
     * Applies smooth, exponential turning control to the joystick input
     * @param rawTurnInput Raw joystick input (-1.0 to 1.0)
     * @return Smoothed turn rate (-1.0 to 1.0)
     */
    private double applySmoothTurning(double rawTurnInput) {
        // Apply exponential curve to the input for more gradual response
        double sign = Math.signum(rawTurnInput);
        double magnitude = Math.abs(rawTurnInput);
        
        // If input is below threshold, set to zero for fine control
        if (magnitude < minTurnThreshold) {
            targetTurnRate = 0.0;
            timeAtMaxInput = 0.0; // Reset timer when not at max input
        } else {
            // Apply exponential curve (magnitude^exponent) for better fine control
            magnitude = Math.pow(magnitude, turnExponent);
            
            // Check if we're at maximum input (very close to 1.0)
            if (magnitude > 0.95) {
                timeAtMaxInput += 0.02; // Assuming ~50Hz loop rate
                
                // Gradually increase max allowed rate based on time at max input
                double maxAllowedRate = Math.min(1.0, timeAtMaxInput / maxPowerDelay);
                currentMaxAllowedRate = maxAllowedRate;
            } else {
                timeAtMaxInput = 0.0; // Reset timer when not at max input
                currentMaxAllowedRate = 1.0; // Allow full rate for non-max inputs
            }
            
            // Set target turn rate, but limit it based on time at max input
            targetTurnRate = sign * magnitude * maxTurnRate * currentMaxAllowedRate;
        }
        
        // Smoothly transition current turn rate towards target
        if (Math.abs(targetTurnRate - currentTurnRate) < turnAcceleration) {
            currentTurnRate = targetTurnRate;
        } else {
            double direction = Math.signum(targetTurnRate - currentTurnRate);
            currentTurnRate += direction * turnAcceleration;
        }
        
        return currentTurnRate;
    }
    
    /**
     * Normalize angle to -180 to 180 degrees range
     * @param angle Angle in degrees
     * @return Normalized angle in degrees
     */
    private double normalizeAngle(double angle) {
        while (angle > 180.0) angle -= 360.0;
        while (angle < -180.0) angle += 360.0;
        return angle;
    }

    @Override
    public void loop() {
        //Call this once per loop
        follower.update();
        telemetryM.update();

        if (!automatedDrive) {
            //Make the last parameter false for field-centric
            //In case the drivers want to use a "slowMode" you can scale the vectors

            // Live Tuning for Strafe Correction (Hold Guide/Back + Dpad Up/Down)
            if (gamepad1.guide || gamepad1.back) { // Use 'guide' (PS button) or 'back' (Select/View button)
                if (gamepad1.dpad_up && !lastDpadUp) {
                    strafeCorrectionFactor += 0.05;
                } else if (gamepad1.dpad_down && !lastDpadDown) {
                    strafeCorrectionFactor -= 0.05;
                }
            }
            // Logic for regular dpad_up/down is handled below, ensure we don't trigger shooter/intake when tuning
            // We will filter that in the shooter control section

            // Apply smooth turning to the right stick input
            smoothTurnRate = applySmoothTurning(-gamepad1.right_stick_x);
            
            // Strafe correction
            double driveStrafe = -gamepad1.left_stick_x;
            // If strafing right (negative input), we might need to turn right (negative) or left (positive)
            // Correction adds rotation: turnInput + (strafe * factor)
            double strafeCorrection = driveStrafe * strafeCorrectionFactor;

            //This is the normal version to use in the TeleOp
            if (!slowMode) follower.setTeleOpDrive(
                    -gamepad1.left_stick_y,
                    driveStrafe,
                    smoothTurnRate + strafeCorrection,
                    true // Robot Centric
            );

                //This is how it looks with slowMode on
            else follower.setTeleOpDrive(
                    -gamepad1.left_stick_y * slowModeMultiplier,
                    driveStrafe * slowModeMultiplier,
                    (smoothTurnRate + strafeCorrection) * slowModeMultiplier,
                    true // Robot Centric
            );
        }
/*
        //Automated PathFollowing
        if (gamepad1.aWasPressed()) {
            follower.followPath(pathChain.get());
            automatedDrive = true;
        }

        //Stop automated following if the follower is done
        if (automatedDrive && (gamepad1.bWasPressed() || !follower.isBusy())) {
            follower.startTeleopDrive();
            automatedDrive = false;
        }
 */
        if (gamepad1.x) {
            robot.kickerUp();
        }

        if (gamepad1.y) {
            robot.kickerDown();
        }

        if (gamepad1.right_bumper) {
            robot.intakeRev();
        }

        // Shooter control with edge detection
        // Only run if NOT tuning strafe correction (Guide/Back button not held)
        boolean isTuning = gamepad1.guide || gamepad1.back;

        Pose currentPose;
        if (gamepad1.dpadUpWasPressed()) {
            currentPose = follower.getPose();
            this.telemetry.addLine("Shooter ON");
            shooter.shooterOn(currentPose.getX(), currentPose.getY());
        }

        if (!isTuning && gamepad1.dpad_down && !lastDpadDown) {
            shooter.shooterIdle();
            robot.intakeOff();
            sequenceState = BSequenceState.IDLE; // Reset sequence state
        }
        
        // Update last states for edge detection
        lastDpadUp = gamepad1.dpad_up;
        lastDpadDown = gamepad1.dpad_down;

        if (gamepad1.a) {
            robot.intakeShoot();
            /*
            // Handle A button sequence with state machine (instead of B)
            if (sequenceState == BSequenceState.IDLE) {
                robot.shooterClose(); // Use close range shooter power (0.64)
                pathTimer.resetTimer();
                sequenceState = BSequenceState.SHOOTER_ON_WAIT;
            }
             */
        }

        if (gamepad1.b) {
            robot.intakeFast();
        }

        // Calculate delta time for turret position tracking
        double currentTime = time;
        double deltaTime = currentTime - lastLoopTime;
        lastLoopTime = currentTime;
        if (deltaTime <= 0) deltaTime = 0.02; // Default to ~50Hz if time hasn't updated
        
        // Turret mode switching logic
        // Left Dpad: Toggle AprilTag mode on/off
        if (gamepad1.dpad_left && !lastDpadLeft) {
            if (turretMode == TurretMode.APRILTAG) {
                // Turn off AprilTag mode, go back to manual
                turretMode = TurretMode.MANUAL;
                turretFieldRelativeLocked = false;
            } else {
                // Turn on AprilTag mode
                turretMode = TurretMode.APRILTAG;
                turretFieldRelativeLocked = false;
            }
        }
        lastDpadLeft = gamepad1.dpad_left;
        
        // Left Bumper: Mode switching
        if (gamepad1.leftBumperWasPressed()) {
            if (turretMode == TurretMode.APRILTAG) {
                // From AprilTag mode: switch to field-relative mode
                turretMode = TurretMode.FIELD_RELATIVE;
                turretFieldRelativeLocked = true;
                // Lock current field-relative direction using odometry
                double currentTurretPosition = turret.getTurretPositionDegrees();
                turretPositionEstimate = currentTurretPosition;
                // Use odometry heading (more accurate)
                double currentRobotHeading = Math.toDegrees(follower.getPose().getHeading());
                lockedFieldDirection = normalizeAngle(currentRobotHeading + turretPositionEstimate);
                // Reset smoothed power
                smoothedTurretPower = 0.0;
            } else if (turretMode == TurretMode.FIELD_RELATIVE) {
                // From field-relative mode: switch to manual mode
                turretMode = TurretMode.MANUAL;
                turretFieldRelativeLocked = false;
            } else {
                // From manual mode: switch to field-relative mode
                turretMode = TurretMode.FIELD_RELATIVE;
                turretFieldRelativeLocked = true;
                // Lock current field-relative direction using odometry
                double currentTurretPosition = turret.getTurretPositionDegrees();
                turretPositionEstimate = currentTurretPosition;
                // Use odometry heading (more accurate)
                double currentRobotHeading = Math.toDegrees(follower.getPose().getHeading());
                lockedFieldDirection = normalizeAngle(currentRobotHeading + turretPositionEstimate);
                // Reset smoothed power
                smoothedTurretPower = 0.0;
            }
        }
        lastLeftBumper = gamepad1.left_bumper;

        // Turret control - only run if not in AprilTag mode
        if (turretMode == TurretMode.FIELD_RELATIVE) {
            currentPose = follower.getPose();
            double currentRobotHeading = Math.toDegrees(currentPose.getHeading()); // degrees (field-relative)
            double x = currentPose.getX();
            double y = currentPose.getY();
            turret.autoTurn(x, y, currentRobotHeading);
/*  FIXME
            // Hybrid control: Combines velocity feedforward with position feedback using encoder
            
            // Adjust calibration values with triggers (when in field-relative mode)
            // Hold left trigger + dpad_up/down: adjust position correction gain
            // Hold right trigger + dpad_up/down: adjust turret velocity conversion
            // Just triggers: adjust robot max angular velocity
            boolean currentLeftTrigger = gamepad1.left_trigger > 0.1;
            boolean currentRightTrigger = gamepad1.right_trigger > 0.1;
            
            if (currentLeftTrigger && !lastLeftTrigger) {
                // Left trigger pressed: adjust position correction gain with dpad
                if (gamepad1.dpad_up) {
                    turretPositionCorrectionGain = Math.min(MAX_POSITION_CORRECTION_GAIN, turretPositionCorrectionGain + POSITION_GAIN_ADJUSTMENT_RATE);
                } else if (gamepad1.dpad_down) {
                    turretPositionCorrectionGain = Math.max(MIN_POSITION_CORRECTION_GAIN, turretPositionCorrectionGain - POSITION_GAIN_ADJUSTMENT_RATE);
                } else {
                    // Just left trigger: decrease robot max angular velocity (less aggressive prediction)
                    maxRobotAngularVelocity = Math.max(MIN_MAX_ROBOT_ANGULAR_VELOCITY, maxRobotAngularVelocity - ROBOT_ANGULAR_VELOCITY_ADJUSTMENT_RATE);
                }
            }
            if (currentRightTrigger && !lastRightTrigger) {
                // Right trigger pressed: adjust turret velocity conversion with dpad
                if (gamepad1.dpad_up) {
                    turretDegPerSecPerPower = Math.min(MAX_TURRET_DEG_PER_SEC_PER_POWER, turretDegPerSecPerPower + TURRET_CALIBRATION_ADJUSTMENT_RATE);
                } else if (gamepad1.dpad_down) {
                    turretDegPerSecPerPower = Math.max(MIN_TURRET_DEG_PER_SEC_PER_POWER, turretDegPerSecPerPower - TURRET_CALIBRATION_ADJUSTMENT_RATE);
                } else {
                    // Just right trigger: increase robot max angular velocity (more aggressive prediction)
                    maxRobotAngularVelocity = Math.min(MAX_MAX_ROBOT_ANGULAR_VELOCITY, maxRobotAngularVelocity + ROBOT_ANGULAR_VELOCITY_ADJUSTMENT_RATE);
                }
            }
            lastLeftTrigger = currentLeftTrigger;
            lastRightTrigger = currentRightTrigger;
            
            // Get ACTUAL turret position from encoder (PRIMARY SOURCE for position - always use encoder)
            double currentTurretPosition = turret.getTurretPositionDegrees(); // degrees relative to robot (0 = forward)
            turretPositionEstimate = currentTurretPosition; // Always use encoder reading, never estimate
            
            // Get current robot heading from odometry (more accurate than IMU for position)
            // Odometry fuses encoder data with IMU for better accuracy
            double currentRobotHeading = Math.toDegrees(follower.getPose().getHeading()); // degrees (field-relative)
            
            // Calculate desired turret position to maintain locked field direction
            // Field-relative direction = robotHeading + turretPosition (when locked)
            // To maintain: lockedFieldDirection = currentRobotHeading + desiredTurretPosition
            // Therefore: desiredTurretPosition = lockedFieldDirection - currentRobotHeading
            double desiredTurretPosition = normalizeAngle(lockedFieldDirection - currentRobotHeading);
            
            // Calculate position error (how far off the turret is from desired position)
            // This is the error that needs to be corrected
            double positionError = normalizeAngle(desiredTurretPosition - currentTurretPosition);
            
            // Get ACTUAL robot angular velocity from IMU (for velocity feedforward)
            // IMU provides real-time angular velocity which is more responsive than odometry derivative
            double rawActualAngularVelocity = robot.getRobotAngularVelocity(); // deg/sec (positive = CCW)
            
            // Apply low-pass filter to actual velocity to reduce noise and jitter
            // This smooths out sensor noise while maintaining responsiveness
            filteredActualAngularVelocity = filteredActualAngularVelocity * (1.0 - IMU_VELOCITY_FILTER_ALPHA) + rawActualAngularVelocity * IMU_VELOCITY_FILTER_ALPHA;
            
            // Calculate velocity feedforward term (to counteract robot rotation)
            double velocityFeedforward = 0.0;
            
            if (Math.abs(filteredActualAngularVelocity) > MIN_ACTUAL_VELOCITY_THRESHOLD) {
                // Robot is actually moving - use actual velocity as primary source
                // Optionally blend in a small amount of prediction for smoother response
                double predictedRobotAngularVelocity = smoothTurnRate * maxRobotAngularVelocity; // deg/sec
                
                // Blend: 80% actual, 20% prediction (prediction only helps smooth the response)
                double finalAngularVelocity = filteredActualAngularVelocity * (1.0 - PREDICTION_SMOOTHING_ALPHA) + predictedRobotAngularVelocity * PREDICTION_SMOOTHING_ALPHA;
                
                // Calculate required turret velocity to counteract robot rotation
                // If robot rotates +X deg/sec, turret must rotate -X deg/sec to maintain field direction
                double requiredTurretVelocity = -finalAngularVelocity; // deg/sec (negative to counteract)
                
                // Convert turret velocity to motor power
                if (Math.abs(requiredTurretVelocity) > 0.2) { // Small deadband to prevent jitter
                    velocityFeedforward = requiredTurretVelocity / turretDegPerSecPerPower;
                }
            }
            
            // Calculate position feedback term (to correct for drift and accumulated error)
            // This ensures the turret maintains the correct field-relative direction even if
            // velocity feedforward has small errors or there's encoder drift
            double positionFeedback = 0.0;
            if (Math.abs(positionError) > TURRET_POSITION_DEADBAND) {
                // Apply proportional correction with adjustable gain
                // Higher gain = faster correction but may cause oscillation
                // Lower gain = slower correction but more stable
                positionFeedback = positionError * turretPositionCorrectionGain;
                
                // Limit position feedback to prevent sudden large corrections
                double maxPositionFeedback = TURRET_MAX_POWER * 0.3; // Max 30% of max power for position correction
                positionFeedback = Math.max(-maxPositionFeedback, Math.min(maxPositionFeedback, positionFeedback));
            }
            
            // Combine velocity feedforward and position feedback
            // Feedforward handles the main correction (counteracting robot rotation)
            // Feedback handles small corrections (drift, accumulated error)
            double turretPower = velocityFeedforward + positionFeedback;
            
            // Limit power to max
            turretPower = Math.max(-TURRET_MAX_POWER, Math.min(TURRET_MAX_POWER, turretPower));
            
            // Apply exponential smoothing to power output for smooth movement
            smoothedTurretPower = smoothedTurretPower * (1.0 - POWER_SMOOTHING_ALPHA) + turretPower * POWER_SMOOTHING_ALPHA;
            
            // Apply deadband to smoothed power
            if (Math.abs(smoothedTurretPower) < 0.01) {
                turret.turretStop();
                smoothedTurretPower = 0.0;
            } else {
                turret.turretSetPower(smoothedTurretPower);
            }
            
            lastTurretPower = smoothedTurretPower;
 */
        } else if (turretMode == TurretMode.MANUAL) {
            // Manual mode: turret control with left and right triggers
            double manualTurretPower = 0.0;
            if (gamepad1.left_trigger > 0.1) {
                manualTurretPower = 0.3;
                turret.turretTurnRight();
            } else if (gamepad1.right_trigger > 0.1) {
                manualTurretPower = -0.3;
                turret.turretTurnLeft();
            } else {
                turret.turretStop();
            }
            
            // Update turret position estimate in manual mode
            // Use encoder if available, otherwise estimate based on power
            double currentTurretPosition = turret.getTurretPositionDegrees();
            if (Math.abs(currentTurretPosition) > 0.1 || Math.abs(turretPositionEstimate) < 0.1) {
                // Encoder is available, use it
                turretPositionEstimate = currentTurretPosition;
            } else {
                // Encoder not available, estimate based on power and time
                double motorRotationsPerSecond = (manualTurretPower / TURRET_MAX_POWER) * (ESTIMATED_MOTOR_RPM_AT_MAX_POWER / 60.0);
                turretPositionEstimate += motorRotationsPerSecond * TURRET_DEGREES_PER_MOTOR_ROTATION * deltaTime;
                turretPositionEstimate = normalizeAngle(turretPositionEstimate);
            }
            
            // Store power for next loop's position estimation
            lastTurretPower = manualTurretPower;
            
            // Display manual turret control info on Driver Station
            /*
            telemetry.addLine("=== TURRET MANUAL MODE ===");
            telemetry.addData("Left Trigger", "%.2f", gamepad1.left_trigger);
            telemetry.addData("Right Trigger", "%.2f", gamepad1.right_trigger);
            telemetry.addData("Turret Power", "%.2f", manualTurretPower);
            telemetry.addData("Turret Position", "%.1f deg", turretPositionEstimate);
             */
        }
        // Note: AprilTag mode is handled separately via turret.followTag() call

        // State machine for A button sequence (originally was for B button)
        switch (sequenceState) {
            case SHOOTER_ON_WAIT:
                if (pathTimer.getElapsedTimeSeconds() >= 4 || gamepad1.dpad_down) {
                    if (!gamepad1.dpad_down) {
                        robot.kickerUp();
                    }
                    pathTimer.resetTimer();
                    sequenceState = BSequenceState.KICKER_UP_WAIT_1;
                }
                break;
            case KICKER_UP_WAIT_1:
                if (pathTimer.getElapsedTimeSeconds() >= 1 || gamepad1.dpad_down) {
                    if (!gamepad1.dpad_down) {
                        robot.kickerDown();
                    }
                    pathTimer.resetTimer();
                    sequenceState = BSequenceState.KICKER_DOWN_WAIT_1;
                }
                break;
            case KICKER_DOWN_WAIT_1:
                if (pathTimer.getElapsedTimeSeconds() >= 2 || gamepad1.dpad_down) {
                    if (!gamepad1.dpad_down) {
                        robot.intakeShoot();
                    }
                    pathTimer.resetTimer();
                    sequenceState = BSequenceState.INTAKE_SHOOT_WAIT;
                }
                break;
            case INTAKE_SHOOT_WAIT:
                if (pathTimer.getElapsedTimeSeconds() >= 1 || gamepad1.dpad_down) {
                    if (!gamepad1.dpad_down) {
                        robot.kickerUp();
                    }
                    pathTimer.resetTimer();
                    sequenceState = BSequenceState.KICKER_UP_WAIT_2;
                }
                break;
            case KICKER_UP_WAIT_2:
                if (pathTimer.getElapsedTimeSeconds() >= 1 || gamepad1.dpad_down) {
                    if (!gamepad1.dpad_down) {
                        robot.kickerDown();
                    }
                    pathTimer.resetTimer();
                    sequenceState = BSequenceState.KICKER_DOWN_WAIT_2;
                }
                break;
            case KICKER_DOWN_WAIT_2:
                if (pathTimer.getElapsedTimeSeconds() >= 2 || gamepad1.dpad_down) {
                    if (!gamepad1.dpad_down) {
                        robot.kickerUp();
                    }
                    pathTimer.resetTimer();
                    sequenceState = BSequenceState.KICKER_UP_WAIT_3;
                }
                break;
            case KICKER_UP_WAIT_3:
                if (pathTimer.getElapsedTimeSeconds() >= 1 || gamepad1.dpad_down) {
                    if (!gamepad1.dpad_down) {
                        robot.kickerDown();
                    }
                    pathTimer.resetTimer();
                    sequenceState = BSequenceState.KICKER_DOWN_WAIT_3;
                }
                break;
            case KICKER_DOWN_WAIT_3:
                if (pathTimer.getElapsedTimeSeconds() >= 1 || gamepad1.dpad_down) {
                    robot.shooterOff();
                    robot.intakeOff();
                    sequenceState = BSequenceState.IDLE;
                }
                break;
        }

        telemetryM.debug("position", follower.getPose());
        telemetryM.debug("velocity", follower.getVelocity());
        telemetryM.debug("automatedDrive", automatedDrive);
        telemetryM.debug("rawTurnInput", -gamepad1.right_stick_x);
        telemetryM.debug("smoothTurnRate", smoothTurnRate);
        telemetryM.debug("strafeCorrectionFactor", strafeCorrectionFactor);
        telemetryM.debug("timeAtMaxInput", timeAtMaxInput);
        telemetryM.debug("maxAllowedRate", currentMaxAllowedRate);
        telemetryM.debug("turretMode", turretMode.toString());
        telemetryM.debug("turretFieldLocked", turretFieldRelativeLocked);
        telemetryM.debug("robotHeading", Math.toDegrees(follower.getPose().getHeading()));
        telemetryM.debug("lockedFieldDir", lockedFieldDirection);
        telemetryM.debug("turretPosEst", turretPositionEstimate);

        /*
        // Display turret info on Driver Station telemetry
        if (turretMode == TurretMode.FIELD_RELATIVE) {
            // Calculate values for display
            double currentRobotHeading = Math.toDegrees(follower.getPose().getHeading());
            double desiredTurretPosition = normalizeAngle(lockedFieldDirection - currentRobotHeading);
            double positionError = normalizeAngle(desiredTurretPosition - turretPositionEstimate);
            double predictedRobotAngularVelocity = smoothTurnRate * maxRobotAngularVelocity;
            double finalAngularVelocity;
            if (Math.abs(filteredActualAngularVelocity) > MIN_ACTUAL_VELOCITY_THRESHOLD) {
                finalAngularVelocity = filteredActualAngularVelocity * (1.0 - PREDICTION_SMOOTHING_ALPHA) + predictedRobotAngularVelocity * PREDICTION_SMOOTHING_ALPHA;
            } else {
                finalAngularVelocity = 0.0;
            }
            double requiredTurretVelocity = -finalAngularVelocity;
            double velocityFeedforward = 0.0;
            if (Math.abs(requiredTurretVelocity) > 0.2) {
                velocityFeedforward = requiredTurretVelocity / turretDegPerSecPerPower;
            }
            double positionFeedback = 0.0;
            if (Math.abs(positionError) > TURRET_POSITION_DEADBAND) {
                positionFeedback = positionError * turretPositionCorrectionGain;
                // Limit position feedback to prevent sudden large corrections
                double maxPositionFeedback = TURRET_MAX_POWER * 0.3;
                positionFeedback = Math.max(-maxPositionFeedback, Math.min(maxPositionFeedback, positionFeedback));
            }
            
            // Display on Panels telemetry
            telemetryM.debug("=== TURRET CALIBRATION (TUNE THIS) ===", String.format("%.1f", maxRobotAngularVelocity));
            telemetryM.debug("LT: Decrease | RT: Increase", "");
            telemetryM.debug("Final Robot Vel", String.format("%.1f deg/s", finalAngularVelocity));
            telemetryM.debug("Position Error", String.format("%.1f deg", positionError));
            
            // Display on Driver Station telemetry (standard FTC telemetry)
            telemetry.addLine("=== TURRET FIELD-RELATIVE MODE (ENCODER-BASED) ===");
            telemetry.addLine("=== CALIBRATION VALUES (TUNE WITH TRIGGERS) ===");
            telemetry.addData("Max Robot Angular Vel", "%.1f deg/s", maxRobotAngularVelocity);
            telemetry.addData("Turret Deg/Sec Per Power", "%.1f", turretDegPerSecPerPower);
            telemetry.addData("Position Correction Gain", "%.4f", turretPositionCorrectionGain);
            telemetry.addLine("LT+DPad: Adjust Pos Gain | RT+DPad: Adjust Vel Conv");
            telemetry.addLine("");
            telemetry.addLine("=== CONTROL VALUES ===");
            telemetry.addData("Smooth Turn Rate", "%.3f", smoothTurnRate);
            telemetry.addData("Predicted Robot Vel", "%.2f deg/s", predictedRobotAngularVelocity);
            telemetry.addData("Actual Robot Vel (IMU)", "%.2f deg/s", filteredActualAngularVelocity);
            telemetry.addData("Min Velocity Threshold", "%.1f deg/s", MIN_ACTUAL_VELOCITY_THRESHOLD);
            if (Math.abs(filteredActualAngularVelocity) > MIN_ACTUAL_VELOCITY_THRESHOLD) {
                telemetry.addData("Status", "ACTIVE - Robot moving");
            } else {
                telemetry.addData("Status", "INACTIVE - Robot not moving");
            }
            telemetry.addData("Final Robot Vel", "%.2f deg/s", finalAngularVelocity);
            telemetry.addData("Required Turret Vel", "%.2f deg/s", requiredTurretVelocity);
            telemetry.addData("Velocity Feedforward", "%.3f", velocityFeedforward);
            telemetry.addData("Position Feedback", "%.3f", positionFeedback);
            telemetry.addData("Turret Power", "%.3f", smoothedTurretPower);
            telemetry.addLine("");
            telemetry.addLine("=== ENCODER POSITION (FROM ENCODER) ===");
            telemetry.addData("Robot Heading", "%.1f deg", currentRobotHeading);
            telemetry.addData("Locked Field Dir", "%.1f deg", lockedFieldDirection);
            telemetry.addData("Desired Turret Pos", "%.1f deg", desiredTurretPosition);
            telemetry.addData("Actual Turret Pos", "%.1f deg", turretPositionEstimate);
            telemetry.addData("Position Error", "%.1f deg", positionError);
            telemetry.addLine("LT: Decrease Max Vel | RT: Increase Max Vel");
        } else if (turretMode == TurretMode.MANUAL) {
            // Display manual mode info
            telemetry.addLine("=== TURRET MODE: MANUAL ===");
            telemetry.addLine("Left Bumper: Field-Relative Mode");
            telemetry.addLine("Left Dpad: AprilTag Mode");
        } else if (turretMode == TurretMode.APRILTAG) {
            // Display AprilTag mode info
            telemetry.addLine("=== TURRET MODE: APRILTAG ===");
            telemetry.addLine("Left Bumper: Switch to Field-Relative");
            telemetry.addLine("Left Dpad: Switch to Manual");
        }
*/
        // Only run AprilTag control when in AprilTag mode
        if (turretMode == TurretMode.APRILTAG) {
            turret.followTag();
        }

        // Update standard telemetry for Driver Station
        telemetry.update();
    }
}