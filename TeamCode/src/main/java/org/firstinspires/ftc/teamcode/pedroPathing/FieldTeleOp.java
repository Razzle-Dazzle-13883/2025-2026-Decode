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
    private Robot robot;
    
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
    
    // Intake slow timing variables
    private boolean intakeSlowActive = false;
    private double intakeSlowStartTime = 0.0;
    private double intakeSlowDuration = 0.4; // Duration in seconds for intake slow (400ms)

    @Override
    public void init() {
        robot = new Robot(this);
        robot.initHardware(); // Initialize all hardware components
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startingPose == null ? new Pose() : startingPose);
        follower.update();
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        pathChain = () -> follower.pathBuilder() //Lazy Curve Generation
                .addPath(new Path(new BezierLine(follower::getPose, new Pose(45, 98))))
                .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading, Math.toRadians(45), 0.8))
                .build();
    }

    @Override
    public void start() {
        //The parameter controls whether the Follower should use break mode on the motors (using it is recommended).
        //In order to use float mode, add .useBrakeModeInTeleOp(true); to your Drivetrain Constants in Constant.java (for Mecanum)
        //If you don't pass anything in, it uses the default (false)
        follower.startTeleopDrive();
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

    @Override
    public void loop() {
        //Call this once per loop
        follower.update();
        telemetryM.update();

        if (!automatedDrive) {
            //Make the last parameter false for field-centric
            //In case the drivers want to use a "slowMode" you can scale the vectors

            // Apply smooth turning to the right stick input
            double smoothTurnRate = applySmoothTurning(-gamepad1.right_stick_x);
            
            //This is the normal version to use in the TeleOp
            if (!slowMode) follower.setTeleOpDrive(
                    -gamepad1.left_stick_y,
                    -gamepad1.left_stick_x,
                    smoothTurnRate,
                    true // Robot Centric
            );

                //This is how it looks with slowMode on
            else follower.setTeleOpDrive(
                    -gamepad1.left_stick_y * slowModeMultiplier,
                    -gamepad1.left_stick_x * slowModeMultiplier,
                    smoothTurnRate * slowModeMultiplier,
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

        if (gamepad1.dpad_up) {
            robot.shooterOn();
        }

        if (gamepad1.dpad_down) {
            robot.shooterOff();
            robot.intakeOff();
        }

        if (gamepad1.a) {
            robot.intakeShoot();
        }

        if (gamepad1.dpad_right) {
            robot.shooterRev();
            // Start intake slow sequence
            intakeSlowActive = true;
            intakeSlowStartTime = time;
        }

        if (gamepad1.dpad_left) {
            robot.intakeFast();
        }

        // Handle intake slow sequence
        if (intakeSlowActive) {
            double elapsedTime = time - intakeSlowStartTime;
            if (elapsedTime < intakeSlowDuration) {
                // Run intake slow for the duration
                robot.intakeSlow();
            } else {
                // After duration, turn off intake
                robot.intakeOff();
                intakeSlowActive = false;
            }
        }

        telemetryM.debug("position", follower.getPose());
        telemetryM.debug("velocity", follower.getVelocity());
        telemetryM.debug("automatedDrive", automatedDrive);
        telemetryM.debug("rawTurnInput", -gamepad1.right_stick_x);
        telemetryM.debug("smoothTurnRate", currentTurnRate);
        telemetryM.debug("timeAtMaxInput", timeAtMaxInput);
        telemetryM.debug("maxAllowedRate", currentMaxAllowedRate);
    }
}