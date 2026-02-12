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
    public Pose startingPose; //See ExampleAuto to understand how to use this
    private boolean automatedDrive;
    private Supplier<PathChain> pathChain;
    private TelemetryManager telemetryM;
    private boolean slowMode = false;
    private double slowModeMultiplier = 0.5;
    private Robot robot;
    private Turret turret;
    private Shooter shooter;

    // we want to know what auto mode it ran before the teleop
    // so we know the robot's startPose
    private enum AutonomusOpMode {
        BLUE_NEAR,
        BLUE_FAR,
        RED_NEAR,
        RED_FAR,
        ORIGIN
    }
    AutonomusOpMode autonomusOpMode = AutonomusOpMode.BLUE_NEAR;

    private Timer pathTimer;

    // Turret control modes
    private enum TurretMode {
        MANUAL,           // Manual trigger control
        FIELD_RELATIVE,   // Field-relative lock mode
        APRILTAG          // AprilTag following mode
    }
    private TurretMode turretMode = TurretMode.MANUAL;

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

        pathChain = () -> follower.pathBuilder() //Lazy Curve Generation
                .addPath(new Path(new BezierLine(follower::getPose, new Pose(45, 98))))
                .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading, Math.toRadians(45), 0.8))
                .build();
    }

    @Override
    public void init_loop() {
        telemetry.addData("Auto was", autonomusOpMode.toString());
        telemetry.addLine("Press Left Bumper to choose which auto it ran...");

        if (gamepad1.leftBumperWasPressed()) {
            if (autonomusOpMode == AutonomusOpMode.BLUE_NEAR) {
                autonomusOpMode = AutonomusOpMode.BLUE_FAR;
            } else if (autonomusOpMode == AutonomusOpMode.BLUE_FAR) {
                autonomusOpMode = AutonomusOpMode.RED_NEAR;
            } else if (autonomusOpMode == AutonomusOpMode.RED_NEAR) {
                autonomusOpMode = AutonomusOpMode.RED_FAR;
            } else if (autonomusOpMode == AutonomusOpMode.RED_FAR) {
                autonomusOpMode = AutonomusOpMode.ORIGIN;
            } else {
                autonomusOpMode = AutonomusOpMode.BLUE_NEAR;
            }
        }
    }

    @Override
    public void start() {
        initStartingPose();
        follower.setStartingPose(startingPose);
        follower.update();
        //The parameter controls whether the Follower should use break mode on the motors (using it is recommended).
        //In order to use float mode, add .useBrakeModeInTeleOp(true); to your Drivetrain Constants in Constant.java (for Mecanum)
        //If you don't pass anything in, it uses the default (false)
        follower.startTeleopDrive();

        boolean isRedAlliance = false;
        isRedAlliance = (autonomusOpMode == AutonomusOpMode.RED_NEAR || autonomusOpMode == AutonomusOpMode.RED_FAR);
        turret.setAlliance(isRedAlliance);
        shooter.setAlliance(isRedAlliance);

        telemetry.addData("Auto was", autonomusOpMode.toString());
        telemetry.addData("startingPosition", startingPose.toString());
        telemetry.addLine("Start...");
        telemetry.update();
    }

    @Override
    public void loop() {
        //Call this once per loop
        follower.update();

        if (!automatedDrive) {
            //Make the last parameter false for field-centric
            //In case the drivers want to use a "slowMode" you can scale the vectors
            //This is the normal version to use in the TeleOp
            if (!slowMode) follower.setTeleOpDrive(
                    -gamepad1.left_stick_y,
                    -gamepad1.left_stick_x,
                    -gamepad1.right_stick_x,
                    true // Robot Centric
            );

                //This is how it looks with slowMode on
            else follower.setTeleOpDrive(
                    -gamepad1.left_stick_y * slowModeMultiplier,
                    -gamepad1.left_stick_x * slowModeMultiplier,
                    -gamepad1.right_stick_x * slowModeMultiplier,
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
            robot.kickerUp();
        }

        Pose currentPose;
        if (gamepad1.dpadUpWasPressed()) {
            currentPose = follower.getPose();
            this.telemetry.addLine("Shooter ON");
            double targetVelocity = shooter.shooterOn(currentPose.getX(), currentPose.getY());
            pathTimer.resetTimer();
            while (pathTimer.getElapsedTimeSeconds() <= 1.0) {
                double error = Math.abs(targetVelocity - shooter.getVelocity());
                if (error <= 40.0) {
                    break;
                }
            }
            robot.intakeShoot();
            robot.kickerUp();
            pathTimer.resetTimer();
        }

        if (gamepad1.dpadDownWasPressed()) {
            shooter.shooterIdle();
            robot.intakeOff();
            robot.kickerDown();
        }

        if (gamepad1.a) {
            robot.intakeShoot();
            robot.kickerDown();
        }

        // Left Bumper: Mode switching
        if (gamepad1.leftBumperWasPressed()) {
            if (turretMode == TurretMode.APRILTAG) {
                // From AprilTag mode: switch to field-relative mode
                turretMode = TurretMode.FIELD_RELATIVE;
            } else if (turretMode == TurretMode.FIELD_RELATIVE) {
                // From field-relative mode: switch to manual mode
                turretMode = TurretMode.MANUAL;
            } else {
                // From manual mode: switch to field-relative mode
                turretMode = TurretMode.FIELD_RELATIVE;
            }
        }

        // Turret control - only run if not in AprilTag mode
        if (turretMode == TurretMode.FIELD_RELATIVE) {
            currentPose = follower.getPose();
            double currentRobotHeading = Math.toDegrees(currentPose.getHeading()); // degrees (field-relative)
            double x = currentPose.getX();
            double y = currentPose.getY();
            turret.autoTurn(x, y, currentRobotHeading);
        } else if (turretMode == TurretMode.MANUAL) {
            // Manual mode: turret control with left and right triggers
            if (gamepad1.left_trigger > 0.1) {
                turret.turretTurnRight();
            } else if (gamepad1.right_trigger > 0.1) {
                turret.turretTurnLeft();
            } else {
                turret.turretStop();
            }
        }

        // Only run AprilTag control when in AprilTag mode
        if (turretMode == TurretMode.APRILTAG) {
            turret.followTag();
        }

        // Update standard telemetry for Driver Station
        telemetry.update();
    }

    private void initStartingPose() {
        if (autonomusOpMode == AutonomusOpMode.BLUE_NEAR) {
            startingPose = new Pose(24.037, 96.815);
        } else if (autonomusOpMode == AutonomusOpMode.BLUE_FAR) {
            startingPose = new Pose(61.674, 35.700);
        } else if (autonomusOpMode == AutonomusOpMode.RED_NEAR) {
            startingPose = new Pose(125.972, 100.376);
        } else if (autonomusOpMode == AutonomusOpMode.RED_FAR) {
            startingPose = new Pose(83.478, 35.461);
        } else {
            startingPose = new Pose();
        }
    }
}