package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

@Autonomous(name = "RedNearAuto", group = "Auto")
public class RedNearAuto extends OpMode {

    private Robot robot;
    private Turret turret;
    private Shooter shooter;
    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;

    private final Pose startPose = new Pose(109.80561555075593, 137.69330453563714, Math.toRadians(0)); // Start Pose of our robot.
    private final Pose scorePose = new Pose(95.66689199215207, 107.86764051242314, Math.toRadians(38)); // Scoring Pose of our robot. It is facing the goal at a 135 degree angle.
    private final Pose intake1Pose = new Pose(95.2159827213823, 59.761776004484545, Math.toRadians(0)); // Scoring Pose of our robot. It is facing the goal at a 135 degree angle.
    private final Pose pickup1Pose = new Pose(132.6564885496183, 59.221374045801525, Math.toRadians(0)); // Highest (First Set) of Artifacts from the Spike Mark.
    private final Pose revPose = new Pose(112.62595419847328, 59.221374045801525, Math.toRadians(0)); // Intake (First Set) of Artifacts from the Spike Mark.
    private final Pose intake2Pose = new Pose(95.2159827213823, 83.19083969465649, Math.toRadians(0)); // Middle (Second Set) of Artifacts from the Spike Mark.
    private final Pose pickup2Pose = new Pose(126.04580152671755, 83.12213740458016, Math.toRadians(0)); // Middle (Second Set) of Artifacts from the Spike Mark.
    private final Pose parkPose = new Pose(120.42762063227953, 96.16971713810318, Math.toRadians(0)); // Lowest (Third Set) of Artifacts from the Spike Mark.

    Pose currentPose;
    private Path scorePreload;
    private PathChain readyIntake1, intakePickup1, reverse, scorePickup1, readyIntake2, intakePickup2, scorePickup2, park;

    public void buildPaths() {
        scorePreload = new Path(new BezierLine(startPose, scorePose));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading());

        /* This is our grabPickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        readyIntake1 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, intake1Pose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), intake1Pose.getHeading())
                .build();

        intakePickup1 = follower.pathBuilder()
                .addPath(new BezierLine(intake1Pose, pickup1Pose))
                .setLinearHeadingInterpolation(intake1Pose.getHeading(), pickup1Pose.getHeading())
                .build();

        /* This is our scorePickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        reverse = follower.pathBuilder()
                .addPath(new BezierLine(pickup1Pose, revPose))
                .setLinearHeadingInterpolation(pickup1Pose.getHeading(), revPose.getHeading())
                .build();

        /* This is our grabPickup2 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        scorePickup1 = follower.pathBuilder()
                .addPath(new BezierLine(revPose, scorePose))
                .setLinearHeadingInterpolation(revPose.getHeading(), scorePose.getHeading())
                .build();

        readyIntake2 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, intake2Pose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), intake2Pose.getHeading())
                .build();

        /* This is our scorePickup2 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        intakePickup2 = follower.pathBuilder()
                .addPath(new BezierLine(intake2Pose, pickup2Pose))
                .setLinearHeadingInterpolation(intake2Pose.getHeading(), pickup2Pose.getHeading())
                .build();

        /* This is our grabPickup3 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        scorePickup2 = follower.pathBuilder()
                .addPath(new BezierLine(pickup2Pose, scorePose))
                .setLinearHeadingInterpolation(pickup2Pose.getHeading(), scorePose.getHeading())
                .build();

        /* This is our scorePickup3 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        park = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, parkPose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), parkPose.getHeading())
                .build();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(scorePreload);
                setPathState(1);
                break;
            case 1:
                if (!follower.isBusy()) {
                    /* Score Preload */
                    currentPose = follower.getPose();
                    shooter.shooterOn(currentPose.getX(), currentPose.getY());
                    pathTimer.resetTimer();

                    while (pathTimer.getElapsedTimeSeconds() <= 1) {}
                    robot.kickerUp();
                    robot.intakeShoot();
                    pathTimer.resetTimer();

                    while (pathTimer.getElapsedTimeSeconds() <= 2) {}
                    shooter.shooterIdle();
                    robot.kickerDown();
                    pathTimer.resetTimer();

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    follower.followPath(readyIntake1, true);
                    setPathState(2);
                }
                break;
            case 2:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup1Pose's position */
                if (!follower.isBusy()) {
                    pathTimer.resetTimer();
                    follower.followPath(intakePickup1, true);
                    setPathState(3);
                }
                break;
            case 3:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if (!follower.isBusy()) {
                    pathTimer.resetTimer();
                    follower.followPath(reverse, true);
                    setPathState(4);
                }
                break;
            case 4:
                if (!follower.isBusy()) {
                    robot.intakeOff();
                    pathTimer.resetTimer();
                    follower.followPath(scorePickup1, true);
                    setPathState(5);
                }
                break;
            case 5:
                if (!follower.isBusy()) {
                    currentPose = follower.getPose();
                    shooter.shooterOn(currentPose.getX(), currentPose.getY());
                    pathTimer.resetTimer();

                    while (pathTimer.getElapsedTimeSeconds() <= 1) {}
                    robot.kickerUp();
                    robot.intakeShoot();
                    pathTimer.resetTimer();

                    while (pathTimer.getElapsedTimeSeconds() <= 2) {}
                    shooter.shooterIdle();
                    robot.kickerDown();
                    pathTimer.resetTimer();

                    follower.followPath(readyIntake2, true);
                    setPathState(6);
                }
                break;
            case 6:
                if (!follower.isBusy()) {
                    pathTimer.resetTimer();
                    follower.followPath(intakePickup2, true);
                    setPathState(7);
                }
                break;
            case 7:
                if (!follower.isBusy()) {
                    robot.intakeOff();
                    pathTimer.resetTimer();
                    follower.followPath(scorePickup2, true);
                    setPathState(8);
                }
                break;
            case 8:
                if (!follower.isBusy()) {
                    currentPose = follower.getPose();
                    shooter.shooterOn(currentPose.getX(), currentPose.getY());
                    pathTimer.resetTimer();

                    while (pathTimer.getElapsedTimeSeconds() <= 1) {}
                    robot.kickerUp();
                    robot.intakeShoot();
                    pathTimer.resetTimer();

                    while (pathTimer.getElapsedTimeSeconds() <= 2) {}
                    robot.shooterOff();
                    robot.intakeOff();
                    robot.kickerDown();
                    pathTimer.resetTimer();

                    follower.followPath(park, true);
                    setPathState(9);
                }
                break;
            case 9:
                if (!follower.isBusy()) {
                    pathTimer.resetTimer();
                    setPathState(-1);
                }
                break;
        }
    }

    /**
     * These change the states of the paths and actions. It will also reset the timers of the individual switches
     **/
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    /**
     * This is the main loop of the OpMode, it will run repeatedly after clicking "Play".
     **/
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
        telemetry.update();
    }

    /**
     * This method is called once at the init of the OpMode.
     **/
    @Override
    public void init() {
        robot = new Robot(this);
        robot.initHardware();

        turret = new Turret(this);
        turret.init();

        shooter = new Shooter(this);
        shooter.init();

        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(startPose);

    }

    /**
     * This method is called continuously after Init while waiting for "play".
     **/
    @Override
    public void init_loop() {
    }

    /**
     * This method is called once at the start of the OpMode.
     * It runs all the setup actions, including building paths and starting the path system
     **/
    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
    }

    /**
     * We do not use this because everything should automatically disable
     **/
    @Override
    public void stop() {
    }
}