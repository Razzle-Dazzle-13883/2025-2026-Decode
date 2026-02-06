package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

@Autonomous(name = "BlueNearAuto", group = "Auto")
public class BlueNearAuto extends OpMode {

    private Robot robot;
    private Turret turret;
    private Shooter shooter;
    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;

    private final Pose startPose = new Pose(34.35584739001405, 137.69330453563714, Math.toRadians(180)); // Start Pose of our robot.
    private final Pose scorePose = new Pose(47.81526911734527, 113.8769140827477, Math.toRadians(142)); // Scoring Pose of our robot. It is facing the goal at a 135 degree angle.
    private final Pose intake1Pose = new Pose(54.041330480269465, 90.53550739171988, Math.toRadians(180)); // Scoring Pose of our robot. It is facing the goal at a 135 degree angle.
    private final Pose pickup1Pose = new Pose(18.0983517585568, 90.69246197954148, Math.toRadians(180)); // Highest (First Set) of Artifacts from the Spike Mark.
    private final Pose parkPose = new Pose(24.037094281298288, 96.81452859350847, Math.toRadians(0)); // Lowest (Third Set) of Artifacts from the Spike Mark.

    Pose currentPose;
    private Path scorePreload;
    private PathChain readyIntake1, intakePickup1, scorePickup1, park;

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

        /* This is our grabPickup2 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        scorePickup1 = follower.pathBuilder()
                .addPath(new BezierLine(pickup1Pose, scorePose))
                .setLinearHeadingInterpolation(pickup1Pose.getHeading(), scorePose.getHeading())
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

                    while (pathTimer.getElapsedTimeSeconds() <= 3) {}
                    robot.kickerUp();
                    robot.intakeShoot();
                    pathTimer.resetTimer();

                    while (pathTimer.getElapsedTimeSeconds() <= 2) {}
                    robot.shooterOff();
                    robot.intakeOff();
                    robot.kickerDown();
                    pathTimer.resetTimer();

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    follower.followPath(park, true);
                    setPathState(5);
                }
                break;
            case 2:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup1Pose's position */
                if (!follower.isBusy()) {
                    pathTimer.resetTimer();
                    follower.followPath(intakePickup1, true);
                    setPathState(-1);
                }
                break;
            case 3:
                if (!follower.isBusy()) {
                    while (pathTimer.getElapsedTimeSeconds() <= 1) {}
                    robot.intakeOff();
                    pathTimer.resetTimer();
                    follower.followPath(scorePickup1, true);
                    setPathState(-1);
                }
                break;
            case 4:
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
                    setPathState(-1);
                }
                break;
            case 5:
                if (!follower.isBusy()) {
                    turret.reset();
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
        turret.setAlliance(false); // blue

        shooter = new Shooter(this);
        shooter.init();
        shooter.setAlliance(false); // blue

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