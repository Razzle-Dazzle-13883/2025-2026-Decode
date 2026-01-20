package org.firstinspires.ftc.teamcode.pedroPathing; // make sure this aligns with class location

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import  com.qualcomm.robotcore.eventloop.opmode.OpMode;

@Autonomous(name = "RedNearAuto", group = "Auto")
public class RedNearAuto extends OpMode {

    private Robot robot;
    private Turret turret;
    private Shooter shooter;

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;

    // Shooter sequence state machine (from FieldTeleOp)
    /*
    private enum ShooterSequenceState {
        IDLE, SHOOTER_ON_WAIT, KICKER_UP_WAIT_1, KICKER_DOWN_WAIT_1,
        INTAKE_SHOOT_WAIT, KICKER_UP_WAIT_2, KICKER_DOWN_WAIT_2,
        KICKER_UP_WAIT_3, KICKER_DOWN_WAIT_3, SHOOTER_OFF_WAIT
    }
     */

    // private ShooterSequenceState sequenceState = ShooterSequenceState.IDLE;

    private int pathState;
    private final Pose startPose = new Pose(120.38012958963283, 125.87473002159828, Math.toRadians(38)); // Start Pose of our robot.
    private final Pose scorePose = new Pose(101.71922246220304, 110.33261339092871, Math.toRadians(38)); // Scoring Pose of our robot. It is facing the goal at a 135 degree angle.
    private final Pose readyPose = new Pose(101.56803455723542, 101.2095032397408, Math.toRadians(0)); // Scoring Pose of our robot. It is facing the goal at a 135 degree angle.
    private final Pose parkPose = new Pose(122.33693304535636, 101.11231101511879, Math.toRadians(0)); // Highest (First Set) of Artifacts from the Spike Mark.

    private final Pose intake1Pose = new Pose(120.05183585313175, 83.66306695464363, Math.toRadians(0)); // Intake (First Set) of Artifacts from the Spike Mark.
    private final Pose pickup2Pose = new Pose(95.48164146868251, 59.40388768898488, Math.toRadians(0)); // Middle (Second Set) of Artifacts from the Spike Mark.
    private final Pose intake2Pose = new Pose(120.05183585313175, 59.714902807775374, Math.toRadians(0)); // Middle (Second Set) of Artifacts from the Spike Mark.
    private final Pose pickup3Pose = new Pose(49, 135, Math.toRadians(0)); // Lowest (Third Set) of Artifacts from the Spike Mark.

    Pose currentPose;
    private Path scorePreload;
    private PathChain readyPark, park, scorePickup1, grabPickup2, intakePickup2, scorePickup2, grabPickup3, scorePickup3;

    public void buildPaths() {
        /* This is our scorePreload path. We are using a BezierLine, which is a straight line. */
        scorePreload = new Path(new BezierLine(startPose, scorePose));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading());

    /* Here is an example for Constant Interpolation
    scorePreload.setConstantInterpolation(startPose.getHeading()); */

        /* This is our grabPickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        readyPark = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, readyPose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), readyPose.getHeading())
                .build();

        park = follower.pathBuilder()
                .addPath(new BezierLine(readyPose, parkPose))
                .setLinearHeadingInterpolation(readyPose.getHeading(), parkPose.getHeading())
                .build();

        /* This is our scorePickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        scorePickup1 = follower.pathBuilder()
                .addPath(new BezierLine(intake1Pose, parkPose))
                .setLinearHeadingInterpolation(intake1Pose.getHeading(), parkPose.getHeading())
                .build();

        /* This is our grabPickup2 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        grabPickup2 = follower.pathBuilder()
                .addPath(new BezierLine(parkPose, pickup2Pose))
                .setLinearHeadingInterpolation(parkPose.getHeading(), pickup2Pose.getHeading())
                .build();

        intakePickup2 = follower.pathBuilder()
                .addPath(new BezierLine(pickup2Pose, intake2Pose))
                .setLinearHeadingInterpolation(pickup2Pose.getHeading(), intake2Pose.getHeading())
                .build();

        /* This is our scorePickup2 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        scorePickup2 = follower.pathBuilder()
                .addPath(new BezierLine(pickup2Pose, scorePose))
                .setLinearHeadingInterpolation(intake2Pose.getHeading(), scorePose.getHeading())
                .build();

        /* This is our grabPickup3 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        grabPickup3 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, pickup3Pose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup3Pose.getHeading())
                .build();

        /* This is our scorePickup3 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        scorePickup3 = follower.pathBuilder()
                .addPath(new BezierLine(pickup3Pose, scorePose))
                .setLinearHeadingInterpolation(pickup3Pose.getHeading(), scorePose.getHeading())
                .build();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(scorePreload);
                setPathState(1);
                break;
            case 1:

            /* You could check for
            - Follower State: "if(!follower.isBusy()) {}"
            - Time: "if(pathTimer.getElapsedTimeSeconds() > 1) {}"
            - Robot Position: "if(follower.getPose().getX() > 36) {}"
            */

                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if (!follower.isBusy()) {
                    /* Score Preload */
                    currentPose = follower.getPose();
                    shooter.shooterOn(currentPose.getX(), currentPose.getY());
                    pathTimer.resetTimer();

                    while (pathTimer.getElapsedTimeSeconds() <= 2) {}
                    robot.intakeShoot();
                    pathTimer.resetTimer();

                    while (pathTimer.getElapsedTimeSeconds() <= 3) {}
                    shooter.shooterIdle();
                    robot.intakeOff();
                    pathTimer.resetTimer();

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    follower.followPath(readyPark, true);
                    setPathState(2);
                }
                break;
            case 2:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup1Pose's position */
                if (!follower.isBusy()) {
                    /* Grab Sample */
                    pathTimer.resetTimer();
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    follower.followPath(park, true);
                    setPathState(3);
                }
                break;
            case 3:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if (!follower.isBusy()) {
                    /* Score Sample */
                    robot.shooterOff();
                    pathTimer.resetTimer();
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    //follower.followPath(scorePickup1, true);
                    setPathState(-1);
                }
                break;
            case 4:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup2Pose's position */
                if (!follower.isBusy()) {
                    /* Grab Sample */
                    turret.followTag();
                    robot.shooterOn();

                    pathTimer.resetTimer();

                    while (pathTimer.getElapsedTimeSeconds() <= 3) {}
                    robot.intakeFast();

                    pathTimer.resetTimer();

                    while (pathTimer.getElapsedTimeSeconds() <= 4) {}
                    robot.shooterOff();
                    robot.intakeOff();
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    // follower.followPath(grabPickup2, true);
                    setPathState(-1);
                }
                break;
            case 5:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if (!follower.isBusy()) {
                    /* Score Sample */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    follower.followPath(intakePickup2, true);
                    setPathState(6);
                }
                break;
            case 6:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup3Pose's position */
                if (!follower.isBusy()) {
                    /* Grab Sample */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    follower.followPath(scorePickup2, true);
                    setPathState(7);
                }
                break;
            case 7:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if (!follower.isBusy()) {
                    /* Set the state to a Case we won't use or define, so it just stops running an new paths */
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