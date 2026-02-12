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

    private static final double TICKS_PER_DEGREE = 384.5 // ticks per rotation per Yellow Jacket 5203 spec
            * 121.0 / 57.0 // This is the external gear reduction, a 57T pinion gear that drives a 127T hub-mount gear
            * 1/360.0; // we want ticks per degree, not per rotation
    private double turretAutoTurnPower = 0.8;
    private boolean isRedAlliance = false; // default to Blue

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

    // reset turret back to start position
    public void reset() {
        turretMotor.setTargetPosition(-turretMotor.getCurrentPosition());
        turretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        turretMotor.setPower(this.turretAutoTurnPower);
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void setAlliance(boolean isRedAlliance) {
        this.isRedAlliance = isRedAlliance;
    }

    public void autoTurn(double x, double y, double robotHeadingInDegree) {
        double turnInDegree = 0.0;
        boolean doAutoTurn = false;
        if (isRedAlliance) {
            x = x + RobotUtils.poseXOffsetRed;
            y = y + RobotUtils.poseYOffsetRed;
        } else {
            x = x + RobotUtils.poseXOffsetBlue;
            y = y + RobotUtils.poseYOffsetBlue;
        }
        // normalize to [0, 360)
        double normalizedRobotHeadingInDegree = (robotHeadingInDegree % 360 + 360) % 360;

        myOpMode.telemetry.addData("Current bot heading", robotHeadingInDegree);
        myOpMode.telemetry.addData("Current bot heading normalized [0,360)", normalizedRobotHeadingInDegree);
        myOpMode.telemetry.addData("Current turret ticks", turretMotor.getCurrentPosition());
        myOpMode.telemetry.addData("Current bot X", x);
        myOpMode.telemetry.addData("Current bot Y", y);


        // when bot facing the goal, turret auto turns; otherwise no turn
        if (isRedAlliance) { // RED GOAL
            double turnInRadians = Math.atan(Math.abs((144.0 - Math.abs(y)) / (144.0 - Math.abs(x)))); // 0 to pi/2
            if (!Double.isNaN(turnInRadians)) {
                if (normalizedRobotHeadingInDegree >= 0.0 && normalizedRobotHeadingInDegree <= 135.0) {
                    turnInDegree = Math.toDegrees(turnInRadians) - normalizedRobotHeadingInDegree;
                    doAutoTurn = true;
                }
                if (normalizedRobotHeadingInDegree >= 315.0) {
                    turnInDegree = Math.toDegrees(turnInRadians) - (normalizedRobotHeadingInDegree - 360);
                    doAutoTurn = true;
                }
            }
        } else { // BLUE GOAL
            // when bot facing the goal, turret auto turns; otherwise no turn
            double turnInRadians = Math.atan((144.0 - Math.abs(y)) / (Math.abs(x) - 0.0)); // 0 to pi/2
            if (!Double.isNaN(turnInRadians)) {
                myOpMode.telemetry.addData("Turret atan: ", Math.toDegrees(turnInRadians));
                if (normalizedRobotHeadingInDegree >= 45.0 && normalizedRobotHeadingInDegree <= 180.0) { // wire
                    turnInDegree = (180 - normalizedRobotHeadingInDegree) - Math.toDegrees(turnInRadians);
                    doAutoTurn = true;
                }
            }
        }

        if (doAutoTurn) {
            if (turnInDegree > 80.0) {
                turnInDegree = 80.0;
            }
            if (turnInDegree < -80.0) {
                turnInDegree = -80.0;
            }
            int turnInTicks = (int) (turnInDegree * TICKS_PER_DEGREE);

            turretMotor.setTargetPosition(turnInTicks);
            turretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            turretMotor.setPower(this.turretAutoTurnPower);

            myOpMode.telemetry.addData("Turret turn", turnInDegree);
            myOpMode.telemetry.addData("Turret ticks", turnInTicks);
            myOpMode.telemetry.addData("Turret power", turretAutoTurnPower);

        }
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