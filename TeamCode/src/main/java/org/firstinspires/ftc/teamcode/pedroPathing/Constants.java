package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.Encoder;
import com.pedropathing.ftc.localization.constants.TwoWheelConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Constants {
    public static FollowerConstants followerConstants = new FollowerConstants()
           .forwardZeroPowerAcceleration(-42.878528433854)
           .lateralZeroPowerAcceleration(-78.28003095482067)
            .translationalPIDFCoefficients(new PIDFCoefficients(0.1, 0, 0.02, 0.03))
            .headingPIDFCoefficients(new PIDFCoefficients(1.8, 0, 0.06, 0.03))
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.15,0,0.00001,0.6,0.01))
            .centripetalScaling(0.0003)
            .mass(10.996);

    public static TwoWheelConstants localizerConstants = new TwoWheelConstants()
            // Match hardwareMap names defined in Robot.initHardware()
            .forwardEncoder_HardwareMapName("backRightMotor")
            .strafeEncoder_HardwareMapName("frontLeftMotor")
            .forwardPodY(-1.5)
            .strafePodX(-4.5)
            .forwardEncoderDirection(Encoder.FORWARD)
            .strafeEncoderDirection(Encoder.FORWARD)
            .forwardTicksToInches(0.0005351735451991927)
            .strafeTicksToInches(0.0005325156446866985)
            .IMU_HardwareMapName("imu")
            .IMU_Orientation(
                    new RevHubOrientationOnRobot(
                            RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                            RevHubOrientationOnRobot.UsbFacingDirection.UP
                    )
            );
    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 1);

    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(0.9)
            .xVelocity(56.37155256733736)
            .yVelocity(40.066178034378005)
            // Match hardwareMap names defined in Robot.initHardware()
            .rightFrontMotorName("frontRightMotor")
            .rightRearMotorName("backRightMotor")
            .leftRearMotorName("backLeftMotor")
            .leftFrontMotorName("frontLeftMotor")
            // Directions: reverse right side to match physical mounting
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .twoWheelLocalizer(localizerConstants)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .build();
    }
}
