package org.firstinspires.ftc.teamcode.pedroPathing;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.PredictiveBrakingCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.Encoder;
import com.pedropathing.ftc.localization.constants.TwoWheelConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.digitalchickenlabs.OctoQuad;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Configurable
public class Constants {
    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(11.22)
            .headingPIDFCoefficients(new PIDFCoefficients(0.8,0,0.02,0.025))
            .predictiveBrakingCoefficients(new PredictiveBrakingCoefficients(0.07,0.1023153894736,0.00103368066))
            .centripetalScaling(0);

    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1) // TUNE
            .rightFrontMotorName("driveRightFront")
            .rightRearMotorName("driveRightRear")
            .leftRearMotorName("driveLeftRear")
            .leftFrontMotorName("driveLeftFront")
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .xVelocity(69.55331458)
            .yVelocity(56.9114796648);

    public static TwoWheelConstants localizerConstants = new TwoWheelConstants()
            .forwardEncoder_HardwareMapName("driveRightFront")//port 0, CHANGE THIS PROBABLY
            .strafeEncoder_HardwareMapName("driveLeftRear")//port 3
            .IMU_HardwareMapName("imu")
            .IMU_Orientation(
                    new RevHubOrientationOnRobot(
                            RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD,
                            RevHubOrientationOnRobot.UsbFacingDirection.LEFT
                    )
            )
            .forwardPodY(-8.90625) // for some reason we negated this
            .strafePodX(-8.125)
            .forwardEncoderDirection(Encoder.FORWARD)
            .forwardTicksToInches(0.002968434004)
            .strafeTicksToInches(0.002968434004);
//            .forwardTicksToInches(); 0.0029416303, 81.15649

    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 1);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .twoWheelLocalizer(localizerConstants)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .build();
    }
}
