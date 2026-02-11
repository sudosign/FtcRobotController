
package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.TelemetryManager;
import com.bylazar.telemetry.PanelsTelemetry;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.follower.Follower;
import com.pedropathing.paths.PathChain;
import com.pedropathing.geometry.Pose;

@Autonomous(name = "Pedro Pathing Autonomous", group = "Autonomous")
@Configurable // Panels
public class PedroAutonomous extends OpMode {
    private TelemetryManager panelsTelemetry; // Panels Telemetry instance
    public Follower follower; // Pedro Pathing follower instance
    private int pathState; // Current autonomous path state (state machine)
    private Paths paths; // Paths defined in the Paths class

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(72, 8, Math.toRadians(90)));

        paths = new Paths(follower); // Build paths

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void loop() {
        follower.update(); // Update Pedro Pathing
        //pathState =
        autonomousPathUpdate(); // Update autonomous state machine

        // Log values to Panels and Driver Station
        panelsTelemetry.debug("Path State", pathState);
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading", follower.getPose().getHeading());
        panelsTelemetry.update(telemetry);
    }


    public static class Paths {
        public PathChain line1;
        public PathChain line2;
        public PathChain line3;
        public PathChain line4;
        public PathChain line5;
        public PathChain line6;
        public PathChain line7;

        public Paths(Follower follower) {
            line1 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(21.000, 123.000),

                                    new Pose(48.000, 96.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(143), Math.toRadians(135))

                    .build();

            line2 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(48.000, 96.000),

                                    new Pose(41.032, 84.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(180))

                    .build();

            line3 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(41.032, 84.000),

                                    new Pose(18.194, 84.000)
                            )
                    ).setTangentHeadingInterpolation()

                    .build();

            line4 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(18.194, 84.000),

                                    new Pose(48.000, 96.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(135))

                    .build();

            line5 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(48.000, 96.000),
                                    new Pose(62.548, 53.839),
                                    new Pose(18.387, 59.419)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(180))

                    .build();

            line6 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(18.387, 59.419),

                                    new Pose(48.000, 96.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(135))

                    .build();

            line7 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(48.000, 96.000),

                                    new Pose(39.097, 40.065)
                            )
                    ).setTangentHeadingInterpolation()

                    .build();
        }
    }


    public void autonomousPathUpdate() {
        // Add your state machine Here
        // Access paths with paths.pathName
        // Refer to the Pedro Pathing Docs (Auto Example) for an example state machine
    }
}
    