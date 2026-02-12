package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "blueClose", group = "Comp")
@Configurable
public class blueClose extends OpMode {
    private TelemetryManager panelsTelemetry;
    private Follower follower;
    private Timer pathTimer;
    private Outtake outtake;
    private Intake intake;

    private int pathState;
    private boolean actionTriggered = false;

    private PathChain scorePreload, grabSample1, scoreSample1, grabSample2, scoreSample2, park;

    private Pose startPose = new Pose(9, 63, Math.toRadians(0));
    private Pose basketPose = new Pose(14, 129, Math.toRadians(45));
    private Pose pickup1Pose = new Pose(24, 120, Math.toRadians(90));
    private Pose pickup2Pose = new Pose(24, 130, Math.toRadians(90));
    private Pose parkPose = new Pose(40, 60, Math.toRadians(270));

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
        pathTimer = new Timer();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);

        outtake = new Outtake();
        outtake.init(hardwareMap);

        intake = new Intake();
        intake.init(hardwareMap);

        buildPaths();

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void start() {
        pathTimer.resetTimer();
        pathState = 0;
    }

    @Override
    public void loop() {
        follower.update();
        outtake.update();
        intake.update();
        autonomousPathUpdate();

        panelsTelemetry.debug("Path State", pathState);
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading", Math.toDegrees(follower.getPose().getHeading()));
        panelsTelemetry.debug("Time", pathTimer.getElapsedTimeSeconds());
        panelsTelemetry.update(telemetry);
    }

    private void buildPaths() {
        scorePreload = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(startPose.getX(), startPose.getY(), startPose.getHeading()),
                        new Pose(basketPose.getX(), basketPose.getY(), basketPose.getHeading())
                ))
                .setLinearHeadingInterpolation(startPose.getHeading(), basketPose.getHeading())
                .build();

        grabSample1 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(basketPose.getX(), basketPose.getY(), basketPose.getHeading()),
                        new Pose(pickup1Pose.getX(), pickup1Pose.getY(), pickup1Pose.getHeading())
                ))
                .setLinearHeadingInterpolation(basketPose.getHeading(), pickup1Pose.getHeading())
                .build();

        scoreSample1 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(pickup1Pose.getX(), pickup1Pose.getY(), pickup1Pose.getHeading()),
                        new Pose(basketPose.getX(), basketPose.getY(), basketPose.getHeading())
                ))
                .setLinearHeadingInterpolation(pickup1Pose.getHeading(), basketPose.getHeading())
                .build();

        grabSample2 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(basketPose.getX(), basketPose.getY(), basketPose.getHeading()),
                        new Pose(pickup2Pose.getX(), pickup2Pose.getY(), pickup2Pose.getHeading())
                ))
                .setLinearHeadingInterpolation(basketPose.getHeading(), pickup2Pose.getHeading())
                .build();

        scoreSample2 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(pickup2Pose.getX(), pickup2Pose.getY(), pickup2Pose.getHeading()),
                        new Pose(basketPose.getX(), basketPose.getY(), basketPose.getHeading())
                ))
                .setLinearHeadingInterpolation(pickup2Pose.getHeading(), basketPose.getHeading())
                .build();

        park = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(basketPose.getX(), basketPose.getY(), basketPose.getHeading()),
                        new Pose(parkPose.getX(), parkPose.getY(), parkPose.getHeading())
                ))
                .setLinearHeadingInterpolation(basketPose.getHeading(), parkPose.getHeading())
                .build();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                if (!actionTriggered) {
                    follower.followPath(scorePreload, true);
                    actionTriggered = true;
                }
                if (!follower.isBusy()) {
                    setPathState(1);
                }
                break;

            case 1:
                if (!actionTriggered) {
                    outtake.startShooter();
                    actionTriggered = true;
                }
                if (outtake.isReady() && actionTriggered) {
                    outtake.fire();
                    actionTriggered = false;
                }
                if (!outtake.isBusy()) {
                    setPathState(2);
                }
                break;

            case 2:
                if (!actionTriggered) {
                    follower.followPath(grabSample1, true);
                    actionTriggered = true;
                }
                if (follower.getPose().getX() > 18 && !intake.isBusy()) {
                    intake.startIntake(2.0);
                }
                if (!follower.isBusy()) {
                    setPathState(3);
                }
                break;

            case 3:
                if (!actionTriggered) {
                    intake.startIntake(1.5);
                    actionTriggered = true;
                }
                if (!intake.isBusy()) {
                    setPathState(4);
                }
                break;

            case 4:
                if (!actionTriggered) {
                    follower.followPath(scoreSample1, true);
                    actionTriggered = true;
                }
                if (follower.getPose().getX() < 20 && !outtake.isBusy()) {
                    outtake.startShooter();
                }
                if (!follower.isBusy()) {
                    setPathState(5);
                }
                break;

            case 5:
                if (!actionTriggered) {
                    if (!outtake.isBusy()) {
                        outtake.startShooter();
                    }
                    actionTriggered = true;
                }
                if (outtake.isReady()) {
                    outtake.fire();
                    actionTriggered = false;
                }
                if (!outtake.isBusy()) {
                    setPathState(6);
                }
                break;

            case 6:
                if (!actionTriggered) {
                    follower.followPath(grabSample2, true);
                    actionTriggered = true;
                }
                if (follower.getPose().getX() > 18 && !intake.isBusy()) {
                    intake.startIntake(2.0);
                }
                if (!follower.isBusy()) {
                    setPathState(7);
                }
                break;

            case 7:
                if (!actionTriggered) {
                    intake.startIntake(1.5);
                    actionTriggered = true;
                }
                if (!intake.isBusy()) {
                    setPathState(8);
                }
                break;

            case 8:
                if (!actionTriggered) {
                    follower.followPath(scoreSample2, true);
                    actionTriggered = true;
                }
                if (follower.getPose().getX() < 20 && !outtake.isBusy()) {
                    outtake.startShooter();
                }
                if (!follower.isBusy()) {
                    setPathState(9);
                }
                break;

            case 9:
                if (!actionTriggered) {
                    if (!outtake.isBusy()) {
                        outtake.startShooter();
                    }
                    actionTriggered = true;
                }
                if (outtake.isReady()) {
                    outtake.fire();
                    actionTriggered = false;
                }
                if (!outtake.isBusy()) {
                    setPathState(10);
                }
                break;

            case 10:
                if (!actionTriggered) {
                    follower.followPath(park, false);
                    actionTriggered = true;
                }
                if (!follower.isBusy()) {
                    setPathState(11);
                }
                break;

            case 11:
                if (!actionTriggered) {
                    intake.stop();
                    outtake.stopShooter();
                    actionTriggered = true;
                }
                if (pathTimer.getElapsedTimeSeconds() > 0.5) {
                    setPathState(-1);
                }
                break;
        }
    }

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
        actionTriggered = false;
    }
}
