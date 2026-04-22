package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.pedropathing.util.Timer;

import org.firstinspires.ftc.teamcode.mechanisms.IntakeControl;
import org.firstinspires.ftc.teamcode.mechanisms.ShootSequence;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name="blueClose9BallPedro", group = "Concept")
public class pedroAuto12Ball extends OpMode {
    private Follower follower;
    private Timer pathTimer, opModeTimer;



    //====================== SHOOTER SEQUENCE ======================
    private ShootSequence shooter= new ShootSequence();
    private IntakeControl intake = new IntakeControl();
    private boolean shotsTriggered=false;



    public enum PathState {
        // START POSITION_END POSITION
        // DRIVE > MOVEMENT STATE
        // SHOOT > SCORING ARTIFACT
        DRIVE_STARTPOS_SHOOTPOS,
        SHOOT_PRELOAD,
        DRIVE_SHOOTONEPOS_SPIKEONEPOS,
        DRIVE_COLLECT_SPIKEONE,
        DRIVE_SPIKEONE_GATE,
        DRIVE_GATE_SHOOT2,
        DRIVE_SHOOT2_SPIKETWO,
        DRIVE_COLLECT_SPIKETWO,
        DRIVE_SPIKETWO_SHOOT3,
        DRIVE_SHOOT3_SPIKETHREE,
        DRIVE_COLLECT_SPIKETHREE,
        DRIVE_SPIKETHREE_SHOOT4,
        DRIVE_SHOOT4_PARK
    }
    PathState pathState;
    private final Pose startPose = new Pose(33,136, Math.toRadians(90));
    private final Pose shootPoseOne = new Pose(38.340,102.751, Math.toRadians(133));
    private final Pose spikeOnePose = new Pose(45.717,84.321,Math.toRadians(180));
    private final Pose collectSpikeOnePose = new Pose (15.547169811320757,84.321,Math.toRadians(180));

    private final Pose gate= new Pose(15.509000000000002,74.26415094339625,Math.toRadians(180));
    private final Pose gateControlPoint = new Pose (26.35849056603773,77.72641509433963);

    private final Pose shootPoseTwo = new Pose(38.340,102.751, Math.toRadians(133));
    private final Pose spikeTwoPose=new Pose(45.71698,59.849,Math.toRadians(180));
    private final Pose collectSpikeTwoPose=new Pose(17,59.849056,Math.toRadians(180));
    private final Pose shootPoseThree = new Pose(38.340,102.751, Math.toRadians(133));
    private final Pose spikeThreePose = new Pose(42.54716867924528,35.60377358490567,Math.toRadians(180));
    private final Pose collectSpikeThreePose=new Pose (10,35.60377358490567,Math.toRadians(180));
    private final Pose shootPoseFour = new Pose(38.340,102.751, Math.toRadians(133));
    private final Pose parkPose=new Pose(18.22641509433962,102.52830188679245,Math.toRadians(135));
    private PathChain driveStartToShoot1, driveShoot1ToSpike1, driveCollectSpike1, driveSpikeOneGate, driveGateToShoot2,driveShoot2ToSpike2, driveCollectSpike2,driveSpike2ToShoot3,driveShoot3ToSpike3,driveCollectSpike3,driveSpike3ToShoot4,driveShoot4ToPark;

    public void buildPaths() {
        //to build path, put in starting pose coords, and put in ending pose coords
        driveStartToShoot1 = follower.pathBuilder()
                .addPath(new BezierLine(startPose,shootPoseOne))
                .setLinearHeadingInterpolation(startPose.getHeading(),shootPoseOne.getHeading())
                .build();
        driveShoot1ToSpike1 = follower.pathBuilder()
                .addPath(new BezierLine(shootPoseOne,spikeOnePose))
                .setLinearHeadingInterpolation(shootPoseOne.getHeading(),spikeOnePose.getHeading())
                .build();
        driveCollectSpike1 = follower.pathBuilder()
                .addPath(new BezierLine(spikeOnePose,collectSpikeOnePose))
                .setLinearHeadingInterpolation(spikeOnePose.getHeading(),collectSpikeOnePose.getHeading())
                .build();
        driveSpikeOneGate = follower.pathBuilder()
                .addPath(new BezierCurve(collectSpikeOnePose,gateControlPoint,gate))
                .setLinearHeadingInterpolation(collectSpikeOnePose.getHeading(),gate.getHeading())
                .build();
        driveGateToShoot2 = follower.pathBuilder()
                .addPath(new BezierLine(gate,shootPoseTwo))
                .setLinearHeadingInterpolation(gate.getHeading(),shootPoseTwo.getHeading())
                .build();
        driveShoot2ToSpike2 = follower.pathBuilder()
                .addPath(new BezierLine(shootPoseTwo,spikeTwoPose))
                .setLinearHeadingInterpolation(shootPoseTwo.getHeading(),spikeTwoPose.getHeading())
                .build();
        driveCollectSpike2 = follower.pathBuilder()
                .addPath(new BezierLine(spikeTwoPose,collectSpikeTwoPose))
                .setLinearHeadingInterpolation(spikeTwoPose.getHeading(),collectSpikeTwoPose.getHeading())
                .build();
        driveSpike2ToShoot3 = follower.pathBuilder()
                .addPath(new BezierLine(collectSpikeTwoPose,shootPoseThree))
                .setLinearHeadingInterpolation(collectSpikeTwoPose.getHeading(),shootPoseThree.getHeading())
                .build();
        driveShoot3ToSpike3 = follower.pathBuilder()
                .addPath(new BezierLine(shootPoseThree,spikeThreePose))
                .setLinearHeadingInterpolation(shootPoseThree.getHeading(),spikeThreePose.getHeading())
                .build();
        driveCollectSpike3=follower.pathBuilder()
                .addPath(new BezierLine(spikeThreePose,collectSpikeThreePose))
                .setLinearHeadingInterpolation(spikeThreePose.getHeading(),collectSpikeThreePose.getHeading())
                .build();
        driveSpike3ToShoot4=follower.pathBuilder()
                .addPath(new BezierLine(collectSpikeThreePose,shootPoseFour))
                .setLinearHeadingInterpolation(collectSpikeThreePose.getHeading(), shootPoseFour.getHeading())
                .build();
        driveShoot4ToPark = follower.pathBuilder()
                .addPath(new BezierLine(shootPoseFour,parkPose))
                .setLinearHeadingInterpolation(shootPoseFour.getHeading(),parkPose.getHeading())
                .build();
    }

    public void statePathUpdate(){
        switch(pathState){
            case DRIVE_STARTPOS_SHOOTPOS:
                follower.followPath(driveStartToShoot1,true);
                setPathState(PathState.SHOOT_PRELOAD); //reset timer and set new state
                shooter.startSpinningUpFlywheel();
                telemetry.addLine("DRIVING");
                break;

            case SHOOT_PRELOAD:
                //TODO: THIS TOP CODE HAS FLYWHEEL LOGIC


                if (!follower.isBusy()){
                    telemetry.addLine("Done Path 1 (start to shoot pose 1)");
                    //have we requested shots?
                    if (!shotsTriggered){
                        shooter.fireShots(3);
                        shotsTriggered=true;
                    }
                    else if(shotsTriggered && !shooter.isBusy()){
                        //shots are done and we can transition
                        telemetry.addLine("Done Path 1 (done shooting)");
                        follower.followPath(driveShoot1ToSpike1);
                        setPathState(PathState.DRIVE_SHOOTONEPOS_SPIKEONEPOS);
                    }
                }




                //TODO: BELOW IS CODE THAT IS WITHOUT FLYWHEEL LOGIC
                //check if follower is done
                //and check that 5 seconds has elapsed

//                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds()>5){
//                    //TODO add flywheel logic here
//                    telemetry.addLine("Done Path 1 (start to shoot pose 1)");
//                    follower.followPath(driveShoot1ToSpike1,true);
//                    setPathState(PathState.DRIVE_SHOOTONEPOS_SPIKEONEPOS);
//                    //transition to next state
//                }



                break;
            case DRIVE_SHOOTONEPOS_SPIKEONEPOS:
                if (!follower.isBusy()){
                    telemetry.addLine("Done Path 2 (shoot to spike line one)");
                    intake.setIntakePower(-1);
                    follower.followPath(driveCollectSpike1,1,true);
                    setPathState(PathState.DRIVE_COLLECT_SPIKEONE);
                }
                break;
            case DRIVE_COLLECT_SPIKEONE:
                if (!follower.isBusy()){
                    telemetry.addLine("Done Path 3 (Collect spike one)");

                    follower.followPath(driveSpikeOneGate,1,true);
                    setPathState(PathState.DRIVE_SPIKEONE_GATE);
                }
                break;
            case DRIVE_SPIKEONE_GATE:
                if (!follower.isBusy()){
                    telemetry.addLine("Done Path 4 (hit gate)");

                    follower.followPath(driveGateToShoot2);
                    setPathState(PathState.DRIVE_GATE_SHOOT2);
                    shooter.startSpinningUpFlywheel();
                }
                break;
            case DRIVE_GATE_SHOOT2:
                if (!follower.isBusy()){
                    telemetry.addLine("Done Path 5 (go to shoot pos 2)");


                    if (!follower.isBusy()){
                        //have we requested shots?
                        if (!shotsTriggered){
                            shooter.fireShots(3);
                            shotsTriggered=true;
                            intake.setIntakePower(0);
                        }
                        else if(shotsTriggered && !shooter.isBusy()){
                            //shots are done and we can transition
                            telemetry.addLine("Done Path 5 (shoot 3)");

                            follower.followPath(driveShoot2ToSpike2);
                            setPathState(PathState.DRIVE_SHOOT2_SPIKETWO);
                        }
                    }


                }
                break;
            case DRIVE_SHOOT2_SPIKETWO:
                if (!follower.isBusy()){
                    telemetry.addLine("Done Path 6 (shoot 2 to spike two)");
                    intake.setIntakePower(-1);
                    follower.followPath(driveCollectSpike2,0.25,true);
                    setPathState(PathState.DRIVE_COLLECT_SPIKETWO);
                }
                break;
            case DRIVE_COLLECT_SPIKETWO:
                if (!follower.isBusy()){
                    telemetry.addLine("Done Path 7 (collect Spike 2)");

                    follower.followPath(driveSpike2ToShoot3,1,true);
                    shooter.startSpinningUpFlywheel();
                    setPathState(PathState.DRIVE_SPIKETWO_SHOOT3);
                }
            case DRIVE_SPIKETWO_SHOOT3:
                if (!follower.isBusy()){
                    telemetry.addLine("Done Path 8 (go to shoot pos 3)");


                    if (!follower.isBusy()){
                        //have we requested shots?
                        if (!shotsTriggered){
                            shooter.fireShots(3);
                            shotsTriggered=true;
                            intake.setIntakePower(0);
                        }
                        else if(shotsTriggered && !shooter.isBusy()){
                            //shots are done and we can transition
                            telemetry.addLine("Done Path 8 (shoot 3)");

                            follower.followPath(driveShoot3ToSpike3);
                            setPathState(PathState.DRIVE_SHOOT3_SPIKETHREE);


                        }
                    }


                }
                break;
            case DRIVE_SHOOT3_SPIKETHREE:
                if (!follower.isBusy()){
                    telemetry.addLine("Done Path 9 (shoot 3 to spike three)");
                    intake.setIntakePower(-1);
                    follower.followPath(driveCollectSpike3,1,true);
                    setPathState(PathState.DRIVE_COLLECT_SPIKETHREE);
                }
                break;
            case DRIVE_COLLECT_SPIKETHREE:
                if (!follower.isBusy()){
                    telemetry.addLine("Done Path 10 (collect spike 3)");
                    follower.followPath(driveSpike3ToShoot4,1,true);
                    shooter.startSpinningUpFlywheel();
                    setPathState(PathState.DRIVE_SPIKETHREE_SHOOT4);
                }
                break;
            case DRIVE_SPIKETHREE_SHOOT4:
                if (!follower.isBusy()) {
                    telemetry.addLine("Done Path 11 (go to shoot pos 4)");


                    if (!follower.isBusy()) {
                        //have we requested shots?
                        if (!shotsTriggered) {
                            shooter.fireShots(3);
                            shotsTriggered = true;
                            intake.setIntakePower(0);
                        } else if (shotsTriggered && !shooter.isBusy()) {
                            //shots are done and we can transition
                            telemetry.addLine("Done Path 11 (shoot 4)");

                            follower.followPath(driveShoot4ToPark);
                            setPathState(PathState.DRIVE_SHOOT4_PARK);


                        }
                    }
                }
                break;
            case DRIVE_SHOOT4_PARK:
                if (!follower.isBusy()){
                    telemetry.addLine("Done Path 12 (shoot 4 to park)");
                    telemetry.addLine("Done All paths");
                }

            default:
                telemetry.addLine("NO STATE COMMANDED");
                break;

        }
    }

    public void setPathState(PathState newState){
        pathState=newState;
        pathTimer.resetTimer();

        //when transitioning, we turn off shots triggered
        shotsTriggered=false;
    }

    @Override
    public void init() {
        pathState=PathState.DRIVE_STARTPOS_SHOOTPOS;
        pathTimer = new Timer();
        opModeTimer=new Timer();
        follower = Constants.createFollower(hardwareMap);

        // TODO add in other init mechanisms needed, flywheel, intake, blah blah blah
        shooter.init(hardwareMap);
        intake.init(hardwareMap);

        buildPaths();
        follower.setPose(startPose);
    }

    public void start(){
        opModeTimer.resetTimer();
        setPathState(pathState);
    }

    @Override
    public void loop() {
        follower.update();
        shooter.update();
        intake.update();
        statePathUpdate();

        telemetry.addData("path state", pathState.toString());
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.addData("path time", pathTimer.getElapsedTimeSeconds());

    }
}