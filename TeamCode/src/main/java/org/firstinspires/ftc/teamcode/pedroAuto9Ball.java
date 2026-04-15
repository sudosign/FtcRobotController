package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.pedropathing.util.Timer;

import org.firstinspires.ftc.teamcode.mechanisms.IntakeControl;
import org.firstinspires.ftc.teamcode.mechanisms.ShootSequence;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name="blueClose9BallPedro", group = "Concept")
public class pedroAuto9Ball extends OpMode {
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
        DRIVE_GATEONE_GATETWO,
        DRIVE_GATE_SHOOT2,
        SHOOT_2
    }
    PathState pathState;
    private final Pose startPose = new Pose(33,136, Math.toRadians(90));
    private final Pose shootPoseOne = new Pose(38.340,102.751, Math.toRadians(133));
    private final Pose spikeOnePose = new Pose(45.717,84.321,Math.toRadians(180));
    private final Pose collectSpikeOnePose = new Pose (16,84.321,Math.toRadians(180));
    private final Pose shootPoseTwo = new Pose(38.340,102.751, Math.toRadians(133));
    private final Pose gate=new Pose(21.5094,71.0943,Math.toRadians(180));
    private final Pose gateTwo=new Pose(15.509,71.075,Math.toRadians(180));

    private PathChain driveStartToShoot1, driveShoot1ToSpike1, driveCollectSpike1, driveGateToShoot2,driveSpikeOneGate,driveGateOneGateTwo;

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
        driveGateToShoot2 = follower.pathBuilder()
                .addPath(new BezierLine(gateTwo,shootPoseTwo))
                .setLinearHeadingInterpolation(gateTwo.getHeading(),shootPoseTwo.getHeading())
                .build();
        driveSpikeOneGate = follower.pathBuilder()
                .addPath(new BezierLine(collectSpikeOnePose,gate))
                .setLinearHeadingInterpolation(collectSpikeOnePose.getHeading(),gate.getHeading())
                .build();
        driveGateOneGateTwo = follower.pathBuilder()
                .addPath(new BezierLine(gate,gateTwo))
                .setLinearHeadingInterpolation(gate.getHeading(),gateTwo.getHeading())
                .build();
    }

    public void statePathUpdate(){
        switch(pathState){
            case DRIVE_STARTPOS_SHOOTPOS:
                follower.followPath(driveStartToShoot1,true);
                setPathState(PathState.SHOOT_PRELOAD); //reset timer and set new state
                break;
            case SHOOT_PRELOAD:

                //TODO: THIS TOP CODE HAS FLYWHEEL LOGIC


                if (!follower.isBusy()){
                    //have we requested shots?
                    if (!shotsTriggered){
                        shooter.fireShots(3);
                        shotsTriggered=true;
                    }
                    else if(shotsTriggered && !shooter.isBusy()){
                        //shots are done and we can transition
                        telemetry.addLine("Done Path 1 (start to shoot pose 1)");
                        follower.followPath(driveShoot1ToSpike1,true);
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
                    follower.followPath(driveCollectSpike1, true);
                    setPathState(PathState.DRIVE_COLLECT_SPIKEONE);
                }
                break;
            case DRIVE_COLLECT_SPIKEONE:
                if (!follower.isBusy()){
                    telemetry.addLine("Done Path 3 (Collect spike one)");

                    follower.followPath(driveSpikeOneGate);
                    setPathState(PathState.DRIVE_SPIKEONE_GATE);
                }
                break;
            case DRIVE_SPIKEONE_GATE:
                if (!follower.isBusy()){
                    telemetry.addLine("Done Path 4 (hit gate)");

                    follower.followPath(driveGateOneGateTwo);
                    setPathState(PathState.DRIVE_GATEONE_GATETWO);
                }
                break;
            case DRIVE_GATEONE_GATETWO:
                if(!follower.isBusy()){
                    telemetry.addLine("Done Path 5 (hit gate 2)");

                    follower.followPath(driveGateToShoot2);
                    setPathState(PathState.DRIVE_GATE_SHOOT2);
                }
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
                            telemetry.addLine("Done all paths");
                        }
                    }


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