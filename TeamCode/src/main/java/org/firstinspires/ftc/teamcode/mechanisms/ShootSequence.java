package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class ShootSequence {
    private DcMotor shooterLeft;
    private DcMotor intake;
    private DcMotor shooterRight;
    private Servo paddleLeft;
    private Servo paddleRight;

    private ElapsedTime stateTimer = new ElapsedTime();

    private enum ShooterState{
        IDLE,
        PRE_SPIN_UP,
        SPIN_UP,
        SHOOTLEFT,
        SHOOTRIGHT,
        KICKCENTER,
        SHOOTCENTER,
        RESETINDEXER

    }
    private ShooterState shooterState;

    // --------------- GATE LOGIC ----------------
    private double left_rest_position = .87;
    private double right_rest_position = .05;
    private double left_shoot_position = 0.575;
    private double right_shoot_position = 0.3;


    //TODO TUNE THESE TO BE FASTER
    private double paddle_shoot_time = 0.5;
    private double center_shoot_delay = 0.5;


    //reset time is the same for both
    private double paddle_reset_time = 1;


    // =============== FLYWHEEL ===============
    private int shotsRemaining = 0;
    private boolean preSpinFlywheel=false;
    private double flywheelVelocity=0;
    private double MIN_FLYWHEEL_RPM = 67676767;
    private double TARGET_FLYWHEEL_RPM=0.6;
    private double FLYWHEEL_MAX_SPINUP_TIME=3;

    public void init(HardwareMap hwMap){
        paddleLeft=hwMap.get(Servo.class, "FlickLeft");
        paddleRight=hwMap.get(Servo.class, "FlickRight");

        shooterLeft=hwMap.get(DcMotor.class,"shooterLeft");
        shooterRight=hwMap.get(DcMotor.class,"shooterRight");

        intake=hwMap.get(DcMotor.class,"intake");

        shooterRight.setDirection(DcMotor.Direction.REVERSE);
        shooterRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooterLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        //TODO FIND OUT HOW ENCODER WORKS AND CHANGE TO USING ENCODER
        shooterRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooterLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        shooterState = ShooterState.IDLE;

        shooterRight.setPower(0);
        shooterLeft.setPower(0);
        paddleLeft.setPosition(left_rest_position);
        paddleRight.setPosition(right_rest_position);
    }


    public void update(){
        switch (shooterState){
            case IDLE:
                if (preSpinFlywheel){
                    paddleLeft.setPosition(left_rest_position);
                    paddleRight.setPosition(right_rest_position);

                    //TODO MAKE FLYWHEEL ENCODER VELOCITY BASIC PID LOGIC
                    shooterRight.setPower(TARGET_FLYWHEEL_RPM);
                    shooterLeft.setPower(TARGET_FLYWHEEL_RPM);

                    stateTimer.reset();

                    shooterState=ShooterState.PRE_SPIN_UP;
                }
                if (shotsRemaining>0) {
                    paddleLeft.setPosition(left_rest_position);
                    paddleRight.setPosition(right_rest_position);

                    //TODO MAKE FLYWHEEL ENCODER VELOCITY BASIC PID LOGIC
                    shooterRight.setPower(TARGET_FLYWHEEL_RPM);
                    shooterLeft.setPower(TARGET_FLYWHEEL_RPM);

                    stateTimer.reset();

                    shooterState = ShooterState.SPIN_UP;
                }
                break;
            case PRE_SPIN_UP:
                if (stateTimer.seconds()>FLYWHEEL_MAX_SPINUP_TIME && shotsRemaining>0){
                    paddleLeft.setPosition(left_shoot_position);
                    stateTimer.reset();

                    //HERE WE COULD POTENTIALLY ADD LOGIC FOR APRIL TAG AND SORTING
                    shooterState=ShooterState.SHOOTLEFT;
                }
                break;
            case SPIN_UP:
                //velocity maybe idk not necesaary but like cool?
                if (flywheelVelocity>MIN_FLYWHEEL_RPM || stateTimer.seconds()>FLYWHEEL_MAX_SPINUP_TIME){
                    paddleLeft.setPosition(left_shoot_position);
                    stateTimer.reset();

                    //HERE WE COULD POTENTIALLY ADD LOGIC FOR APRIL TAG AND SORTING
                    shooterState=ShooterState.SHOOTLEFT;
                }
                break;
            case SHOOTLEFT:
                if (stateTimer.seconds()>paddle_shoot_time){
                    shotsRemaining--; //decrease shots remaining by 1
                    paddleRight.setPosition(right_shoot_position);
                    stateTimer.reset();

                    shooterState=ShooterState.SHOOTRIGHT;
                }
                break;
            case SHOOTRIGHT:
                if (stateTimer.seconds()>paddle_shoot_time){
                    shotsRemaining--; //decrease remaining shots by 1
                    paddleRight.setPosition(right_rest_position);
                    paddleLeft.setPosition(left_rest_position);

                    stateTimer.reset();
                    shooterState=ShooterState.RESETINDEXER;
                }
                break;
            case RESETINDEXER:
                if (stateTimer.seconds()>paddle_reset_time){
                    stateTimer.reset();

                    intake.setPower(-1);

                    shooterState=ShooterState.KICKCENTER;
                }
                break;
            case KICKCENTER:
                if (stateTimer.seconds()>center_shoot_delay){
                    stateTimer.reset();

                    paddleRight.setPosition(right_shoot_position);
                    paddleLeft.setPosition((left_shoot_position));

                    shooterState=ShooterState.SHOOTCENTER;
                }
                break;
            case SHOOTCENTER:
                if (stateTimer.seconds()>paddle_shoot_time){
                    stateTimer.reset();

                    shotsRemaining--;
                    shotsRemaining=0;

                    paddleLeft.setPosition(left_rest_position);
                    paddleRight.setPosition(right_rest_position);

                    shooterRight.setPower(0);
                    shooterLeft.setPower(0);
                    preSpinFlywheel=false;

                    intake.setPower(0);

                    shooterState=ShooterState.IDLE;
                }
                break;

        }
    }

    public void fireShots(int numberOfShots){
        if (shooterState==ShooterState.IDLE){
            shotsRemaining=numberOfShots;
        }
    }

    public void startSpinningUpFlywheel(){
        preSpinFlywheel=true;
    }

    public boolean isBusy(){
        return shooterState != ShooterState.IDLE;
    }
}

