package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="Shooter PIDF Tuner")
public class shooterPIDFTuner extends LinearOpMode {
    DcMotorEx shooterLeft;
    DcMotor intake;
    DcMotorEx shooterRight;
    Servo FlickLeft;
    Servo FlickRight;
    public double highVelocity=1100;
    public double lowVelocity=800;
    double currentTargetVelocity=highVelocity;

    public double P=500;
    public double I=0;
    public double D=0;
    public double F=11;

    public boolean isFlywheelOn=true;

    public double increment=0.01;

    double flyWheelSpeed=1100;



    @Override public void runOpMode(){
        FlickLeft = hardwareMap.get(Servo.class, "FlickLeft" );
        FlickRight = hardwareMap.get(Servo.class, "FlickRight");

        shooterLeft = hardwareMap.get(DcMotorEx.class, "shooterLeft");
        intake = hardwareMap.get(DcMotor.class, "intake");
        shooterRight = hardwareMap.get(DcMotorEx.class, "shooterRight");

        shooterRight.setDirection(DcMotor.Direction.REVERSE);
        shooterRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooterLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        shooterRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(500,0,0,11);
        shooterLeft.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,pidfCoefficients);
        shooterRight.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,pidfCoefficients);


        telemetry.addLine("init done");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()){

            if (gamepad1.xWasPressed()){
                if (currentTargetVelocity==highVelocity){
                    currentTargetVelocity=lowVelocity;
                }else{
                    currentTargetVelocity=highVelocity;
                }
            }

            pidfCoefficients = new PIDFCoefficients(P,I,D,F);
            shooterLeft.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,pidfCoefficients);
            shooterRight.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,pidfCoefficients);


            if ((gamepad1.b)||(gamepad2.b)){
                isFlywheelOn=false;
            }

            if ((gamepad1.a)||(gamepad2.a)){
                isFlywheelOn=true;
            }

            if (isFlywheelOn){
                shooterLeft.setVelocity(currentTargetVelocity);
                shooterRight.setVelocity(currentTargetVelocity);
            }else{
                shooterLeft.setVelocity(0);
                shooterRight.setVelocity(0);
            }
            if (gamepad1.dpadUpWasPressed()){
                increment=increment*10;
            }else if (gamepad1.dpadDownWasPressed()){
                increment=increment/10;
            }


            if (gamepad1.dpadRightWasPressed()){
                F=F+increment;
            }else if (gamepad1.dpadLeftWasPressed()){
                F=F-increment;
            }



            if ((gamepad2.right_bumper)||(gamepad1.right_bumper)){
                //move RIGHT servo to the rotated position like 90 degrees prob
                FlickRight.setPosition(0.3);

            }else{
                //move RIGHT servo back to original position
                FlickRight.setPosition(0.05);
            }
            if ((gamepad2.left_bumper) || (gamepad1.left_bumper)){
                //move LEFT servo to the rotated position like 90 degrees prob
                FlickLeft.setPosition(0.575);
            }else{
                //move LEFT servo back to original position
                FlickLeft.setPosition(0.87);
            }

            if ((gamepad1.y)||(gamepad2.y)) {
                intake.setPower(-1);

            } else if ((gamepad1.dpad_up)||(gamepad2.dpad_up)){
                intake.setPower(0);
            }

            double currentVelocity=shooterLeft.getVelocity();
            double error =currentTargetVelocity-currentVelocity;

            telemetry.addData("target velocity",currentTargetVelocity);
            telemetry.addData("current velocity", "%.2f",currentVelocity);
            telemetry.addData("error", "%.2f",error);
            telemetry.addLine("-------------------------------------");
            telemetry.addData("P",P);
            telemetry.addData("I",I);
            telemetry.addData("D",D);
            telemetry.addData("F",F);
            telemetry.addData("increment",increment);
            telemetry.update();


        }
    }
}
