/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

/*
 * This OpMode illustrates the concept of driving a path based on time.
 * The code is structured as a LinearOpMode
 *
 * The code assumes that you do NOT have encoders on the wheels,
 *   otherwise you would use: RobotAutoDriveByEncoder;
 *
 *   The desired path in this example is:
 *   - Drive forward for 3 seconds
 *   - Spin right for 1.3 seconds
 *   - Drive Backward for 1 Second
 *
 *  The code is written in a simple form with no optimizations.
 *  However, there are several ways that this type of sequence could be streamlined,
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@Autonomous(name="blue-far moveup 3ball", group="Robot")

public class blueSideFarMoveUp extends LinearOpMode {

    DcMotor frontLeftDrive;
    DcMotor frontRightDrive;
    DcMotor backLeftDrive;
    DcMotor backRightDrive;
    DcMotor shooterLeft;
    DcMotor intake;
    DcMotor shooterRight;
    Servo FlickLeft;
    Servo FlickRight;

    IMU imu;

    /* Declare OpMode members. */


    private ElapsedTime     runtime = new ElapsedTime();


    static final double     FORWARD_SPEED = 0.6;
    static final double     TURN_SPEED    = 0.5;

    @Override
    public void runOpMode() {

        // Initialize the drive system variables.
        frontLeftDrive = hardwareMap.get(DcMotor.class, "driveLeftFront");
        frontRightDrive = hardwareMap.get(DcMotor.class, "driveRightFront");
        backLeftDrive = hardwareMap.get(DcMotor.class, "driveLeftRear");
        backRightDrive = hardwareMap.get(DcMotor.class, "driveRightRear");

        FlickLeft = hardwareMap.get(Servo.class, "FlickLeft" );
        FlickRight = hardwareMap.get(Servo.class, "FlickRight");

        shooterLeft = hardwareMap.get(DcMotor.class, "shooterLeft");
        intake = hardwareMap.get(DcMotor.class, "intake");
        shooterRight = hardwareMap.get(DcMotor.class, "shooterRight");

        shooterRight.setDirection(DcMotor.Direction.REVERSE);

        shooterRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooterLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        FlickLeft.setPosition(0.85);
        FlickRight.setPosition(0.05);

        imu = hardwareMap.get(IMU.class, "imu");
        // This needs to be changed to match the orientation on your robot
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection =
                RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection =
                RevHubOrientationOnRobot.UsbFacingDirection.LEFT;

        RevHubOrientationOnRobot orientationOnRobot = new
                RevHubOrientationOnRobot(logoDirection, usbDirection);
        imu.initialize(new IMU.Parameters(orientationOnRobot));

        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // When run, this OpMode should start both motors driving forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        // backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        // frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Ready to run");    //
        telemetry.update();

        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);

        // Wait for the game to start (driver presses START)
        waitForStart();
        double angle=0;
        double targetAngle=0;
        double startAngle=0;
        double tolerance=0.02;

        sleep(20000);


        //drive backward for 2.5 seconds, and then stop for 1 se ond
        backLeftDrive.setPower(1);
        backRightDrive.setPower(1);
        frontLeftDrive.setPower(1);
        frontRightDrive.setPower(1);
        sleep(1500);
        frontLeftDrive.setPower(0);
        frontRightDrive.setPower(0);
        backLeftDrive.setPower(0);
        backRightDrive.setPower(0);
        sleep(500);

        //turn left 45 degrees
        angle = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        startAngle=angle;
        targetAngle=targetAngle-0.7853975;
        frontLeftDrive.setPower(-0.5);
        backLeftDrive.setPower(-0.5);
        frontRightDrive.setPower(0.5);
        backRightDrive.setPower(0.5);
        // 0.7853975 is 45 degrees
        while (Math.abs(angle-startAngle)<.5){
            angle = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        }
        frontLeftDrive.setPower(-0.25);
        backLeftDrive.setPower(-0.25);
        frontRightDrive.setPower(0.25);
        backRightDrive.setPower(0.25);
        while ((targetAngle-angle)>tolerance){
            angle = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        }

        frontLeftDrive.setPower(0);
        frontRightDrive.setPower(0);
        backLeftDrive.setPower(0);
        backRightDrive.setPower(0);
        sleep(250);

        //turn on shooters, and wait 3 seconds for it to spin up
        //turn on shooters, and wait 2 seconds for it to spin up
        shooterRight.setPower(0.6);
        shooterLeft.setPower(0.6);
        sleep(2000);
        //shoot the right
        FlickRight.setPosition(0.3);
        sleep(1000);
        //reset right and shoot left
        FlickRight.setPosition(0.05);
        FlickLeft.setPosition(0.575);
        sleep(500);
        //reset left
        FlickLeft.setPosition(0.87);
        sleep(500);
        //push third ball
        intake.setPower(-1);
        sleep(500);
        intake.setPower(0);
        //shoot both
        FlickRight.setPosition(0.3);
        sleep(500);
        FlickRight.setPosition(0.05);
        FlickLeft.setPosition(0.575);
        sleep(500);
        FlickLeft.setPosition(0.87);

        //stop the shooter
        shooterLeft.setPower(0);
        shooterRight.setPower(0);
        sleep(500);


        //drives 2 seconds forward, and stops
        frontLeftDrive.setPower(1);
        backLeftDrive.setPower(0);
        frontRightDrive.setPower(0);
        backRightDrive.setPower(1);
        sleep(1000);
        frontLeftDrive.setPower(0);
        frontRightDrive.setPower(0);
        backLeftDrive.setPower(0);
        backRightDrive.setPower(0);





        telemetry.addData("Path", "Complete");
        telemetry.update();
    }
}
