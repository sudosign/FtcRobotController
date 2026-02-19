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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

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

@Autonomous(name="blue-close 2ball", group="Robot")

public class blueSideClose extends LinearOpMode {

    DcMotor frontLeftDrive;
    DcMotor frontRightDrive;
    DcMotor backLeftDrive;
    DcMotor backRightDrive;
    DcMotor shooterLeft;
    DcMotor intake;
    DcMotor shooterRight;
    Servo FlickLeft;
    Servo FlickRight;

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

        FlickLeft.setPosition(0.835);
        FlickRight.setPosition(0.035);

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



        //drive backward for 2.5 seconds, and then stop for 1 se ond
        backLeftDrive.setPower(-.25);
        backRightDrive.setPower(-.25);
        frontLeftDrive.setPower(-.25);
        frontRightDrive.setPower(-.25);
        sleep(2500);
        frontLeftDrive.setPower(0);
        frontRightDrive.setPower(0);
        backLeftDrive.setPower(0);
        backRightDrive.setPower(0);
        sleep(1000);


        //turn on shooters, and wait 3 seconds for it to spin up
        shooterRight.setPower(0.6);
        shooterLeft.setPower(0.6);
        sleep(3000);

        //shoot the right
        FlickRight.setPosition(0.3);
        sleep(500);
        FlickRight.setPosition(0.035);
        sleep(500);

        //shoot the left
        FlickLeft.setPosition(0.575);
        sleep(500);
        FlickLeft.setPosition(0.835);
        sleep(500);

        intake.setPower(-1);
        sleep(1000);
        intake.setPower(0);


        //reshoot the right
        FlickRight.setPosition(0.3);
        sleep(500);
        FlickRight.setPosition(0.035);
        sleep(500);

        //reshoot the left
        FlickLeft.setPosition(0.575);
        sleep(500);
        FlickLeft.setPosition(0.835);
        sleep(500);


        //stop the shooter
        shooterLeft.setPower(0);
        shooterRight.setPower(0);
        sleep(2000);

        //turns right for 1 second, stops, and waits 1 second
        frontLeftDrive.setPower(0.25);
        backLeftDrive.setPower(0.25);
        frontRightDrive.setPower(-0.25);
        backRightDrive.setPower(-0.25);
        sleep(1000);
        frontLeftDrive.setPower(0);
        frontRightDrive.setPower(0);
        backLeftDrive.setPower(0);
        backRightDrive.setPower(0);
        sleep(1000);


        //drives 2 seconds forward, and stops
        frontLeftDrive.setPower(0.25);
        backLeftDrive.setPower(0.25);
        frontRightDrive.setPower(0.25);
        backRightDrive.setPower(0.25);
        sleep(2000);
        frontLeftDrive.setPower(0);
        frontRightDrive.setPower(0);
        backLeftDrive.setPower(0);
        backRightDrive.setPower(0);


        // Step 1:  Drive forward for 3 seconds
        // leftDrive.setPower(FORWARD_SPEED);
        // rightDrive.setPower(FORWARD_SPEED);
        // runtime.reset();
        // while (opModeIsActive() && (runtime.seconds() < 3.0)) {
        //     telemetry.addData("Path", "Leg 1: %4.1f S Elapsed", runtime.seconds());
        //     telemetry.update();
        // }

        // Step 2:  Spin right for 1.3 seconds
        // leftDrive.setPower(TURN_SPEED);
        // rightDrive.setPower(-TURN_SPEED);
        // runtime.reset();
        // while (opModeIsActive() && (runtime.seconds() < 1.3)) {
        //     telemetry.addData("Path", "Leg 2: %4.1f S Elapsed", runtime.seconds());
        //     telemetry.update();
        // }

        // Step 3:  Drive Backward for 1 Second
        // leftDrive.setPower(-FORWARD_SPEED);
        // rightDrive.setPower(-FORWARD_SPEED);
        // runtime.reset();
        // while (opModeIsActive() && (runtime.seconds() < 1.0)) {
        //     telemetry.addData("Path", "Leg 3: %4.1f S Elapsed", runtime.seconds());
        //     telemetry.update();
        // }

        // Step 4:  Stop



        telemetry.addData("Path", "Complete");
        telemetry.update();
        sleep(1000);
    }
}
