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

package org.firstinspires.ftc.teamcode.autotests;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.auton.Constants;
import org.firstinspires.ftc.teamcode.auton.PIDFController;
import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.hardware.RobotDiagonal;
import org.openftc.easyopencv.OpenCvCamera;

/**
 This Op Mode is for use when we start from Red Alliance Station and Red cones stack
 on the left side of the robot.

 Path is:
 Read the signal on the cone and keep the id in the variable
 Strafe right by 12 inches, move forward by 18 inches, drop the cone in
 low junction.
 Based on the id on the cone strafe right or left and move forward by
 24 inches and park.

 */

@Autonomous(name="Robot: Alliance Red Cone Left", group="Robot")
@Disabled
public class AutonRedParking extends LinearOpMode {

    /* Declare OpMode members. */
    RobotDiagonal robot = new RobotDiagonal();
    private ElapsedTime     runtime = new ElapsedTime();
    //Elevator Code
    private static final PIDFController pid = new PIDFController(0.001, 0d, 0d, 0d);
    public static int setElevatorHeight = 0;
    public static final int maxElevatorHeight = 2850;


    String teleMsg = "";

    @Override
    public void runOpMode() {

        // Initialize the drive system variables.
        robot.init(hardwareMap,telemetry);
        //robot.setEncoderMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        teleMsg = "Starting robot and reset Encoders";

        // Send telemetry message to indicate successful Encoder reset
        telemetry.addLine(teleMsg);
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        boolean opMode = opModeIsActive();

        // Step through each leg of the path,
        // Note: Reverse movement is obtained by setting a negative distance (not speed)
//        encoderDrive(DRIVE_SPEED,  48,  48, 5.0);  // S1: Forward 47 Inches with 5 Sec timeout
//        encoderDrive(TURN_SPEED,   12, -12, 4.0);  // S2: Turn Right 12 Inches with 4 Sec timeout
//        encoderDrive(DRIVE_SPEED, -24, -24, 4.0);  // S3: Reverse 24 Inches with 4 Sec timeout
        Log.d("Test Logging","Logging from Main Method of autonomous...1");
        robot.strafe(Constants.AUTO_STRAFE_SPEED,12,5, opMode);
        sleep(1500);
        //setElevatorHeight = Constants.LOW_JUNCTION;
        setElevatorHeight = 500;
        double powerElevator = pid.calculate(robot.getElevatorPosition(), setElevatorHeight);
        telemetry.addLine(teleMsg+powerElevator+", "+setElevatorHeight);
        Log.d("Auto mode","Elevator Power: " + powerElevator);
//        telemetry.update();

        sleep(1500);
        robot.setElevatorPower(powerElevator);
        sleep(1500);

        robot.encoderDrive(Constants.AUTO_DRIVE_SPEED,6,6,30,opMode);
        sleep(2000);
        Log.d("Test Logging","Logging from Main Method of autonomous... 2");
        //robot.strafe(Constants.AUTO_STRAFE_SPEED,12,5, opMode);
        sleep(2000);

        telemetry.addLine("Path Complete");
        telemetry.update();
        sleep(1000);  // pause to display final telemetry message.
    }


}
