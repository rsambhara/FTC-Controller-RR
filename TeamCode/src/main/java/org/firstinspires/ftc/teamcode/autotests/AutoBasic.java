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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.auton.AprilTagDetectionPipeline;
import org.firstinspires.ftc.teamcode.auton.PIDFController;
import org.firstinspires.ftc.teamcode.auton.TurnPIDController;
import org.firstinspires.ftc.teamcode.hardware.RobotDiagonal;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

/**
 This Op Mode is for use when we start from Red Alliance Station and Red cones stack
 on the left side of the robot.

 Path is:
 Read the signal on the cone and keep the id in the variable
 Strafe right by 12 inches, move forward by 8 inches, drop the cone in
 low junction.
 Based on the id on the cone strafe right or left and move forward by
 24 inches and park.

 */

@Autonomous(name="Basic OP Mode", group="Robot")
public class AutoBasic extends LinearOpMode {

    /* Declare OpMode members. */
    RobotDiagonal robot = new RobotDiagonal();
    private ElapsedTime     runtime = new ElapsedTime();
    //Elevator Code
    private static final PIDFController pid = new PIDFController(0.001, 0d, 0d, 0d);
    public static int setElevatorHeight = 0;
    public static final int maxElevatorHeight = 2850;
    TurnPIDController turnPID = new TurnPIDController(0.03, 0, 0.003);

    int cwise = -1;
    int c_cwise = 1;

    // Tag detection Code
    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;
    static final double FEET_PER_METER = 3.28084;
    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    // UNITS ARE METERS
    double tagsize = 0.166;

    // Tag ID 1,2,3 from the 36h11 family
    int LEFT = 1;
    int MIDDLE = 2;
    int RIGHT = 3;

    AprilTagDetection tagOfInterest = null;


    String teleMsg = "";

    @Override
    public void runOpMode() {

        // Initialize the drive system variables.
        robot.init(hardwareMap,telemetry);
        //robot.setEncoderMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        teleMsg = "Starting robot and reset Encoders";
        double elevatorPower =0.0;

        // Send telemetry message to indicate successful Encoder reset
        telemetry.addLine(teleMsg);
        telemetry.update();
        boolean opMode = !isStopRequested();

        /*
        double r_error = error;
        if Math.abs(r_error < 1) r_error = 0;
         */

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        telemetry.addData("Starting turn now",opMode);
        telemetry.update();
        //robot.Rotate(0.5,180,opMode);
//        robot.resetAngle();

        robot.gyroTurn(telemetry,opMode,.4,90 * cwise);
        sleep(3000);
//        robot.gyroTurn(telemetry,opMode,.4,90 * cwise);
//        turnPID.setTargetAngle(90 * -1);
//        sleep(3000);
//        robot.gyroTurn(telemetry,opMode,.4,90 * cwise);
//        turnPID.setTargetAngle(90 * -1);
        sleep(3000);
        robot.gyroTurn(telemetry,opMode,.4,90 * cwise);

        turnPID.setTargetAngle(90 * -1);
        telemetry.addData("Starting PID turn now",opMode);
        telemetry.update();
        //robot.turnAbsolutePID(turnPID,true);



        /* You wouldn't have this in your autonomous, this is just to prevent the sample from ending */
        while (opModeIsActive()) {sleep(50);}



    }



}
