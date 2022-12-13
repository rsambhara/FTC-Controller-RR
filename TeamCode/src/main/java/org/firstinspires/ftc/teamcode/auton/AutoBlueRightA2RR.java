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

package org.firstinspires.ftc.teamcode.auton;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.hardware.RobotElevator;
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
@Config
@Autonomous(name="1RR - Blue Right A2 - Red Right F5", group="Robot")
public class AutoBlueRightA2RR extends LinearOpMode {

    private String PARK_POS = "LEFT";
    /* Declare OpMode members. */
    RobotElevator robot = new RobotElevator();

    private ElapsedTime     runtime = new ElapsedTime();
    //Elevator Code
    private static final PIDFController pid = new PIDFController(0.001, 0d, 0d, 0d);
    public static int setElevatorHeight = 0;
    public static final int maxElevatorHeight = 2850;

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

    //Trajectory Constants
    public static double stfeRight1Low = 14.0;
    public static double fwd1Low = 5.6;
    public static double back1Low = 4.0;
    public static double loc1StfePark = 15.0;
    public static double loc1DrivePark = 24.0;
    public static double loc2StfePark =15;
    public static double loc2DrivePark = 30.0;
    public static double loc3StfePark =38.0;
    public static double loc3DrivePark = 30.0;
    public static double turnToDriver = Math.toRadians(180.0);

    @Override
    public void runOpMode() {

        // Initialize the drive system variables.
        robot.init(hardwareMap,telemetry);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        //robot.setEncoderMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        teleMsg = "Starting robot and reset Encoders";
        double elevatorPower =0.0;

        // Send telemetry message to indicate successful Encoder reset
        telemetry.addLine(teleMsg);
        telemetry.update();

        // Camera testing
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(800,448, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {

            }
        });

        telemetry.setMsTransmissionInterval(50);
        /*
         * The INIT-loop:
         * This REPLACES waitForStart!
         */
        while (!isStarted() && !isStopRequested())
        {
            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

            if(currentDetections.size() != 0)
            {
                boolean tagFound = false;

                for(AprilTagDetection tag : currentDetections)
                {
                    if(tag.id == LEFT || tag.id == MIDDLE || tag.id == RIGHT)
                    {
                        tagOfInterest = tag;
                        tagFound = true;
                        break;
                    }
                }

                if(tagFound)
                {
                    telemetry.addLine("Tag of interest is in sight!\n\nLocation data:");
                    tagToTelemetry(tagOfInterest);
                }
                else
                {
                    telemetry.addLine("Don't see tag of interest :(");

                    if(tagOfInterest == null)
                    {
                        telemetry.addLine("(The tag has never been seen)");
                    }
                    else
                    {
                        telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                        tagToTelemetry(tagOfInterest);
                    }
                }

            }
            else
            {
                telemetry.addLine("Don't see tag of interest :(");

                if(tagOfInterest == null)
                {
                    telemetry.addLine("(The tag has never been seen)");
                }
                else
                {
                    telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                    tagToTelemetry(tagOfInterest);
                }

            }

            telemetry.update();
            sleep(20);
        }


        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        boolean opMode = opModeIsActive();

        /*
         * The START command just came in: now work off the latest snapshot acquired
         * during the init loop.
         */

        /* Update the telemetry */
        if(tagOfInterest != null)
        {
            telemetry.addLine("Tag snapshot:\n");
            tagToTelemetry(tagOfInterest);
            telemetry.update();
        }
        else
        {
            telemetry.addLine("No tag snapshot available, it was never sighted during the init loop :(");
            telemetry.update();
        }

        /* Based on Tag, follow specific trajectory path */
        if(tagOfInterest == null || tagOfInterest.id == LEFT){
            //trajectory
            PARK_POS = "LEFT";
            telemetry.addLine("Park in Left (Id - 1)");
            telemetry.update();
        }
        else if(tagOfInterest.id == MIDDLE){
            //trajectory
            telemetry.addLine("Park in Middle (Id -2 )");
            telemetry.update();
            PARK_POS = "MIDDLE";
        }
        else{
            //trajectory
            PARK_POS = "RIGHT";
            telemetry.addLine("Park in Right (Id -3)");
            telemetry.update();
        }

        // Task 1 go for LOW Junction close by
        //Tighten the claw position
        robot.setClawPower(-0.45);
        sleep(100);
        // Elevator lift for low junction
        runtime = new ElapsedTime();
        while(opModeIsActive() && !isStopRequested() && (runtime.seconds() < 2)){
            setElevatorHeight = Constants.LOW_JUNCTION;
            elevatorPower = pid.calculate(robot.getElevatorPosition(),setElevatorHeight);
            robot.setElevatorPower(elevatorPower);
        }
        //Strafe right to go opposite to low junction
        Trajectory trajStfe1 = drive.trajectoryBuilder(new Pose2d())
                .strafeLeft(stfeRight1Low)//3
                .build();
        drive.followTrajectory(trajStfe1);

        //Strafe right to go opposite to low junction
        Trajectory trajFwd1 = drive.trajectoryBuilder(trajStfe1.end())
                .forward(fwd1Low)//3
                .build();
        drive.followTrajectory(trajFwd1);
        // Drop the cone in the junction
        runtime = new ElapsedTime();
        while(opModeIsActive() && !isStopRequested() && (runtime.seconds() < 2)){
            setElevatorHeight = (int) robot.getElevatorPosition() - 5;
            elevatorPower = pid.calculate(robot.getElevatorPosition(),setElevatorHeight);
            robot.setElevatorPower(elevatorPower);
        }
        robot.setClawPower(0.25);
        sleep(150);
        // Go back and lower the elevator
        //Move back from low junction
        Trajectory trajBack1 = drive.trajectoryBuilder(trajFwd1.end())
                .back(back1Low)//3
                .build();
        drive.followTrajectory(trajBack1);
        runtime = new ElapsedTime();
        while(opModeIsActive() && !isStopRequested() && (runtime.seconds() < 2)){
            setElevatorHeight = 0;
            //TODO: check if it is going to base, if not increase current position height
            elevatorPower = pid.calculate(robot.getElevatorPosition(),setElevatorHeight);
            robot.setElevatorPower(elevatorPower);
        }

        /* Actually do something useful */
        if(tagOfInterest == null || tagOfInterest.id == LEFT){

            //Strafe left to go to Location 1
            Trajectory trajLoc1Stfe = drive.trajectoryBuilder(trajBack1.end().plus(new Pose2d()))
                    .strafeLeft(loc1StfePark)
                    .build();
            drive.followTrajectory(trajLoc1Stfe);
            sleep(50);
            Trajectory trajLoc1Park = drive.trajectoryBuilder(trajLoc1Stfe.end().plus(new Pose2d()))
                    .forward(loc1DrivePark)
                    .build();
            drive.followTrajectory(trajLoc1Park);
            drive.turn(turnToDriver);

        }else if(tagOfInterest.id == MIDDLE){
            //trajectory
            // Park middle
            //Strafe left to go to Location 2
            Trajectory trajLoc2Stfe = drive.trajectoryBuilder(trajBack1.end().plus(new Pose2d()))
                    .strafeLeft(loc2StfePark)
                    .build();
            drive.followTrajectory(trajLoc2Stfe);
            sleep(50);
            Trajectory trajLoc2Park = drive.trajectoryBuilder(trajLoc2Stfe.end().plus(new Pose2d()))
                    .forward(loc2DrivePark)
                    .build();
            drive.followTrajectory(trajLoc2Park);
            drive.turn(turnToDriver);

        }else{
            //trajectory
            // Park right.
            telemetry.addLine("Park in right id is 3");
            //Strafe left to go to Location 2
            Trajectory trajLoc3Stfe = drive.trajectoryBuilder(trajBack1.end().plus(new Pose2d()))
                    .strafeRight(loc3StfePark)
                    .build();
            drive.followTrajectory(trajLoc3Stfe);
            sleep(50);
            Trajectory trajLoc3Park = drive.trajectoryBuilder(trajLoc3Stfe.end().plus(new Pose2d()))
                    .forward(loc3DrivePark)
                    .build();
            drive.followTrajectory(trajLoc3Park);
            drive.turn(turnToDriver);
        }


        /* You wouldn't have this in your autonomous, this is just to prevent the sample from ending */
        while (opModeIsActive()) {sleep(20);}



    }

    void tagToTelemetry(AprilTagDetection detection)
    {
        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
        telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z*FEET_PER_METER));
        telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)));
        telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)));
        telemetry.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)));
    }

}
