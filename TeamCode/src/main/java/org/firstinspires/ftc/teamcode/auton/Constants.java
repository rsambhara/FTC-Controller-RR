package org.firstinspires.ftc.teamcode.auton;

public class Constants {

    public static final double ROBOT_MOVE_STRAIGHT_Y = 0.8;
    public static final double ROBOT_TURN_X = 0.5;
    public static final double ROBOT_STRAFE_Y = 0.8;

    // Encoder distance mover constants
    // Calculate the COUNTS_PER_INCH for your specific drive train.
    // Go to your motor vendor website to determine your motor's COUNTS_PER_MOTOR_REV
    // For external drive gearing, set DRIVE_GEAR_REDUCTION as needed.
    // For example, use a value of 2.0 for a 12-tooth spur gear driving a 24-tooth spur gear.
    // This is gearing DOWN for less speed and more torque.
    // For gearing UP, use a gear ratio less than 1.0. Note this will affect the direction of wheel rotation.COUNTS_PER_MOTOR_REV
    public static final double     COUNTS_PER_MOTOR_REV    = 537.7 ;    // eg: TETRIX Motor Encoder
    public static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // No External Gearing.
    public static final double     WHEEL_DIAMETER_INCHES   = 3.7 ;     // For figuring circumference
    public static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    public static final double     DRIVE_SPEED             = 0.6;
    public static final double     TURN_SPEED              = 0.5;
    public static final double     AUTO_DRIVE_SPEED             = 0.5;
    public static final double     AUTO_TURN_SPEED              = 0.2;
    public static final double     AUTO_STRAFE_SPEED              = 0.5;
    public static final double     AUTO_ELEVATOR_SPEED              = 0.3;
    public static final double STRAFE_SLIP_FACTOR = 1.5;

    // Elevator constants
    /*
    public static final double LIFT_POWER_SCALE_FACTOR = 1.0;
    public static final double LIFT_POWER = 0.75;
    public static final int LIFT_RANGE = 1650;
     */
    public static final double COUNTS_PER_MOTOR_REV_ELEVATOR = 537.7;
    public static final double DRIVE_GEAR_REDUCTION_ELEVATOR   = 1.0;
    public static final double WHEEL_DIAMETER_INCHES_PULLEY = 1.5;
    public static final double COUNTS_PER_INCH_ELEVATOR         = (COUNTS_PER_MOTOR_REV_ELEVATOR * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES_PULLEY * 3.1415);
    public static final int HIGH_JUNCTION = 3100;
    public static final int MEDIUM_JUNCTION = 2351;
    public static final int LOW_JUNCTION = 1450;
    public static final int GROUND_LEVEL = 0;
    public static final double ANGLE_TURN_SPEED =0.3 ;
    public static final double ROBOT_TURN_ANGLE = 30;

    // Rotation by Angle
    // Width side to side is approximately 14.0 INCHES, wheel to wheel
    // Assume rotating about the centerline, so the circumference of a 360 degree rotation is diameter * pi
    // Use COUNTS_PER_INCH after calculating the circumference
    private static final double ROTATION_SLIP_FACTOR = 1.3;
    private static final double WIDTH = 14;
    public static final double COUNTS_PER_DEGREE = ((ROTATION_SLIP_FACTOR*WIDTH*Math.PI)*COUNTS_PER_INCH)/360;
    public static final double kP_TURN = 0.1d;
}
