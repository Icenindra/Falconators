

package org.firstinspires.ftc.teamcode.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import java.util.Objects;

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

@Autonomous(name="AutoDriveByTime")
public class time extends LinearOpMode{

    /* Declare OpMode members. */
    private DcMotor         leftFrontDrive   = null;
    private DcMotor         rightFrontDrive  = null;
    private DcMotor         leftBottomDrive = null;
    private DcMotor         rightBottomDrive = null;



    static final double     FORWARD_SPEED = 0.6;
    static final double     TURN_SPEED    = 0.5;
    private double degrees =0;
    private boolean isStraight = false;
    final double DESIRED_DISTANCE = 5.0; // how close
    final double SPEED_GAIN  =  0.02  ;   //  Forward Speed Control "Gain". eg: Ramp up to 50% power at a 25 inch error.   (0.50 / 25.0)
    final double STRAFE_GAIN =  0.015 ;   //  Strafe Speed Control "Gain".  eg: Ramp up to 25% power at a 25 degree Yaw error.   (0.25 / 25.0)
    final double TURN_GAIN   =  0.01  ;   //  Turn Control "Gain".  eg: Ramp up to 25% power at a 25 degree error. (0.25 / 25.0)
    final double MAX_AUTO_SPEED = 0.5;   //  Clip the approach speed to this max value
    final double MAX_AUTO_STRAFE= 0.5;   //  Clip the approach speed to this max value
    final double MAX_AUTO_TURN  = 0.3;   //  Clip the turn speed to this max value
    boolean targetSaw = false;
    private DcMotor lift = null;
    private Servo door = null;
    private Servo intakeRight = null;
    private Servo intakeLeft = null;
    @Override
    public void runOpMode() throws InterruptedException {
        cameraSystem camerasystem = new cameraSystem();
        camerasystem.initAprilTag();
        leftFrontDrive = hardwareMap.get(DcMotor.class, "leftFrontDrive");
        leftBottomDrive = hardwareMap.get(DcMotor.class, "leftBottomDrive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "rightFrontDrive");
        rightBottomDrive = hardwareMap.get(DcMotor.class, "rightBottomDrive");
        telemetry.addData("Status of all systems", "Initialized, may press start now");
        telemetry.update();
        waitForStart();

        moveForward(2000);
        turnRight(1500);
        driveToAprilTag(camerasystem.updateAprilTag());
    }
    public void setDegrees(double deg){
        this.degrees = deg;
    }
    public double getDegrees(){
        return this.degrees;
    }
    public void setisStraight(boolean deg){
        this.isStraight = deg;
    }

    public void moveRobot(double x, double y, double yaw) {
        direction("forward");
        // Calculate wheel powers.
        double leftFrontPower    =  x -y -yaw;
        double rightFrontPower   =  x +y +yaw;
        double leftBackPower     =  x +y -yaw;
        double rightBackPower    =  x -y +yaw;

        // Normalize wheel powers to be less than 1.0
        double max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));

        if (max > 1.0) {
            leftFrontPower /= max;
            rightFrontPower /= max;
            leftBackPower /= max;
            rightBackPower /= max;
        }

        // Send powers to the wheels.
        leftFrontDrive.setPower(leftFrontPower);
        rightFrontDrive.setPower(rightFrontPower);
        leftBottomDrive.setPower(leftBackPower);
        rightBottomDrive.setPower(rightBackPower);
    }
    public void direction(String direct) {
        if (Objects.equals(direct, "forward")) {
            leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
            leftBottomDrive.setDirection(DcMotor.Direction.REVERSE);
            rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
            rightBottomDrive.setDirection(DcMotor.Direction.FORWARD);
        } else if (Objects.equals(direct, "backward")) {
            leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
            leftBottomDrive.setDirection(DcMotor.Direction.FORWARD);
            rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
            rightBottomDrive.setDirection(DcMotor.Direction.REVERSE);
        } else if (Objects.equals(direct, "right")) {
            leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
            leftBottomDrive.setDirection(DcMotor.Direction.FORWARD);
            rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
            rightBottomDrive.setDirection(DcMotor.Direction.FORWARD);
        } else if (Objects.equals(direct, "left")) {
            leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
            leftBottomDrive.setDirection(DcMotor.Direction.REVERSE);
            rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
            rightBottomDrive.setDirection(DcMotor.Direction.REVERSE);
        } else if (Objects.equals(direct, "clockwise")) {
            leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
            leftBottomDrive.setDirection(DcMotor.Direction.REVERSE);
            rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
            rightBottomDrive.setDirection(DcMotor.Direction.REVERSE);
        } else if (Objects.equals(direct, "counterClockwise")) {
            leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
            leftBottomDrive.setDirection(DcMotor.Direction.FORWARD);
            rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
            rightBottomDrive.setDirection(DcMotor.Direction.FORWARD);
                } else if (Objects.equals(direct, "up")) {
                    lift.setDirection(DcMotorSimple.Direction.REVERSE);
                } else if (Objects.equals(direct, "down")) {
                    lift.setDirection(DcMotorSimple.Direction.FORWARD);
                } else if (Objects.equals(direct, "push")) {
                    door.setDirection(Servo.Direction.REVERSE);
              } else if (Objects.equals(direct, "pull")) {
                  door.setDirection(Servo.Direction.FORWARD);
              }
        }
    public void driveToAprilTag(double[] values) {
        boolean targetFound;
        targetFound = (values[3] == 1);
        double drive = 0;        // Desired forward power/speed (-1 to +1)
        double strafe = 0;        // Desired strafe power/speed (-1 to +1)
        double turn = 0;        // Desired turning power/speed (-1 to +1)
        while (values[0] >= 5 && values[0] <= 8) {
            if (targetFound) {

                // Determine heading, range and Yaw (tag image rotation) error so we can use them to control the robot automatically.
                double rangeError = (values[0] - DESIRED_DISTANCE);
                double headingError = values[1];
                double yawError = values[2];

                // Use the speed and turn "gains" to calculate how we want the robot to move.
                drive = Range.clip(rangeError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
                turn = Range.clip(headingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN);
                strafe = Range.clip(-yawError * STRAFE_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);
                setDegrees(values[1]);
                if (rangeError < 0.5 || rangeError > -0.5) {
                    setisStraight(true);
                } else {
                    setisStraight(false);
                }
                this.targetSaw = true;

            } else if (targetSaw && !targetFound) {
                if (isStraight) {
                    setDegrees(0);
                    continue;
                }
                double rangeError = 0;
                double headingError = getDegrees();
                double yawError = 0;
                drive = Range.clip(rangeError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
                turn = Range.clip(headingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN);
                strafe = Range.clip(-yawError * STRAFE_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);
            }
            // Apply desired  motions to the drivetrain.

            moveRobot(drive, strafe, turn);
        }
    }


    public void stops(){
            leftFrontDrive.setPower(0);
            leftBottomDrive.setPower(0);
            rightFrontDrive.setPower(0);
            rightBottomDrive.setPower(0);
        }
        public void waits(){
        sleep(500);
        }
        public void moveForward(long seconds){
            direction("forward");
            waits();
            leftFrontDrive.setPower(FORWARD_SPEED);
            leftBottomDrive.setPower(FORWARD_SPEED);
            rightFrontDrive.setPower(FORWARD_SPEED);
            rightBottomDrive.setPower(FORWARD_SPEED);
            sleep(seconds);
            stops();
        }
        public void movebackward(long seconds){
        direction("backward");

        waits();
        leftFrontDrive.setPower(FORWARD_SPEED);
            leftBottomDrive.setPower(FORWARD_SPEED);
            rightFrontDrive.setPower(FORWARD_SPEED);
            rightBottomDrive.setPower(FORWARD_SPEED);
            sleep(seconds);
            stops();
        }
        public void turnLeft(long seconds){
        direction("counterClockwise");

        waits();
        leftFrontDrive.setPower(FORWARD_SPEED);
        leftBottomDrive.setPower(FORWARD_SPEED);
        rightFrontDrive.setPower(FORWARD_SPEED);
        rightBottomDrive.setPower(FORWARD_SPEED);
        sleep(seconds);
        stops();

    }
    public void turnRight(long seconds){
        direction("clockwise");

        waits();
        leftFrontDrive.setPower(FORWARD_SPEED);
        leftBottomDrive.setPower(FORWARD_SPEED);
        rightFrontDrive.setPower(FORWARD_SPEED);
        rightBottomDrive.setPower(FORWARD_SPEED);
        sleep(seconds);
        stops();
    }
    public void moveleft(long seconds){
        direction("left");
        waits();
        leftFrontDrive.setPower(TURN_SPEED);
        leftBottomDrive.setPower(TURN_SPEED);
        rightFrontDrive.setPower(TURN_SPEED);
        rightBottomDrive.setPower(TURN_SPEED);
        sleep(seconds);
        stops();
    }
    public void moveright(long seconds){
        direction("right");
        waits();
        leftFrontDrive.setPower(TURN_SPEED);
        leftBottomDrive.setPower(TURN_SPEED);
        rightFrontDrive.setPower(TURN_SPEED);
        rightBottomDrive.setPower(TURN_SPEED);
        sleep(seconds);
        stops();
    }

}
