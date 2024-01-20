package org.firstinspires.ftc.teamcode.Auto;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import java.util.Objects;

public class MechanumDrive {
    private double degrees =0;
    private boolean isStraight = false;
    final double DESIRED_DISTANCE = 5.0; // how close
    final double SPEED_GAIN  =  0.02  ;   //  Forward Speed Control "Gain". eg: Ramp up to 50% power at a 25 inch error.   (0.50 / 25.0)
    final double STRAFE_GAIN =  0.015 ;   //  Strafe Speed Control "Gain".  eg: Ramp up to 25% power at a 25 degree Yaw error.   (0.25 / 25.0)
    final double TURN_GAIN   =  0.01  ;   //  Turn Control "Gain".  eg: Ramp up to 25% power at a 25 degree error. (0.25 / 25.0)
    final double MAX_AUTO_SPEED = 0.5;   //  Clip the approach speed to this max value
    final double MAX_AUTO_STRAFE= 0.5;   //  Clip the approach speed to this max value
    final double MAX_AUTO_TURN  = 0.3;   //  Clip the turn speed to this max value

    private final DcMotor leftFrontDrive;
    private final DcMotor leftBackDrive;
    private final DcMotor rightFrontDrive;
    private final DcMotor rightBackDrive;
    boolean targetSaw = false;
    private DcMotor lift = null;
    private Servo door = null;
    private Servo intakeRight = null;
    private Servo intakeLeft = null;
    private final double ticks = 1440;
    private double target;
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
        leftBackDrive.setPower(leftBackPower);
        rightBackDrive.setPower(rightBackPower);
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

    public void driveToPixel(double[] values){
        boolean targetFound;
        targetFound = (values[2] == 1);
        double drive = 0;        // Desired forward power/speed (-1 to +1)
        double strafe = 0;        // Desired strafe power/speed (-1 to +1)
        double turn = 0;        // Desired turning power/speed (-1 to +1)
        while (values[0] >= 10 && values[0] <= 17) {
            if (targetFound) {

                // Determine heading, range and Yaw (tag image rotation) error so we can use them to control the robot automatically.
                double rangeError = (values[0] - 15);
                double headingError = values[1];

                // Use the speed and turn "gains" to calculate how we want the robot to move.
                drive = Range.clip(rangeError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
                turn = Range.clip(headingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN);

            }

            moveRobot(drive, strafe, turn);
        }
    }


    public void direction(String direct) {
        if (Objects.equals(direct, "forward")) {
            leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
            leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
            rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
            rightBackDrive.setDirection(DcMotor.Direction.FORWARD);
        } else if (Objects.equals(direct, "backward")) {
            leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
            leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
            rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
            rightBackDrive.setDirection(DcMotor.Direction.REVERSE);
        } else if (Objects.equals(direct, "right")) {
            leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
            leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
            rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
            rightBackDrive.setDirection(DcMotor.Direction.FORWARD);
        } else if (Objects.equals(direct, "left")) {
            leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
            leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
            rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
            rightBackDrive.setDirection(DcMotor.Direction.REVERSE);
        } else if (Objects.equals(direct, "clockwise")) {
            leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
            leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
            rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
            rightBackDrive.setDirection(DcMotor.Direction.REVERSE);
        } else if (Objects.equals(direct, "counterClockwise")) {
            leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
            leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
            rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
            rightBackDrive.setDirection(DcMotor.Direction.FORWARD);
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
    public MechanumDrive(boolean useEncoder, DcMotor lf,DcMotor lb,DcMotor rf,DcMotor rb){

            this.leftFrontDrive = lf;
            this.leftBackDrive = lb;
            this.rightFrontDrive = rf;
            this.rightBackDrive = rb;
            //        lift = hardwareMap.get(DcMotor.class, "linearLift");
            //        door = hardwareMap.get(Servo.class, "trapDoor");
            //        lift.setDirection(DcMotorSimple.Direction.REVERSE);
            //        door.setDirection(Servo.Direction.REVERSE);
            if(leftFrontDrive != null) {
                if (!useEncoder) {
                //configure motors and servos.

                // if we are not using encoder
                    rightFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    leftFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    rightBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    leftBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                //            lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                } else {
                    //if we use the encoder
                    rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                //            lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }
        }
    }

    public void moveForward(double power, int turnage) throws InterruptedException {
        direction("forward");
        rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.target = ticks/turnage;
        rightFrontDrive.setTargetPosition((int)this.target);
        leftFrontDrive.setTargetPosition((int)this.target);
        rightBackDrive.setTargetPosition((int)this.target);
        leftBackDrive.setTargetPosition((int)this.target);
        leftFrontDrive.setPower(power);
        rightFrontDrive.setPower(power);
        leftBackDrive.setPower(power);
        rightBackDrive.setPower(power);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    public void moveBackward(double power, int turnage) throws InterruptedException {
        direction("backward");
        rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.target = ticks * turnage;
        rightFrontDrive.setTargetPosition((int)target);
        leftFrontDrive.setTargetPosition((int)target);
        rightBackDrive.setTargetPosition((int)target);
        leftBackDrive.setTargetPosition((int)target);
        leftFrontDrive.setPower(power);
        rightFrontDrive.setPower(power);
        leftBackDrive.setPower(power);
        rightBackDrive.setPower(power);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    public void moveRight(double power, int turnage) throws InterruptedException {
        direction("right");

        rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.target = ticks * turnage;
        rightFrontDrive.setTargetPosition((int)target);
        leftFrontDrive.setTargetPosition((int)target);
        rightBackDrive.setTargetPosition((int)target);
        leftBackDrive.setTargetPosition((int)target);
        leftFrontDrive.setPower(power);
        rightFrontDrive.setPower(power);
        leftBackDrive.setPower(power);
        rightBackDrive.setPower(power);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    public void moveLeft(double power, int turnage) throws InterruptedException {
        direction("left");

        rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.target = ticks * turnage;
        rightFrontDrive.setTargetPosition((int)target);
        leftFrontDrive.setTargetPosition((int)target);
        rightBackDrive.setTargetPosition((int)target);
        leftBackDrive.setTargetPosition((int)target);
        leftFrontDrive.setPower(power);
        rightFrontDrive.setPower(power);
        leftBackDrive.setPower(power);
        rightBackDrive.setPower(power);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    public void turnRight(double power, int turnage) throws InterruptedException {
        direction("clockwise");
        rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.target = ticks * turnage;
        rightFrontDrive.setTargetPosition((int)target);
        leftFrontDrive.setTargetPosition((int)target);
        rightBackDrive.setTargetPosition((int)target);
        leftBackDrive.setTargetPosition((int)target);
        leftFrontDrive.setPower(power);
        rightFrontDrive.setPower(power);
        leftBackDrive.setPower(power);
        rightBackDrive.setPower(power);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    public void turnLeft(double power, int turnage) throws InterruptedException {
        direction("counterClockwise");
        rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.target = ticks * turnage;
        rightFrontDrive.setTargetPosition((int)target);
        leftFrontDrive.setTargetPosition((int)target);
        rightBackDrive.setTargetPosition((int)target);
        leftBackDrive.setTargetPosition((int)target);
        leftFrontDrive.setPower(power);
        rightFrontDrive.setPower(power);
        leftBackDrive.setPower(power);
        rightBackDrive.setPower(power);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    public void moveLiftUp(double power, int turnage) throws InterruptedException {
        direction("up");

        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.target = ticks * turnage;
        lift.setTargetPosition((int)target);
        lift.setPower(power);
        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    public void moveDoorUp(double turn) throws InterruptedException {
        direction("push");
        door.setPosition(turn);
    }
    public void reset() throws InterruptedException {
        direction("down");
        door.setPosition(0);
        direction("down");
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setTargetPosition((int)this.target);
        lift.setPower(0.3);
        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
}
