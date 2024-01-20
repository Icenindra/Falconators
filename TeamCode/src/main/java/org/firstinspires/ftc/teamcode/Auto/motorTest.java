package org.firstinspires.ftc.teamcode.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Autonomous (name = "motorTest")
public class motorTest extends LinearOpMode {
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;
    private DcMotor arm = null;
    private CRServo claw = null;
    public motorTest(DcMotor fl,DcMotor fr,DcMotor bl, DcMotor br){
        leftFrontDrive = fl;
        rightFrontDrive = fr;
        leftBackDrive = bl;
        rightBackDrive = br;
    }


    @Override
    public void runOpMode() throws InterruptedException {
        leftFrontDrive = hardwareMap.get(DcMotor.class, "leftFrontDrive");
        leftBackDrive = hardwareMap.get(DcMotor.class, "leftBottomDrive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "rightFrontDrive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "rightBottomDrive");
        arm = hardwareMap.get(DcMotor.class, "arm");
        claw = hardwareMap.get(CRServo.class, "claw");
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);



        waitForStart();

        moveForward(0.75 ,  1000);
        turnRight(0.6, 1400);
        moveright(0.6, 1000);
        moveleft(0.6, 1000);
        turnleft(0.6, 1400);
        movebackward(0.6,1000);




    }
    public void letgo(){
        claw.setPower(-1);
        sleep(1000);
        claw.setPower(0);
    }
    public void grab(){
        claw.setPower(1);
        sleep(750);
        claw.setPower(0);
    }
    public void armUp(double power, int time){
        arm.setPower(power);
        sleep(time);
        arm.setPower(0);
    }
    public void armDown(double power, int time){
        arm.setPower(-power);
        sleep(time);
        arm.setPower(0);
    }
    public void movebackward (double power, int time){
        leftBackDrive.setPower(power);
        leftFrontDrive.setPower(power);
        rightBackDrive.setPower(power);
        rightFrontDrive.setPower(power);
        sleep(time);
        leftBackDrive.setPower(0);
        leftFrontDrive.setPower(0);
        rightBackDrive.setPower(0);
        rightFrontDrive.setPower(0);
    }
    public void moveForward (double power, int time){
        leftBackDrive.setPower(-power);
        leftFrontDrive.setPower(-power);
        rightBackDrive.setPower(-power);
        rightFrontDrive.setPower(-power);
        sleep(time);
        leftBackDrive.setPower(0);
        leftFrontDrive.setPower(0);
        rightBackDrive.setPower(0);
        rightFrontDrive.setPower(0);
    }public void moveright (double power, int time){
        leftBackDrive.setPower(power);
        leftFrontDrive.setPower(power);
        rightBackDrive.setPower(-power);
        rightFrontDrive.setPower(-power);
        sleep(time);
        leftBackDrive.setPower(0);
        leftFrontDrive.setPower(0);
        rightBackDrive.setPower(0);
        rightFrontDrive.setPower(0);
    }public void moveleft (double power, int time){
        leftBackDrive.setPower(-power);
        leftFrontDrive.setPower(-power);
        rightBackDrive.setPower(power);
        rightFrontDrive.setPower(power);
        sleep(time);
        leftBackDrive.setPower(0);
        leftFrontDrive.setPower(0);
        rightBackDrive.setPower(0);
        rightFrontDrive.setPower(0);
    }public void turnRight (double power, int time){
        leftBackDrive.setPower(power);
        leftFrontDrive.setPower(-power);
        rightBackDrive.setPower(-power);
        rightFrontDrive.setPower(power);
        sleep(time);
        leftBackDrive.setPower(0);
        leftFrontDrive.setPower(0);
        rightBackDrive.setPower(0);
        rightFrontDrive.setPower(0);
    }public void turnleft (double power, int time){
        leftBackDrive.setPower(-power);
        leftFrontDrive.setPower(power);
        rightBackDrive.setPower(power);
        rightFrontDrive.setPower(-power);
        sleep(time);
        leftBackDrive.setPower(0);
        leftFrontDrive.setPower(0);
        rightBackDrive.setPower(0);
        rightFrontDrive.setPower(0);
    }
}
