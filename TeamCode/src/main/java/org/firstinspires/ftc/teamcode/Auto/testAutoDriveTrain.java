package org.firstinspires.ftc.teamcode.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Auto.MechanumDrive;

@Autonomous (name = "AutoTestDriveTrain")
public class testAutoDriveTrain extends LinearOpMode {
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;

    @Override
    public void runOpMode() throws InterruptedException {
        leftFrontDrive = hardwareMap.get(DcMotor.class, "leftFrontDrive");
        leftBackDrive = hardwareMap.get(DcMotor.class, "leftBottomDrive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "rightFrontDrive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "rightBottomDrive");

        MechanumDrive mechanumDrive = new MechanumDrive(true, leftFrontDrive, leftBackDrive, rightFrontDrive, rightBackDrive);
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();

        mechanumDrive.moveForward(0.25,1);
//        mechanumDrive.turnLeft(0.5, 3);
//        mechanumDrive.turnRight(0.5,3);
//        mechanumDrive.moveBackward(0.5,3);
//        mechanumDrive.moveLeft(0.5,3);
//        mechanumDrive.moveRight(0.5,3);

    }
}
