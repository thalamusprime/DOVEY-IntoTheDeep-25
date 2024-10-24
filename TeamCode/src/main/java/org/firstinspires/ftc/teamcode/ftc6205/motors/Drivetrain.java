package org.firstinspires.ftc.teamcode.ftc6205.motors;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Drivetrain {
    public DcMotor frontLeftMotor;
    public DcMotor backLeftMotor;
    public DcMotor frontRightMotor;
    public DcMotor backRightMotor;

    HardwareMap hwMap;

    public void init(HardwareMap ahwMap) {

        hwMap = ahwMap;

        // Control HUb
        frontLeftMotor = hwMap.dcMotor.get("frontleft");
        backLeftMotor = hwMap.dcMotor.get("backleft");
        frontRightMotor = hwMap.dcMotor.get("frontright");
        backRightMotor = hwMap.dcMotor.get("backright");

        frontLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //THIS IS THE CORRECT ORIENTATION
        //topLeftDriveMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        //bottomLeftDriveMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
    public void initMotors(HardwareMap ahwMap) {
        hwMap = ahwMap;

        frontLeftMotor = hwMap.dcMotor.get("frontleft");
        backLeftMotor = hwMap.dcMotor.get("backleft");
        frontRightMotor = hwMap.dcMotor.get("frontright");
        backRightMotor = hwMap.dcMotor.get("backright");
        // Reverse the right side motors.
        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        // Set to brake mode
        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // Run without encoder
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    public void setPower(double frontLeftPower,
                           double backLeftPower,
                           double frontRightPower,
                           double backRightPower) {
        frontLeftMotor.setPower(frontLeftPower);
        backLeftMotor.setPower(backLeftPower);
        frontRightMotor.setPower(frontRightPower);
        backRightMotor.setPower(backRightPower);
    }
}