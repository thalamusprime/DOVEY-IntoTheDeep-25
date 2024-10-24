package org.firstinspires.ftc.teamcode.ftc6205.motors;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Drivetrain {
    public DcMotor topLeftDriveMotor;
    public DcMotor bottomLeftDriveMotor;
    public DcMotor topRightDriveMotor;
    public DcMotor bottomRightDriveMotor;

    HardwareMap hwMap;

    public void init(HardwareMap ahwMap) {

        hwMap = ahwMap;

        // Control HUb
        topLeftDriveMotor = hwMap.dcMotor.get("frontleft");
        bottomLeftDriveMotor = hwMap.dcMotor.get("backleft");
        topRightDriveMotor = hwMap.dcMotor.get("frontright");
        bottomRightDriveMotor = hwMap.dcMotor.get("backright");

        topLeftDriveMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bottomLeftDriveMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        topRightDriveMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bottomRightDriveMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //THIS IS THE CORRECT ORIENTATION
        //topLeftDriveMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        //bottomLeftDriveMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        topRightDriveMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        bottomRightDriveMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        topLeftDriveMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bottomLeftDriveMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        topRightDriveMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bottomRightDriveMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
    public void initMotors(HardwareMap ahwMap) {
        hwMap = ahwMap;

        topLeftDriveMotor = hwMap.dcMotor.get("frontleft");
        bottomLeftDriveMotor = hwMap.dcMotor.get("backleft");
        topRightDriveMotor = hwMap.dcMotor.get("frontright");
        bottomRightDriveMotor = hwMap.dcMotor.get("backright");
        // Reverse the right side motors.
        topRightDriveMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        bottomRightDriveMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        // Set to brake mode
        topLeftDriveMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bottomLeftDriveMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        topRightDriveMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bottomRightDriveMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // Run without encoder
        topLeftDriveMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bottomLeftDriveMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        topRightDriveMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bottomRightDriveMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    public void setPower(double frontLeftPower,
                           double backLeftPower,
                           double frontRightPower,
                           double backRightPower) {
        topLeftDriveMotor.setPower(frontLeftPower);
        bottomLeftDriveMotor.setPower(backLeftPower);
        topRightDriveMotor.setPower(frontRightPower);
        bottomRightDriveMotor.setPower(backRightPower);
    }
}