package org.firstinspires.ftc.teamcode.teleop;

//import com.acmerobotics.dashboard.FtcDashboard;
//import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

//import org.firstinspires.ftc.teamcode.ftc6205.pidcontrol.TrueNorth;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.ftc6205.motors.Drivetrain;
import org.firstinspires.ftc.teamcode.ftc6205.telemetry.DoveyTelemetry;
import org.firstinspires.ftc.teamcode.ftc6205.sensors.Camera;
import org.firstinspires.ftc.teamcode.test.SensorSparkFunOTOS;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

@TeleOp(name = "*:DOVEY", group = "*")
public class DOVEY_OPS extends LinearOpMode {
    //Drivetrain drivetrain;

    ////////////////////////////////////////////////////
    // Telemetry
    DoveyTelemetry doveyTelemetry;
    Camera doveyVision;
    AprilTagProcessor tagProcessor;
    VisionPortal visionPortal;
    // OpticalTracking
    SparkFunOTOS otos;

    // DRIVETRAIN
    DcMotor frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor;

    // ENCODERS
    DcMotor encoderLeft, encoderRight;
    double encLeftValue, encRightValue;

    // DISTANCE
    DistanceSensor distFront, distBack;

    // IMU
    IMU imu;
    double refHeading, botHeading, pidOutput;
    double y, x, rz, rotX, rotY;


    @Override
    public void runOpMode() throws InterruptedException {

        //drivetrain.init(new HardwareMap());
        ////////////////////////////////////
        // Initialize Telemetry
        doveyTelemetry  = new DoveyTelemetry();
        doveyVision  = new Camera();
        // Initialize Servos, Encoders, Motors
        //initDistSensors();
        initEncoders();
        initMotors();
        //initServos();
        //initOTOS();
        initIMU();
        initVision();

        // Pause until "Play".  Close program if "Stop".
        waitForStart();
        if (isStopRequested()) return;

        /////////////////////////////////////////////////////////////// TELEOP LOOP

        while (opModeIsActive()) {
            // Dashboard
            //telPack.field();
            //dashboard.sendTelemetryPacket(telPack);

            // Set hold, drop (TESTING)
            if (gamepad1.a){
                continue;
                //pixelThumb.setPosition(0); // hold
            }
            if (gamepad1.b){
                continue;
                //pixelThumb.setPosition(0.5); // drop
            }

            // Get current encoder position
            encLeftValue = -encoderLeft.getCurrentPosition();
            encRightValue = -encoderRight.getCurrentPosition();

            // Get OTOS position
            //SparkFunOTOS.Pose2D pos = otos.getPosition();

            // DRIVETRAIN
            // Get yaw, reset in match optional

            if (gamepad1.share) {
                encoderLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                encoderRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                encoderLeft.setDirection(DcMotorSimple.Direction.FORWARD);
                encoderRight.setDirection(DcMotorSimple.Direction.REVERSE);
            }
            if (gamepad1.options) {
                //initOTOS();
                refHeading = 0;
                botHeading = 0;
                imu.resetYaw();
            }
            // Get XY: gamepad1
            y = gamepad1.left_stick_y; // Remember, Y stick value is reversed
            x = -gamepad1.left_stick_x; //-

            // Get Z: rz > 0.03, rz = 0
            if (Math.abs(gamepad1.right_stick_x) > 0.03) { // Yaw threshold
                rz = -gamepad1.right_stick_x;
                refHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS); // ref
            } else {
                rz = 0;
                // PID Controller
//                TrueNorth pidControl = new TrueNorth();
//                pidOutput = pidControl.PIDControl(refHeading, botHeading);
                if (Math.abs(pidOutput) > 0.03) {
                    rz = pidOutput;
                }
            }

            // Get heading
            botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS); // bot


            // Robot-centric, Field-centric
            if (gamepad1.left_bumper) {
                rotX = x;
                rotY = y;
            } else {
                rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
                rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);
            }
            rotX = rotX * 1.1;  // Counteract imperfect strafing

            // Denominator (absolute value) or 1, at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rz), 1);
            double frontLeftPower = (rotY + rotX + rz) / denominator;
            double backLeftPower = (rotY - rotX + rz) / denominator;
            double frontRightPower = (rotY - rotX - rz) / denominator;
            double backRightPower = (rotY + rotX - rz) / denominator;

            // Trigger gain
            frontLeftPower = frontLeftPower * (0.3 + 0.7 * gamepad1.right_trigger);
            backLeftPower = backLeftPower * (0.3 + 0.7 * gamepad1.right_trigger);
            frontRightPower = frontRightPower * (0.3 + 0.7 * gamepad1.right_trigger);
            backRightPower = backRightPower * (0.3 + 0.7 * gamepad1.right_trigger);

            // Set motor power
            frontLeftMotor.setPower(frontLeftPower);
            backLeftMotor.setPower(backLeftPower);
            frontRightMotor.setPower(frontRightPower);
            backRightMotor.setPower(backRightPower);

            // Send Telemetry
            doveyTelemetry.sendTelemetry(telemetry, tagProcessor, otos);
        }
    }

    //////////////////////////////////////////////////////////// CUSTOM PRIVATE FUNCTIONS

    private void initDistSensors() throws InterruptedException {
        //distFront = hardwareMap.get(DistanceSensor.class, "distFront");
        //distBack = hardwareMap.get(DistanceSensor.class, "distBack");
    }

    private void initIMU() throws InterruptedException {
        // Retrieve the IMU from the hardware map
        imu = hardwareMap.get(IMU.class, "imu");
        // Adjust the orientation parameters to match your robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD)); //FORWARD
        imu.initialize(parameters);
    }

    private void initVision() throws InterruptedException {
        // Tag Processing
        tagProcessor = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .build();
        // VisionPortal
        visionPortal = new VisionPortal.Builder()
                .addProcessor(tagProcessor)
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                //.setCameraResolution( new Size(1920,1080) )
                .build();
    }
    //todo
    private void initOTOS() throws InterruptedException {
        otos = hardwareMap.get(SparkFunOTOS.class,"sensor_otos");
        otos.setLinearUnit(DistanceUnit.INCH);
        otos.setAngularUnit(AngleUnit.DEGREES);
        SparkFunOTOS.Pose2D offset = new SparkFunOTOS.Pose2D(0, 0, 0);
        otos.setOffset(offset);
        otos.setLinearScalar(1.0);
        otos.setAngularScalar(1.0);
        otos.calibrateImu();
        otos.resetTracking();
        SparkFunOTOS.Pose2D currentPosition = new SparkFunOTOS.Pose2D(0, 0, 0);
        otos.setPosition(currentPosition);
    }

    //todo
    private void initEncoders() throws InterruptedException {
        encoderLeft = hardwareMap.dcMotor.get("frontleft");
        encoderRight = hardwareMap.dcMotor.get("frontright");
        encoderLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        encoderLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        encoderLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        encoderRight.setDirection(DcMotorSimple.Direction.REVERSE);
        encoderRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        encoderRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    private void initMotors() throws InterruptedException {
        frontLeftMotor = hardwareMap.dcMotor.get("frontleft");
        backLeftMotor = hardwareMap.dcMotor.get("backleft");
        frontRightMotor = hardwareMap.dcMotor.get("frontright");
        backRightMotor = hardwareMap.dcMotor.get("backright");
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

}
