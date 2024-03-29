package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;

import java.util.Locale;

@Autonomous(name = "ZuluRedFarPath", group = "")
public class ZuluRedFarPath extends LinearOpMode {
    Pipeline modifyPipeline = new Pipeline();
    // For a webcam (uncomment below)
    //private OpenCvWebcam webCam;
    // For a phone camera (uncomment below)
    private  OpenCvCamera webCam;
    private boolean isCameraStreaming = false;
    private int resultROI;

    private DcMotor LF = null;
    private DcMotor RF = null;
    private DcMotor LB = null;
    private DcMotor RB = null;
    private CRServo spinspinducky = null;
    private CRServo intake = null;
    private DcMotor armboom = null;
    private Servo platform = null;

    static final float MAX_SPEED = 1.0f;
    static final float MIN_SPEED = 0.4f;
    static final int ACCEL = 75;  // Scaling factor used in accel / decel code.  Was 100!
    public float desiredHeading;

    private static final int ARM_REST = 50;
    private static final int ARM_HIGH = 775;
    private static final int ARM_MED = 950;
    private static final int ARM_LOW = 1050;

    BNO055IMU imu;
    Orientation angles;
    Acceleration gravity;

    @Override
    public void runOpMode() throws InterruptedException {
        int cameraMonitorViewId2 = hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId",
                "id",
                hardwareMap.appContext.getPackageName());
        // For a webcam (uncomment below)
        webCam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam"), cameraMonitorViewId2);

        // For a phone camera (uncomment below)
        // webCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId2);
        webCam.setPipeline(modifyPipeline);
        webCam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webCam.startStreaming(320, 240);
                telemetry.addData("Pipeline: ", "Initialized");
                telemetry.update();
                isCameraStreaming = true;
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addData("Error: ", "Something went wrong :(");
                telemetry.update();
            }
        });

        LF = hardwareMap.get(DcMotor.class, "LF");
        RF = hardwareMap.get(DcMotor.class, "RF");
        LB = hardwareMap.get(DcMotor.class, "LB");
        RB = hardwareMap.get(DcMotor.class, "RB");
        armboom = hardwareMap.get(DcMotor.class, "armboom");
        intake = hardwareMap.get(CRServo.class, "intake");
        spinspinducky = hardwareMap.get(CRServo.class, "spinspinducky");
        platform = hardwareMap.get(Servo.class, "platform");

        LF.setDirection(DcMotor.Direction.REVERSE);  // motor direction set for mecanum wheels with mitre gears
        RF.setDirection(DcMotor.Direction.FORWARD);
        LB.setDirection(DcMotor.Direction.REVERSE);
        RB.setDirection(DcMotor.Direction.FORWARD);

        // IMU initialization
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        // Set up our telemetry dashboard
        composeTelemetry();  // need to add this method at end of code

        desiredHeading = getHeading();

        moveUtils.initialize(LF, RF, LB, RB, imu, desiredHeading);
        actuatorUtils.initializeActuator(armboom, spinspinducky, intake);
        actuatorUtils.initializeActuatorMovement(LF, RF, LB, RB);
        moveUtils.resetEncoders();

        Long startTime = System.currentTimeMillis();
        Long currTime = startTime;

        // Troubleshooting only recommend < 5000
        while (currTime - startTime < 5000) {
            if (currTime - startTime < 500) {
                telemetry.addData("Camera: ", "Waiting to make sure valid data is incoming");
            } else {
                telemetry.addData("Time Delta: ", (currTime - startTime));
                resultROI = modifyPipeline.getResultROI();
                switch (resultROI) {
                    case 0:
                        telemetry.addData("Resulting ROI: ", "Left");
                        break;
                    case 1:
                        telemetry.addData("Resulting ROI: ", "Middle");
                        break;
                    case 2:
                        telemetry.addData("Resulting ROI: ", "Right");
                        break;
                    default:
                        telemetry.addData("Resulting ROI: ", "Something went wrong.");
                        break;
                }
                telemetry.update();
            }
            currTime = System.currentTimeMillis();
        }
        platform.setPosition(90);
        waitForStart();

        // Have started.
        // First thing, stop camera.
        if (isCameraStreaming) {
            webCam.stopStreaming();
            webCam.closeCameraDevice();
            isCameraStreaming = false;
        }

        platform.setPosition(0);

        moveUtils.goStraight(-6, MAX_SPEED, MIN_SPEED, ACCEL);
        moveUtils.turnACW(30);

        switch (resultROI) {
            case 0:
                // Left (Bottom Level)
                moveUtils.goStraight(-14, MAX_SPEED, MIN_SPEED, ACCEL);
                actuatorUtils.moveThatArm(ARM_HIGH);
                actuatorUtils.intakeMove(-1);
                sleep(800);
                actuatorUtils.intakeMove(0);
                actuatorUtils.moveThatArm(ARM_REST);
                moveUtils.turnCW(120);
                moveUtils.strafeBuddy(-60);
                break;
            case 1:
                // Middle (Middle Level)
                moveUtils.goStraight(-10, MAX_SPEED, MIN_SPEED, ACCEL);
                actuatorUtils.moveThatArm(ARM_MED);
                moveUtils.goStraight(-2, MAX_SPEED, MIN_SPEED, ACCEL);
                actuatorUtils.intakeMove(-1);
                sleep(1000);
                actuatorUtils.intakeMove(0);
                actuatorUtils.moveThatArm(ARM_REST);
                moveUtils.turnCW(120);
                moveUtils.strafeBuddy(-60);
                break;
            default:
                // Right (Top Level)
                moveUtils.goStraight(-10, MAX_SPEED, MIN_SPEED, ACCEL);
                actuatorUtils.moveThatArm(ARM_LOW);
                moveUtils.goStraight(-2, MAX_SPEED, MIN_SPEED, ACCEL);
                actuatorUtils.intakeMove(-1);
                sleep(1000);
                actuatorUtils.intakeMove(0);
                actuatorUtils.moveThatArm(ARM_REST);
                moveUtils.turnCW(120);
                moveUtils.strafeBuddy(-60);
                break;
        }
        moveUtils.goStraight(45, MAX_SPEED, MIN_SPEED, ACCEL);
        moveUtils.strafeBuddy(6);
        moveUtils.goStraight(4, MAX_SPEED, MIN_SPEED, ACCEL);
        actuatorUtils.spinThatDucky(true);
        moveUtils.goStraight(-5,MAX_SPEED,MIN_SPEED,ACCEL);
        moveUtils.turnCW(40);
        moveUtils.goStraight(-96,1,MAX_SPEED,ACCEL);

    }

    void composeTelemetry() {

        // At the beginning of each telemetry update, grab a bunch of data
        // from the IMU that we will then display in separate lines.
        telemetry.addAction(new Runnable() {
            @Override
            public void run() {
                // Acquiring the angles is relatively expensive; we don't want
                // to do that in each of the three items that need that info, as that's
                // three times the necessary expense.
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                gravity = imu.getGravity();
            }
        });

        telemetry.addLine()
                .addData("status", new Func<String>() {
                    @Override
                    public String value() {
                        return imu.getSystemStatus().toShortString();
                    }
                })
                .addData("calib", new Func<String>() {
                    @Override
                    public String value() {
                        return imu.getCalibrationStatus().toString();
                    }
                });

        telemetry.addLine()
                .addData("heading", new Func<String>() {
                    @Override
                    public String value() {
                        return formatAngle(angles.angleUnit, angles.firstAngle);
                    }
                })
                .addData("roll", new Func<String>() {
                    @Override
                    public String value() {
                        return formatAngle(angles.angleUnit, angles.secondAngle);
                    }
                })
                .addData("pitch", new Func<String>() {
                    @Override
                    public String value() {
                        return formatAngle(angles.angleUnit, angles.thirdAngle);
                    }
                });

    }

    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    String formatDegrees(double degrees) {
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }

    public float getHeading() {
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC,
                AxesOrder.ZYX,
                DEGREES);
        return angles.firstAngle;
    }
}
