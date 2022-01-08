package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvInternalCamera;

@TeleOp(name = "findpink", group = "Iterative Opmode")
public class findpink extends LinearOpMode {
    Pipeline modifyPipeline = new Pipeline();
    //private OpenCvWebcam webCam;
    private  OpenCvCamera webCam;
    private static String newMessage;


    @Override
    public void runOpMode() throws InterruptedException {
        int cameraMonitorViewId2 = hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId",
                "id",
                hardwareMap.appContext.getPackageName());
        //webCam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam"), cameraMonitorViewId2);
        webCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId2);
        webCam.setPipeline(modifyPipeline);
        webCam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webCam.startStreaming(320, 240);
                telemetry.addData("Pipeline: ", "Initialized");
                telemetry.update();
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addData("Error: ", "Something went wrong :(");
                telemetry.update();
            }
        });

        Long startTime = System.currentTimeMillis();
        Long currTime = startTime;

        // Troubleshooting only recommend < 5000
        while (currTime - startTime < 50000) {
            if (currTime - startTime < 500) {
                telemetry.addData("Camera: ", "Waiting to make sure valid data is incoming");
            } else {
                telemetry.addData("Time Delta: ", (currTime - startTime));
                telemetry.addData("Pixel Count: ", modifyPipeline.getPixelCount());
            }
            telemetry.update();
            currTime = System.currentTimeMillis();
        }

        waitForStart();

        while (opModeIsActive()) {

            if (gamepad2.right_bumper) {
                modifyPipeline.lowX += 10;
            }
            if (gamepad2.left_bumper) {
                modifyPipeline.lowX -= 10;
            }
            if (gamepad2.a) {
                modifyPipeline.highX += 10;
            }
            if (gamepad2.b) {
                modifyPipeline.highX -= 10;
            }
            if (gamepad2.x) {
                modifyPipeline.lowZ += 10;
            }
            if (gamepad2.y) {
                modifyPipeline.lowZ -= 10;
            }
            if (gamepad2.dpad_right) {
                modifyPipeline.highZ += 10;
            }
            if (gamepad2.dpad_left) {
                modifyPipeline.highZ -= 10;
            }
            telemetry.addData("Low: ", modifyPipeline.lowX + ", " + modifyPipeline.lowY + ", " + modifyPipeline.lowZ);
            telemetry.addData("High: ", modifyPipeline.highX + ", " + modifyPipeline.highY + ", " + modifyPipeline.highZ);
            telemetry.update();
            sleep(500);
        }
    }
}
