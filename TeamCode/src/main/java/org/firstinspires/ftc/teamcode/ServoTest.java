package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;

@Autonomous(name="ServoTest", group="")

public class ServoTest extends LinearOpMode {

    CRServo spinspinducky = null;

    @Override
    public void runOpMode() throws InterruptedException {
        spinspinducky = hardwareMap.get(CRServo.class, "spinspinducky");
        sleep(1000);
        waitForStart();
        spinThatDuck(false);
    }

    private void spinThatDuck(boolean isRed) {
        if (isRed) {
            spinspinducky.setPower(-1d);
        } else {
            spinspinducky.setPower(1d);
        }
        //spinning for 10 secs
        telemetry.addData("Spinner: ", "Spinning");
        telemetry.update();
        sleep(10000);
        //finishing
        spinspinducky.setPower(0d);
    }
}
