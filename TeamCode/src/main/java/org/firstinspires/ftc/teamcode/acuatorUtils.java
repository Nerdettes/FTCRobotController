package org.firstinspires.ftc.teamcode;

import static java.lang.Thread.sleep;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

public class acuatorUtils {

    private static DcMotor LF = null;
    private static DcMotor RF = null;
    private static DcMotor LB = null;
    private static DcMotor RB = null;
    private static CRServo spinner = null;
    private static BNO055IMU imu;
    private static float desiredHeading;
    private static CRServo spinspinducky = null;
    private static DcMotor armboom = null;

    public static void initialize(DcMotor LF, DcMotor RF, DcMotor LB, DcMotor RB, BNO055IMU imu, float currHeading) {
        acuatorUtils.LF = LF;
        acuatorUtils.RF = RF;
        acuatorUtils.LB = LB;
        acuatorUtils.RB = RB;
        acuatorUtils.spinner = spinner;
        acuatorUtils.imu = imu;
        acuatorUtils.desiredHeading = currHeading;
    }

    public static void resetEncoders() {
        LF.setPower(0);
        RF.setPower(0);
        LB.setPower(0);
        RB.setPower(0);

        LF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Just a little time to make sure encoders have reset
        //sleep(200);

        // Not technically encoder but...
        LF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Only using the Back motor Encoders
        LF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public static void spinThatDucky (boolean isRed) throws InterruptedException {
        resetEncoders();
        LF.setPower(.05);
        LB.setPower(.05);
        RF.setPower(.05);
        RB.setPower(.05);
        if (isRed) {
            spinspinducky.setPower(-1);
        }
        else {
            spinspinducky.setPower(1);
        }
        sleep(1000);
        resetEncoders();
        sleep(4000);
        spinspinducky.setPower(0);

    }

    public static void moveThatArm (int getthatdistance){

        while (armboom.getCurrentPosition() <  getthatdistance)
        {
            armboom.setPower(1);
        }
    }
}
