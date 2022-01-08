package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class moveUtils {

    // Things that need to be imported
    private static DcMotor LF = null;
    private static DcMotor RF = null;
    private static DcMotor LB = null;
    private static DcMotor RB = null;
    private static CRServo spinner = null;
    private static BNO055IMU imu;
    private static float desiredHeading;

    // Things specific to this class
    private static final float TURN_SPEED_HIGH = 1f;
    private static final float TURN_SPEED_LOW = 0.15f;
    private static final float TURN_HIGH_ANGLE = 45.0f;
    private static final float TURN_LOW_ANGLE = 5.0f;

    public static void initialize(DcMotor LF, DcMotor RF, DcMotor LB, DcMotor RB, BNO055IMU imu, float currHeading) {
        moveUtils.LF = LF;
        moveUtils.RF = RF;
        moveUtils.LB = LB;
        moveUtils.RB = RB;
        moveUtils.spinner = spinner;
        moveUtils.imu = imu;
        moveUtils.desiredHeading = currHeading;
    }

    public static void turnCW(float turnDegrees) {
        desiredHeading -= turnDegrees;
        if (desiredHeading < -180) {
            desiredHeading += 360;
        }
        turnToHeading();
    }

    public void turnACW(float turnDegrees) {
        desiredHeading += turnDegrees;
        if (desiredHeading > 180) {
            desiredHeading -= 360;
        }
        turnToHeading();
    }

    private static void turnToHeading() {
        boolean isCW = deltaHeading() > 0;

        if (isCW) {
            // 1st stage - high power rough heading.
            if (deltaHeading() > TURN_HIGH_ANGLE) {
                setAllMotorsTurnPower(TURN_SPEED_HIGH);
                while (deltaHeading() > TURN_HIGH_ANGLE) {
                }
            }
            // 2nd stage - low power fine heading.
            if (deltaHeading() > TURN_LOW_ANGLE) {
                setAllMotorsTurnPower(TURN_SPEED_LOW);
                while (deltaHeading() > TURN_LOW_ANGLE) {
                }
            }
        } else { // Going ACW
            // 1st stage - high power rough heading.
            if (deltaHeading() < -TURN_HIGH_ANGLE) {
                setAllMotorsTurnPower(-TURN_SPEED_HIGH);
                while (deltaHeading() < -TURN_HIGH_ANGLE) {
                }
            }
            // 2nd stage - low power fine heading.
            if (deltaHeading() < -TURN_LOW_ANGLE) {
                setAllMotorsTurnPower(-TURN_SPEED_LOW);
                while (deltaHeading() < -TURN_LOW_ANGLE) {
                }
            }
        }
        resetEncoders();
    }

    private static void setAllMotorsTurnPower(float turnPower) {
        LF.setPower(turnPower);
        LB.setPower(turnPower);
        RF.setPower(-turnPower);
        RB.setPower(-turnPower);
    }

    private static float deltaHeading() {
        float dH = getHeading() - desiredHeading;
        if (dH < -180) { dH += 360; }
        if (dH > 180) { dH -= 360; }
        return dH;
    }

    private static float getHeading() {
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC,
                AxesOrder.ZYX,
                DEGREES);
        return angles.firstAngle;
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
}
