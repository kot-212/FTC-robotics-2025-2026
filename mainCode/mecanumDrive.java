//This is the main code for your robot, I coded it.
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="My TeleOp", group="TeleOp")
public class MecanumDrive extends OpMode {

    private DcMotor front_left  = null;
    private DcMotor front_right = null;
    private DcMotor back_left   = null;
    private DcMotor back_right  = null;

    private Servo Shalong = null;

    private static final double DEADZONE = 0.05;
    private static final double SERVO_SPEED = 0.01;
    private double servoPos = 0.3; 

    // For smooth acceleration
    private double currentFL = 0;
    private double currentFR = 0;
    private double currentBL = 0;
    private double currentBR = 0;

    private static final double ACCELERATION_RATE = 0.05; // change per loop

    @Override
    public void init() {

        front_left   = hardwareMap.get(DcMotor.class, "frontLeft");
        front_right  = hardwareMap.get(DcMotor.class, "frontRight");
        back_left    = hardwareMap.get(DcMotor.class, "backLeft");
        back_right   = hardwareMap.get(DcMotor.class, "backRight");

        Shalong = hardwareMap.get(Servo.class, "Shalong");

        front_left.setDirection(DcMotor.Direction.FORWARD);
        back_left.setDirection(DcMotor.Direction.FORWARD);
        front_right.setDirection(DcMotor.Direction.REVERSE);
        back_right.setDirection(DcMotor.Direction.REVERSE);

        front_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        front_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        back_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        back_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        Shalong.setPosition(servoPos); 
    }

    private double applyDeadzone(double value, double threshold) {
        return (Math.abs(value) < threshold) ? 0 : value;
    }

    private double rampPower(double current, double target) {
        double delta = target - current;
        if (Math.abs(delta) > ACCELERATION_RATE) {
            delta = Math.signum(delta) * ACCELERATION_RATE;
        }
        return current + delta;
    }

    @Override
    public void loop() {

        // Fixed-speed servo control
        if (gamepad1.right_trigger > 0.05) {
            servoPos += SERVO_SPEED;  // fixed step up
        } 
        else if (gamepad1.left_trigger > 0.05) {
            servoPos -= SERVO_SPEED; // fixed step down
        }
        // else do nothing → stops immediately
        
        // Clamp between 0 and 1
        servoPos = Math.max(0.0, Math.min(1.0, servoPos));
        Shalong.setPosition(servoPos);

        telemetry.addData("Shalong Position", servoPos);
        telemetry.addData("----",null);
        

        // ---- Mecanum drive inputs ----
        double drive  = applyDeadzone(-gamepad1.left_stick_y, DEADZONE);
        double strafe = applyDeadzone(gamepad1.left_stick_x, DEADZONE);
        double twist  = applyDeadzone(gamepad1.right_stick_x, DEADZONE);

        double[] targetSpeeds = {
                drive + strafe + twist,
                drive - strafe - twist,
                drive - strafe + twist,
                drive + strafe - twist
        };

        // Normalize speeds if any exceed 1
        double max = 0;
        for (double s : targetSpeeds) {
            if (Math.abs(s) > max) max = Math.abs(s);
        }
        if (max > 1) {
            for (int i = 0; i < targetSpeeds.length; i++) {
                targetSpeeds[i] /= max;
            }
        }

        // Smooth acceleration
        currentFL = rampPower(currentFL, targetSpeeds[0]);
        currentFR = rampPower(currentFR, targetSpeeds[1]);
        currentBL = rampPower(currentBL, targetSpeeds[2]);
        currentBR = rampPower(currentBR, targetSpeeds[3]);

        front_left.setPower(currentFL);
        front_right.setPower(currentFR);
        back_left.setPower(currentBL);
        back_right.setPower(currentBR);

        telemetry.update();
    }
}
