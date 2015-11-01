package com.qualcomm.ftcrobotcontroller.opmodes;

        import com.qualcomm.robotcore.eventloop.opmode.OpMode;
        import com.qualcomm.robotcore.hardware.DcMotor;
        import com.qualcomm.robotcore.hardware.IrSeekerSensor;

public class AvalancheTeleOp extends OpMode {

    final static double MOTOR_POWER = 0.50; // Higher values will cause the robot to move faster

    final static double HOLD_IR_SIGNAL_STRENGTH = 0.20; // Higher values will cause the robot to follow closer

    DcMotor motorRightFront;
    DcMotor motorRightBack;
    DcMotor motorLeftFront;
    DcMotor motorLeftBack;
    //IrSeekerSensor irSeeker;


    public AvalancheTeleOp() {

    }

    @Override
    public void init() {
        //irSeeker = hardwareMap.irSeekerSensor.get("ir_seeker");
        motorRightFront = hardwareMap.dcMotor.get("rf");
        motorRightBack = hardwareMap.dcMotor.get("rb");
        motorLeftFront = hardwareMap.dcMotor.get("lf");
        motorLeftBack = hardwareMap.dcMotor.get("lb");
        motorRightFront.setDirection(DcMotor.Direction.REVERSE);
        motorRightBack.setDirection(DcMotor.Direction.REVERSE);
    }

    @Override
    public void loop() {

        /*if (irSeeker.signalDetected()) {
            // an IR signal is detected

            // Get the angle and strength of the signal
            double angle = irSeeker.getAngle();
            double strength = irSeeker.getStrength();

            // which direction should we move?
            if (angle < -20) {
                // we need to move to the left
                motorRightFront.setPower(MOTOR_POWER);
                motorRightBack.setPower(MOTOR_POWER);
                motorLeftFront.setPower(-MOTOR_POWER);
                motorLeftBack.setPower(-MOTOR_POWER);
            } else if (angle > 20) {
                // we need to move to the righ
                motorRightFront.setPower(-MOTOR_POWER);
                motorRightBack.setPower(-MOTOR_POWER);
                motorLeftFront.setPower(MOTOR_POWER);
                motorLeftBack.setPower(MOTOR_POWER);
            } else if (strength < HOLD_IR_SIGNAL_STRENGTH) {
                // the IR signal is weak, approach
                motorRightFront.setPower(MOTOR_POWER);
                motorRightBack.setPower(MOTOR_POWER);
                motorLeftFront.setPower(MOTOR_POWER);
                motorLeftBack.setPower(MOTOR_POWER);
            } else {
                // the IR signal is strong, stay here
                motorRightFront.setPower(0.0);
                motorRightBack.setPower(0.0);
                motorLeftFront.setPower(0.0);
                motorLeftBack.setPower(0.0);
            }
            telemetry.addData("Text", "*** Robot Data *** \n");
            telemetry.addData("ir_seeker reading", "ir heading: " + irSeeker.getAngle());
        } else {*/

		/*
		 *Gamepad 1 controls the motors via the left stick
		 *          controls the odometer arm via 'x' and 'b' buttons
		 */

        // Joy: left_stick_y ranges from -1 to 1, where 1 is full up, and
        // -1 is full down
        double leftJoy = scaleInput(-gamepad1.left_stick_y);
        double rightJoy = scaleInput(-gamepad1.right_stick_y);

        // write the values to the motors
        motorLeftFront.setPower(leftJoy);
        motorLeftBack.setPower(leftJoy);
        motorRightFront.setPower(rightJoy);
        motorRightBack.setPower(rightJoy);

		/*
		 * Send telemetry data back to driver station. Note that if we are using
		 * a legacy NXT-compatible motor controller, then the getPower() method
		 * will return a null value. The legacy NXT-compatible motor controllers
		 * are currently write only.
		 */
        telemetry.addData("Text", "*** Robot Data ***");
        telemetry.addData("left power: ", leftJoy);
        telemetry.addData("right power: ", rightJoy);
    }
    //}

    @Override
    public void stop() {

    }

    double scaleInput(double dVal)  {
        double[] scaleArray = { 0.0, 0.05, 0.09, 0.10, 0.12, 0.15, 0.18, 0.24,
                0.30, 0.36, 0.43, 0.50, 0.60, 0.72, 0.85, 1.00, 1.00 };

        // get the corresponding index for the scaleInput array.
        int index = (int) (dVal * 16.0);
        if (index < 0) {
            index = -index;
        } else if (index > 16) {
            index = 16;
        }

        double dScale = 0.0;
        if (dVal < 0) {
            dScale = -scaleArray[index];
        } else {
            dScale = scaleArray[index];
        }

        return dScale;
    }
}
