package com.qualcomm.ftcrobotcontroller.opmodes;

        import com.qualcomm.robotcore.eventloop.opmode.OpMode;
        import com.qualcomm.robotcore.hardware.DcMotor;
        import com.qualcomm.robotcore.hardware.DcMotorController;

public class AvalancheTeleOp extends AvalancheRobot1
{
    public AvalancheTeleOp()
    {
    }

    @Override
    public void init()
    {
        super.init();
        resetDriveEncoders();
    }

    @Override
    public void loop() {

        // Joy: left_stick_y ranges from -1 to 1, where 1 is full up, and
        // -1 is full down
        double leftJoy = scaleInput(-gamepad1.left_stick_y);
        double rightJoy = scaleInput(-gamepad1.right_stick_y);

        setDrivePower(leftJoy, leftJoy, rightJoy, rightJoy);

        // write the values to the motors
        /*motorLeftFront.setPower(leftJoy);
        motorLeftBack.setPower(leftJoy);
        motorRightFront.setPower(rightJoy);
        motorRightBack.setPower(rightJoy);*/

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

    @Override
    public void stop() {

    }
}