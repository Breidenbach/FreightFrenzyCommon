
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
// import com.vuforia.CameraDevice;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.Dictionary;
import java.util.Enumeration;
import java.util.Map;
import java.util.ArrayList;


/**
  *   Desired changes:
  *
  *		It is expected that some slip will occur when starting at full power on a
  *		well used playing field.  Therefore, it would be advantageous to ramp speed
  * 	up and down to reduce slip.
  *			-  Specify the amount of ramp as a fraction of wheel circumference.
  *			-  It will be necessary to check if the distance to go is less than 
  *				than twice the ramp request so that the ramp request value can be
  *				recalculated to 1/2 of the distance.
  *			-  Use a sin function to adjust the power for a smoother ramp.
  *
  *		In testing, the PID function caused the robot to severely swerve off course.
  *		This should be corrected.
  *
  *		Distance sensors could be integrated into the calculations.
  *
  *		The code below includes some methods that have integrated other operations
  *		besides moving the robot into the method.  These were for using the arm that 
  *     placed blocks in SkyStone.  The routines should be replace by or modified to use
  *     the newer accessory hardware.
  *
  **/
  
/**
 * The code REQUIRES that you DO have encoders on the wheels,
 *  This code ALSO requires that the drive Motors have been configured such that a positive
 *  power command moves them forwards, and causes the encoders to count UP.
 *
 *  This methods assumes that each movement is relative to the last stopping place.
 *  There are other ways to perform encoder based moves, but this method is probably the simplest.
 *  This code uses the RUN_TO_POSITION mode to enable the Motor controllers to generate the run
 *  profile
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 **/

public class AutoCommon {

    public AutoHardware robot = null;
    public DrivetrainCommon chassis = null;
//    public LiftClawCommon liftClaw = null;
//    public VuforiaSkyStoneCommon vuforiaCom = null;

    static final double     COUNTS_PER_MOTOR_REV    = 537.7 ;    // After gearbox
    static final double     WHEEL_CIRCUMFERENCE_CM   = 30.4 ;
    //static final double     COUNTS_PER_MOTOR_REV    =720 ;    // SkyStone motors
    //static final double     WHEEL_CIRCUMFERENCE_CM   = 31.0 ; // SkyStone wheels
    static final double		COUNTS_PER_CM			= COUNTS_PER_MOTOR_REV
                                                                / WHEEL_CIRCUMFERENCE_CM;
    public int BASE_DISTANCE = 2300;
    public int ADJUSTMENT_DISTANCE = 0;
    public int ARM_OFFSET = -173;
    public int BLOCK_OFFSET_DISTANCE=346;

    public VectorF blockLoc = null;
//    public CameraDevice vufCam = null;

    private ElapsedTime runtime = new ElapsedTime();

    private LinearOpMode curOpMode = null;

    public AutoCommon(LinearOpMode owningOpMode) {



        curOpMode = owningOpMode;

        chassis = new DrivetrainCommon(curOpMode);
     //   liftClaw = new LiftClawCommon(curOpMode);
        robot = new AutoHardware();
     //   vuforiaCom = new VuforiaSkyStoneCommon(curOpMode);
        robot.init(curOpMode.hardwareMap);

     //   liftClaw.robot.sideClaw.setPosition(0.1);
    //    vufCam = CameraDevice.getInstance();


    }


    /*
     *  Method to perfmorm a relative move, based on encoder counts.
     *  Encoders are not reset as the move is based on the current position.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the opmode running.
     */
    public void encoderDrive(double speed,
                             double distance,  // distance in CM
                             double timeoutS,
                             boolean pid) {
        int encoderValue = calcDesiredCount(distance);
        double correction = 0;

        resetEncoders();
        // Ensure that the opmode is still active
        if (curOpMode.opModeIsActive()) {


            if (pid) {
                correction = chassis.pidDrive.performPID(chassis.getAngle());
            }

            setAllTargetPosition(encoderValue);   //  pass encoderValue to all motors

			setAllRunToPosition();   // Turn on RUN_TO_POSITION

            // reset the timeout time and start motion.
            runtime.reset();
            chassis.robot.driveLF.setPower(Math.abs(speed));
            chassis.robot.driveRF.setPower(Math.abs(speed));
            chassis.robot.driveLR.setPower(Math.abs(speed));
            chassis.robot.driveRR.setPower(Math.abs(speed));

            double currentPower = speed;

            int decelStart = (int)(encoderValue*.75);
            // keep looping while we are still active, and there is time left, and both
            // motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that
            // when EITHER motor hits its target position, the motion will stop.  This
            // is "safer" in the event that the robot will 
            // always end the motion as soon as possible.  However, if you require that
            // BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (curOpMode.opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (chassis.robot.driveLF.isBusy() && chassis.robot.driveRF.isBusy()
                            && chassis.robot.driveLR.isBusy() && chassis.robot.driveRR.isBusy()
                    )) {


                chassis.robot.driveLF.setPower(Math.abs(currentPower - correction));
                chassis.robot.driveRF.setPower(Math.abs(currentPower + correction));
                chassis.robot.driveLR.setPower(Math.abs(currentPower - correction));
                chassis.robot.driveRR.setPower(Math.abs(currentPower + correction));

              // currentPower=rampUpDown(speed,currentPower,.2,chassis.robot.driveLF.getCurrentPosition(),decelStart);


                if(pid) {
                    correction = chassis.pidDrive.performPID(chassis.getAngle());
                    if(encoderValue<0) {
                        correction=correction*(-1);
                    }
                }
            }

            stopAllMotion();    			// set all to 0
            setAllRunUsingEncoder();  		// Turn on RUN_USING_ENCODER 

            //  sleep(250);   // optional pause after each move
        }
    }


    public void encoderDriveToDistance(double speed,
                             int encoderValue,
                             double timeoutS, double distance, boolean pid) {
        double correction = 0;


        resetEncoders();
        // Ensure that the opmode is still active
        if (curOpMode.opModeIsActive()) {


            if (pid) {
                correction = chassis.pidDrive.performPID(chassis.getAngle());
            }

            // Determine new target position, and pass to motor controller;
            setAllTargetPosition(encoderValue);

			setAllRunToPosition();   // Turn on RUN_TO_POSITION

            // reset the timeout time and start motion.
            runtime.reset();
            chassis.robot.driveLF.setPower(Math.abs(speed));
            chassis.robot.driveRF.setPower(Math.abs(speed));
            chassis.robot.driveLR.setPower(Math.abs(speed));
            chassis.robot.driveRR.setPower(Math.abs(speed));

            double currentPower = speed;

            int decelStart = (int)(encoderValue*.75);
            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (curOpMode.opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (chassis.robot.driveLF.isBusy() && chassis.robot.driveRF.isBusy()
                            && chassis.robot.driveLR.isBusy() && chassis.robot.driveRR.isBusy()
                    )) {


                chassis.robot.driveLF.setPower(Math.abs(currentPower - correction));
                chassis.robot.driveRF.setPower(Math.abs(currentPower + correction));
                chassis.robot.driveLR.setPower(Math.abs(currentPower - correction));
                chassis.robot.driveRR.setPower(Math.abs(currentPower + correction));

                // currentPower=rampUpDown(speed,currentPower,.2,chassis.robot.driveLF.getCurrentPosition(),decelStart);
/*
                if(liftClaw.robot.lift_check.getDistance(DistanceUnit.CM)<distance)
                {
                    //curOpMode.sleep(500);
                    chassis.robot.leftGuide.setPosition(.6);
                    chassis.robot.rightGuide.setPosition(.4);

                    curOpMode.sleep(500);
                    break;
                }

*/
                if(pid) {
                    correction = chassis.pidDrive.performPID(chassis.getAngle());

                    if(encoderValue<0) {
                        correction=correction*(-1);
                    }
                }
            }

            stopAllMotion();    			// set all to 0
            setAllRunUsingEncoder();  		// Turn on RUN_USING_ENCODER 

            //  sleep(250);   // optional pause after each move
        }
    }


    public void encoderDriveWithDrift(double leftSpeed,double rightSpeed,
                             int encoderValue,
                             double timeoutS) {


        resetEncoders();
        // Ensure that the opmode is still active
        if (curOpMode.opModeIsActive()) {

            setAllTargetPosition(encoderValue);   //  pass encoderValue to all motors

			setAllRunToPosition();   // Turn on RUN_TO_POSITION

            // reset the timeout time and start motion.
            runtime.reset();
            chassis.robot.driveLF.setPower(Math.abs(leftSpeed));
            chassis.robot.driveRF.setPower(Math.abs(rightSpeed));
            chassis.robot.driveLR.setPower(Math.abs(leftSpeed));
            chassis.robot.driveRR.setPower(Math.abs(rightSpeed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (curOpMode.opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (chassis.robot.driveLF.isBusy() && chassis.robot.driveRF.isBusy()
                            && chassis.robot.driveLR.isBusy() && chassis.robot.driveRR.isBusy()
                    )) {

                chassis.robot.driveLF.setPower(Math.abs(leftSpeed));
                chassis.robot.driveRF.setPower(Math.abs(rightSpeed));
                chassis.robot.driveLR.setPower(Math.abs(leftSpeed));
                chassis.robot.driveRR.setPower(Math.abs(rightSpeed));


            }

            stopAllMotion();    			// set all to 0
            setAllRunUsingEncoder();  		// Turn on RUN_USING_ENCODER 

            //  sleep(250);   // optional pause after each move
        }
    }

    public VectorF objectCheck(double timeoutS) {

        // Ensure that the opmode is still active
        if (curOpMode.opModeIsActive()) {

            // reset the timeout time and start motion.
            runtime.reset();
/*
            CameraDevice.getInstance().setFlashTorchMode(true);
            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (curOpMode.opModeIsActive() &&
                    (runtime.seconds() < timeoutS)
            ) {

                blockLoc = vuforiaCom.executeDetection();
            }

            CameraDevice.getInstance().setFlashTorchMode(false);
*/

        }
        return blockLoc;
    }

    public void encoderDriveObjectCheck(double speed,
                                        int encoderValue,
                                        double timeoutS, boolean pid) {
        double correction = 0;

        resetEncoders();

        // Ensure that the opmode is still active
        if (curOpMode.opModeIsActive()) {


            if (pid) {
                correction = chassis.pidDrive.performPID(chassis.getAngle());
            }

            setAllTargetPosition(encoderValue);   //  pass encoderValue to all motors

			setAllRunToPosition();   // Turn on RUN_TO_POSITION

            // reset the timeout time and start motion.
            runtime.reset();
            chassis.robot.driveLF.setPower(Math.abs(speed));
            chassis.robot.driveRF.setPower(Math.abs(speed));
            chassis.robot.driveLR.setPower(Math.abs(speed));
            chassis.robot.driveRR.setPower(Math.abs(speed));


            //   CameraDevice.getInstance().setFlashTorchMode(true);
            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (curOpMode.opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (chassis.robot.driveLF.isBusy() && chassis.robot.driveRF.isBusy()
                            && chassis.robot.driveLR.isBusy() && chassis.robot.driveRR.isBusy()
                    )) {

                chassis.robot.driveLF.setPower(Math.abs(speed - correction));
                chassis.robot.driveRF.setPower(Math.abs(speed + correction));
                chassis.robot.driveLR.setPower(Math.abs(speed - correction));
                chassis.robot.driveRR.setPower(Math.abs(speed + correction));


         //       blockLoc = vuforiaCom.executeDetection();

                if (blockLoc != null) {

                    break;
                }

            }


//            CameraDevice.getInstance().setFlashTorchMode(false);

            stopAllMotion();    			// set all to 0
            setAllRunUsingEncoder();  		// Turn on RUN_USING_ENCODER 

            //  sleep(250);   // optional pause after each move
        }
    }

    /*
     *  Method to perfmorm a relative move, based on encoder counts.
     *  Encoders are not reset as the move is based on the current position.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the opmode running.
     */
    public void encoderDriveCloseClamps(double speed,
                                        int encoderValue,
                                        double timeoutS, boolean pid, int closeAtValue) {
        double correction = 0;

        resetEncoders();
        // Ensure that the opmode is still active
        if (curOpMode.opModeIsActive()) {


            if (pid) {
                correction = chassis.pidDrive.performPID(chassis.getAngle());
            }

            setAllTargetPosition(encoderValue);   //  pass encoderValue to all motors

			setAllRunToPosition();   // Turn on RUN_TO_POSITION

            // reset the timeout time and start motion.
            runtime.reset();
            chassis.robot.driveLF.setPower(Math.abs(speed));
            chassis.robot.driveRF.setPower(Math.abs(speed));
            chassis.robot.driveLR.setPower(Math.abs(speed));
            chassis.robot.driveRR.setPower(Math.abs(speed));


            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (curOpMode.opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (chassis.robot.driveLF.isBusy() && chassis.robot.driveRF.isBusy()
                            && chassis.robot.driveLR.isBusy() && chassis.robot.driveRR.isBusy()
                    )) {

                correction = chassis.pidDrive.performPID(chassis.getAngle());

                chassis.robot.driveLF.setPower(Math.abs(speed - correction));
                chassis.robot.driveRF.setPower(Math.abs(speed + correction));
                chassis.robot.driveLR.setPower(Math.abs(speed - correction));
                chassis.robot.driveRR.setPower(Math.abs(speed + correction));

/*
                if(liftClaw.robot.lift_check.getDistance(DistanceUnit.CM)<2)
                {
                    liftClaw.engageGrabbers();

                    break;
                }
*/
                //if (chassis.robot.driveLF.getCurrentPosition() >= closeAtValue) {
                //    liftClaw.engageGrabbers();
                //}

            }

            stopAllMotion();    			// set all to 0
            setAllRunUsingEncoder();  		// Turn on RUN_USING_ENCODER 
            
            //  sleep(250);   // optional pause after each move
        }
    }

    public void encoderTurn(double speed,
                            int encoderValue,
                            double timeoutS) {

        resetEncoders();
        // Ensure that the opmode is still active
        if (curOpMode.opModeIsActive()) {


            double correction = chassis.pidDrive.performPID(chassis.getAngle());

            // Determine new target position, and pass to motor controller;
            chassis.robot.driveLF.setTargetPosition(-encoderValue);
            chassis.robot.driveRF.setTargetPosition(encoderValue);
            chassis.robot.driveLR.setTargetPosition(-encoderValue);
            chassis.robot.driveRR.setTargetPosition(encoderValue);

			setAllRunToPosition();   // Turn on RUN_TO_POSITION

            // reset the timeout time and start motion.
            runtime.reset();
            chassis.robot.driveLF.setPower(Math.abs(speed));
            chassis.robot.driveRF.setPower(Math.abs(speed));
            chassis.robot.driveLR.setPower(Math.abs(speed));
            chassis.robot.driveRR.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (curOpMode.opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (chassis.robot.driveLF.isBusy() && chassis.robot.driveRF.isBusy()
                            && chassis.robot.driveLR.isBusy() && chassis.robot.driveRR.isBusy()
                    )) {

                chassis.robot.driveLF.setPower(Math.abs(-speed));
                chassis.robot.driveRF.setPower(Math.abs(speed));
                chassis.robot.driveLR.setPower(Math.abs(-speed));
                chassis.robot.driveRR.setPower(Math.abs(speed));


            }

            stopAllMotion();    			// set all to 0
            setAllRunUsingEncoder();  		// Turn on RUN_USING_ENCODER 


            curOpMode.sleep(500);   // optional pause after each move

            chassis.rotation = chassis.getAngle();
            // reset angle tracking on new heading.
            chassis.resetAngle();
        }
    }


    public void driveToBlockPosition(double blockOffset){
        int encoderValue = (int)((blockOffset/25.4)*43.3);


        encoderValue=encoderValue-ARM_OFFSET;

        ADJUSTMENT_DISTANCE=encoderValue;

        curOpMode.telemetry.addData("Offset:", blockOffset);
        curOpMode.telemetry.addData("Encoder Value", encoderValue);

        curOpMode.telemetry.update();

        encoderDrive(.3, encoderValue, 5, false);


    }

    public VectorF encoderLateral(double power, double timeoutS, double distance, boolean strafeLeft,
        boolean pid, boolean objectDetection) {
        int encoderValue = calcDesiredCount(distance);
        chassis.rotation = chassis.getAngle();        // reset angle tracking on new heading.
        chassis.resetAngle();

        double powerRightRear;
        double powerLeftRear;
        double powerLeftFront;
        double powerRightFront;

        double correction=0;

        double currentPower=.1;

        runtime.reset();

        VectorF blockLoc = null;

        resetEncoders();
     //   CameraDevice.getInstance().setFlashTorchMode(true);

        int setA = 1;
        int setB = 1;

        if(strafeLeft)
        {
            setA=-1;
            setB=1;
        }
        else
        {
            setA=1;
            setB=-1;
        }

        // Determine new target position, and pass to motor controller;
        chassis.robot.driveLF.setTargetPosition(encoderValue*setA);
        chassis.robot.driveRF.setTargetPosition(encoderValue*setB);
        chassis.robot.driveLR.setTargetPosition(encoderValue*setB);
        chassis.robot.driveRR.setTargetPosition(encoderValue*setA);

		setAllRunToPosition();
        // Turn On RUN_TO_POSITION


        int startDecelerationAt = (int) (encoderValue * .95);


        while (curOpMode.opModeIsActive() &&
                (runtime.seconds() < timeoutS) &&
                (chassis.robot.driveLF.isBusy() && chassis.robot.driveRF.isBusy()
                && chassis.robot.driveLR.isBusy() && chassis.robot.driveRR.isBusy())
        )
        {

            currentPower=power;//rampUpDown(power,currentPower,.1,chassis.robot.driveRR.getCurrentPosition(),startDecelerationAt);

            chassis.robot.driveRR.setPower(currentPower);
            chassis.robot.driveLR.setPower(currentPower);
            chassis.robot.driveLF.setPower(currentPower);
            chassis.robot.driveRF.setPower(currentPower);
/*
            if(objectDetection) {
                blockLoc = vuforiaCom.executeDetection();

                if (blockLoc != null) {
                    break;
                }
            }
 */
        }

        stopAllMotion();
		setAllRunUsingEncoder();

   //     CameraDevice.getInstance().setFlashTorchMode(false);

        return blockLoc;
    }


    public double rampUpDown(double maxPower, double curPower, double minPower, int curPosition, int decelStartPosition)
    {

        double returnPower;

            if(Math.abs(curPosition)<Math.abs(decelStartPosition) && curPower<maxPower)
            {
                returnPower=curPower + .01;
            }
            else if(Math.abs(curPosition)>=Math.abs(decelStartPosition) && curPower > minPower)
            {
                returnPower=curPower - .01;
            }
            else
            {
                returnPower=curPower;
            }

            return returnPower;
    }


    public VectorF strafeToDistance(double slideSlowPower, double timeoutS, double distance,
                                        DistanceSensor sensor, boolean objectDetection) {

        chassis.rotation = chassis.getAngle();        // reset angle tracking on new heading.
        chassis.resetAngle();

        double powerRightRear;
        double powerLeftRear;
        double powerLeftFront;
        double powerRightFront;

        double correction=0;

        runtime.reset();

        VectorF blockLoc = null;
/*
        if(objectDetection) {
            CameraDevice.getInstance().setFlashTorchMode(true);
        }
        */

        while (curOpMode.opModeIsActive() &&
                (runtime.seconds() < timeoutS) )
       {


           if((slideSlowPower>0 && sensor.getDistance(DistanceUnit.CM) > distance) ||
                   (slideSlowPower<0 && sensor.getDistance(DistanceUnit.CM) < distance))
           {
               break;
           }


           correction = 0;//chassis.pidDrive.performPID(chassis.getAngle());
           //Front Motors
           powerLeftFront = -slideSlowPower-correction;
           powerRightFront = slideSlowPower+correction;

           //Rear Motors
           powerRightRear = -slideSlowPower+correction;
           powerLeftRear = slideSlowPower-correction;

           chassis.robot.driveRR.setPower(powerRightRear);
           chassis.robot.driveLR.setPower(powerLeftRear);
           chassis.robot.driveLF.setPower(powerLeftFront);
           chassis.robot.driveRF.setPower(powerRightFront);
/*
           if(objectDetection) {
               blockLoc = vuforiaCom.executeDetection();

               if (blockLoc != null) {
                   break;
               }
           }

 */
        }
/*
        if(objectDetection) {
            CameraDevice.getInstance().setFlashTorchMode(false);
        }
*/
        chassis.robot.driveLF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        chassis.robot.driveRF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        chassis.robot.driveLR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        chassis.robot.driveRR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        stopAllMotion();    			// set all to 0
        setAllRunUsingEncoder();  		// Turn on RUN_USING_ENCODER 

        return blockLoc;
    }

    public VectorF strafeAwayDistance(double slideSlowPower, double timeoutS, double distance,
                                    DistanceSensor sensor, boolean objectDetection) {

        chassis.rotation = chassis.getAngle();        // reset angle tracking on new heading.
        chassis.resetAngle();

        double powerRightRear;
        double powerLeftRear;
        double powerLeftFront;
        double powerRightFront;

        double correction=0;

        runtime.reset();

        VectorF blockLoc = null;

//        CameraDevice.getInstance().setFlashTorchMode(true);

        while (curOpMode.opModeIsActive() &&
                (runtime.seconds() < timeoutS) && (sensor.getDistance(DistanceUnit.CM) < distance))
        {

            correction = 0;//chassis.pidDrive.performPID(chassis.getAngle());
            //Front Motors
            powerLeftFront = -slideSlowPower-correction;
            powerRightFront = slideSlowPower+correction;

            //Rear Motors
            powerRightRear = -slideSlowPower+correction;
            powerLeftRear = slideSlowPower-correction;

            chassis.robot.driveRR.setPower(powerRightRear);
            chassis.robot.driveLR.setPower(powerLeftRear);
            chassis.robot.driveLF.setPower(powerLeftFront);
            chassis.robot.driveRF.setPower(powerRightFront);
/*
            if(objectDetection) {
                blockLoc = vuforiaCom.executeDetection();

                if (blockLoc != null) {
                    break;
                }
            }

 */
        }

//        CameraDevice.getInstance().setFlashTorchMode(false);

        chassis.robot.driveLF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        chassis.robot.driveRF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        chassis.robot.driveLR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        chassis.robot.driveRR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        stopAllMotion();    			// set all to 0
        setAllRunUsingEncoder();  		// Turn on RUN_USING_ENCODER 


        return blockLoc;
    }



    public void encoderDriveAndLift(double speed,
                             int encoderValue, int liftPosition, double liftSpeed,
                             double timeoutS, boolean pid) {

        double correction = 0;


        resetEncoders();
        // Ensure that the opmode is still active
        if (curOpMode.opModeIsActive()) {

            if (pid) {
                correction = chassis.pidDrive.performPID(chassis.getAngle());
            }

            setAllTargetPosition(encoderValue);   //  pass encoderValue to all motors

			setAllRunToPosition();   // Turn on RUN_TO_POSITION

            // reset the timeout time and start motion.
            runtime.reset();
            chassis.robot.driveLF.setPower(Math.abs(speed));
            chassis.robot.driveRF.setPower(Math.abs(speed));
            chassis.robot.driveLR.setPower(Math.abs(speed));
            chassis.robot.driveRR.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.


        //    liftClaw.robot.lift.setPower(liftSpeed);

            while (curOpMode.opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (chassis.robot.driveLF.isBusy() && chassis.robot.driveRF.isBusy()
                            && chassis.robot.driveLR.isBusy() && chassis.robot.driveRR.isBusy()
                    )) {

                chassis.robot.driveLF.setPower(Math.abs(speed - correction));
                chassis.robot.driveRF.setPower(Math.abs(speed + correction));
                chassis.robot.driveLR.setPower(Math.abs(speed - correction));
                chassis.robot.driveRR.setPower(Math.abs(speed + correction));

/*
                if(liftClaw.robot.lift.getCurrentPosition()>=liftPosition)
                {
                    liftClaw.robot.lift.setPower(0);
                }

 */
            }
            
            stopAllMotion();    			// set all to 0

/*
            if(liftClaw.robot.lift.getCurrentPosition()<liftPosition) {
                while (curOpMode.opModeIsActive() && runtime.seconds() < timeoutS &&
                        liftClaw.robot.lift.getCurrentPosition() < liftPosition) {

                    liftClaw.robot.lift.setPower(liftSpeed);
                }


            }
*/

            setAllRunUsingEncoder();  		// Turn on RUN_USING_ENCODER 

         //   liftClaw.robot.lift.setPower(0);
            //  sleep(250);   // optional pause after each move
        }
    }

/*
    public void unsetEncoders()
    {
        // Stop all motion;
        chassis.robot.driveLF.setPower(0);
        chassis.robot.driveRF.setPower(0);
        chassis.robot.driveLR.setPower(0);
        chassis.robot.driveRR.setPower(0);

        chassis.robot.driveRR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        chassis.robot.driveLR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        chassis.robot.driveLF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        chassis.robot.driveRF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

 */
 
	/**
	 *  resetEncoders -- reset encoders for all motors, then turn on RUN_USING_ENCODER
	 **/
  
    public void resetEncoders() {
        //Reset the encoders
        chassis.robot.driveLF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        chassis.robot.driveRF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        chassis.robot.driveLR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        chassis.robot.driveRR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        setAllRunUsingEncoder();  		// Turn on RUN_USING_ENCODER 

    }

 
    /**
     * setAllTargetPosition -- set all motors to a common target encoder value
     **/
     
    private void setAllTargetPosition(int encoderValue){
        chassis.robot.driveLF.setTargetPosition(encoderValue);
        chassis.robot.driveRF.setTargetPosition(encoderValue);
        chassis.robot.driveLR.setTargetPosition(encoderValue);
        chassis.robot.driveRR.setTargetPosition(encoderValue);

    }
 
    /**
     *   setAllRunToPosition -- set all motors to RUN_TO_POSITION
     */
 
 	private void setAllRunToPosition() {
 	    chassis.robot.driveLF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        chassis.robot.driveRF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        chassis.robot.driveLR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        chassis.robot.driveRR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
	}

    /**
     *   setAllRunUsingEncoder -- set all motors to RUN_USING_ENCODER
     */
    private void setAllRunUsingEncoder() {
        chassis.robot.driveLF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        chassis.robot.driveRF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        chassis.robot.driveLR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        chassis.robot.driveRR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    /**
     *   stopAllMotion -- stop all 4 motors.
     */
    private void stopAllMotion(){
        chassis.robot.driveLF.setPower(0);
        chassis.robot.driveRF.setPower(0);
        chassis.robot.driveLR.setPower(0);
        chassis.robot.driveRR.setPower(0);
    }

    /**
     *  calculateDesiredCount:  Use desired distance to calculate the counts needed for
     *   the move.
     *
     **/

    public int calcDesiredCount (double distance){
        int count = (int) (distance * COUNTS_PER_CM);
        return count;
    }
}
