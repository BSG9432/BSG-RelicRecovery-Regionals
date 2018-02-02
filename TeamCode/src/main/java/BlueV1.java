package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

/**
 * Created by 299876 on 12/14/2017.
 */




@Autonomous(name="BlueV1", group="V")
public class BlueV1 extends LinearOpMode {
    public static final String TAG = "Vuforia VuMark Sample";

    OpenGLMatrix lastLocation = null;

    DcMotor LeftWheel;
    DcMotor RightWheel;
    DcMotor Clamp;
    DcMotor Clamp2;
    DcMotor LiftL;
    DcMotor LiftR;
    Servo jewelL;
    Servo jewelR;
    ColorSensor colorLeft;
    ColorSensor colorRight;
    DcMotor RightWheel2;
    public DcMotor LeftWheel2;
    int i = 0;

	//Creates variables for distances used later
    double r = 41.5;
    double l = 26.5;
    double c = 34;

	// calculates the number of encoder tics for every inch the robot moves
    private double TICKS_PER_IN = 1120/(4*Math.PI);

	//function that takes in a double for inches, left power, and right power
    public void driveDistance(double inches, double leftPower, double rightPower)
    {

        LeftWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RightWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        LeftWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RightWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        /*create two integers that use the current motor
        position, the inches given, and the calculation from motor
        tics to inches in order to tell the motor how far to go for each side*/
        int leftTickGoal = LeftWheel.getCurrentPosition () + (int) (TICKS_PER_IN * inches);
        int rightTickGoal = RightWheel.getCurrentPosition() + (int) (TICKS_PER_IN * inches);

        /*sets the motors to the new goal created above*/
        LeftWheel.setTargetPosition(leftTickGoal);
        RightWheel.setTargetPosition(rightTickGoal);

        /*tells the motors to continue moving until the new goal is acheived*/
        while( Math.abs(LeftWheel.getCurrentPosition()) < leftTickGoal || Math.abs(RightWheel.getCurrentPosition()) < rightTickGoal)
        {

		/*sets second motors on left and right sides to the
		current power of the motor on the corresponding side*/
            LeftWheel2.setPower(LeftWheel.getPower());
            RightWheel2.setPower(RightWheel.getPower());

		//tells motors to go to the position set above
            LeftWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            RightWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);

		/*sets second motors on left and right sides to the
		current power of the motor on the corresponding side*/
            LeftWheel2.setPower(LeftWheel.getPower());
            RightWheel2.setPower(RightWheel.getPower());

		//sets left and right motors to power given 
            RightWheel.setPower(rightPower);//posss -1
            LeftWheel.setPower(leftPower);//poss +1

		/*sets second motors on left and right sides to the
		current power of the motor on the corresponding side*/
            LeftWheel2.setPower(LeftWheel.getPower());
            RightWheel2.setPower(RightWheel.getPower());

            telemetry.addData("Left Enc ", LeftWheel.getCurrentPosition());
            telemetry.addData("Right Enc ", RightWheel.getCurrentPosition());
            updateTelemetry(telemetry);
            telemetry.update();
        }

		//tell all motors to stop once the required distance is reached
        LeftWheel.setPower(0);
        RightWheel.setPower(0);
        LeftWheel2.setPower(0);
        RightWheel2.setPower(0);

    }



    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    VuforiaLocalizer vuforia;




    @Override
    public void runOpMode() throws InterruptedException {

		//tells the program what the devices are named in the hardware map on the phone
        LeftWheel = hardwareMap.dcMotor.get("leftwheel");
        RightWheel = hardwareMap.dcMotor.get("rightwheel");
        LeftWheel2 = hardwareMap.dcMotor.get("leftwheel 2");
        RightWheel2 = hardwareMap.dcMotor.get("rightwheel 2");
        Clamp = hardwareMap.dcMotor.get("cleft");
        Clamp2 = hardwareMap.dcMotor.get("cright");
        LiftL = hardwareMap.dcMotor.get("liftL");
        LiftR = hardwareMap.dcMotor.get("liftR");
        jewelL = hardwareMap.servo.get("sensor l");
        jewelR = hardwareMap.servo.get("sensor r");
        colorLeft = hardwareMap.colorSensor.get("color l");
        colorRight = hardwareMap.colorSensor.get("color r");

		/*tell jewel servo to be up in order to stay inside the cnstraints during initialization*/
        jewelL.setPosition(.85);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        parameters.vuforiaLicenseKey = "Af5QfN3/////AAAAGbgriAs0LEQ3ocxVc4HpCRklEU7eg4hNfd768FvbnRbrdELeVi0frSJTD9bjHWsSwduesJ2rMSDXF+OH+lMuzvmx11jF8UFUdEl2IswOmZdLJJOyfx5XA71yihvFRElfBDO0NrC4kHKmWR+PKhVWIxPbjXRw85+3ddHiAIvfWRxys1ZqT7Mt3l6i2tLDg/kLSYTL8edY37Bx399lh0LJJ6nH5/0ADVxHAunuVxldTIEqWjPPRgoRCXLM9qMy6d9PcGzggfQ1Xep6PzXBK91vvTj8jde2ITnWWgvVBWvPkv+9YhbI36tRMxF6k4diwD4a2DwmvdhN3ATgIeQkeYkCSD+0QYqqI5fdDXmx08Bpku74\n";

		//chooses camera to use to search for the pictograph
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);

		//loads pictographs to match the picture on the phone
        VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        VuforiaTrackable relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate"); // can help in debugging; otherwise not necessary

        telemetry.addData(">", "Press Play to start");
        telemetry.update();

        LeftWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RightWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

		//reverses the left motors so that positive power makes it go forward
        LeftWheel.setDirection(DcMotorSimple.Direction.REVERSE);
        LeftWheel2.setDirection(DcMotorSimple.Direction.REVERSE);


        //sets encoder values to zero
        LeftWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RightWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        //transitions to initialized driver program
        org.firstinspires.ftc.teamcode.AutoTransitioner.transitionOnStop(this, "driverControl");

        waitForStart();

        relicTrackables.activate();

        Clamp.setPower(1);
        Clamp2.setPower(1);
        sleep(500);
        Clamp.setPower(.3);
        Clamp2.setPower(.3);

        LiftL.setPower(.5);
        LiftR.setPower(.5);
        sleep(250);
        LiftL.setPower(0);
        LiftR.setPower(0);
        sleep(250);

		//repeats the program a maximum of 2 times
        while (opModeIsActive() && i < 3) {

            RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
	
		    //attempts to match pictograph
            if (vuMark != RelicRecoveryVuMark.UNKNOWN) {

                RightWheel.setDirection(DcMotorSimple.Direction.FORWARD);
                RightWheel2.setDirection(DcMotorSimple.Direction.FORWARD);
                LeftWheel.setDirection(DcMotorSimple.Direction.REVERSE);
                LeftWheel2.setDirection(DcMotorSimple.Direction.REVERSE);

			    //displays whether it an see a pictograph
                telemetry.addData("VuMark", "%s visible", vuMark);

			    //displays which pictograph is visible on the driver station
                OpenGLMatrix pose = ((VuforiaTrackableDefaultListener)relicTemplate.getListener()).getPose();
                telemetry.addData("Pose", format(pose));

			    // moves if it detects a pictograph pose
                if (pose != null) {
                    VectorF trans = pose.getTranslation();
                    Orientation rot = Orientation.getOrientation(pose, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);

                    // Extract the X, Y, and Z components of the offset of the target relative to the robot controller
                    double tX = trans.get(0);
                    double tY = trans.get(1);
                    double tZ = trans.get(2);

                    // Extract the rotational components of the target relative to the robot controller
                    double rX = rot.firstAngle;
                    double rY = rot.secondAngle;
                    double rZ = rot.thirdAngle;

				    //raises the lift to prevent dragging
                    LiftL.setPower(.5);
                    LiftR.setPower(.5);
                    sleep(250);
                    LiftL.setPower(0);
                    LiftR.setPower(0);
                    sleep(250);

				    //slowly puts down color senseor for the jewels
                    for (double i = .9; i>=0;i-=.1) {
                        jewelL.setPosition(0);
                        sleep(5);
                    }
                    sleep(2000);

                    //if it detects red, it moves forward to double check
                    if (colorLeft.red() > colorLeft.blue())
                    {

                        LeftWheel.setDirection(DcMotorSimple.Direction.REVERSE);
                        LeftWheel2.setDirection(DcMotorSimple.Direction.REVERSE);
                        RightWheel.setDirection(DcMotorSimple.Direction.FORWARD);
                        RightWheel2.setDirection(DcMotorSimple.Direction.FORWARD);

				/* considering light is naturally red, the
				robot moves forward to detect again if it sees red in order to prevent moving the
				incorrect color*/
                        driveDistance(1.5,.25,.25);

						/*removes one inch from the measured distance to compensate for the inch that it moved forward*/
			             r -= 1.5;
			             l -= 1.5;
			             c -= 1.5;

                        //stops all drive motors
                        LeftWheel.setPower(0);
                        LeftWheel2.setPower(0);
                        RightWheel.setPower(0);
                        RightWheel2.setPower(0);
                        sleep(1000);


                        //if it reads red again, it rotates counterclockwise
                        if (colorLeft.red() > colorLeft.blue()) {


                            // reverses the right motors so that positive numbers rotate the robot counterclockwise

                            RightWheel.setDirection(DcMotorSimple.Direction.REVERSE);
                            RightWheel2.setDirection(DcMotorSimple.Direction.REVERSE);
                            LeftWheel.setDirection(DcMotorSimple.Direction.REVERSE);
                            LeftWheel2.setDirection(DcMotorSimple.Direction.REVERSE);


                            //moves both sides to their encoder goals at 1/4 power
                            driveDistance(3, .25, .25);

                            //stops all drive motors
                            LeftWheel.setPower(0);
                            LeftWheel2.setPower(0);
                            RightWheel.setPower(0);
                            RightWheel2.setPower(0);
                            sleep(500);

                            //sets encoders back to 0

                            LeftWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                            RightWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                            

                            //set both sides forwad to rotate the robot clockwise to readjust straight

                            RightWheel.setDirection(DcMotorSimple.Direction.FORWARD);
                            RightWheel2.setDirection(DcMotorSimple.Direction.FORWARD);
                            LeftWheel.setDirection(DcMotorSimple.Direction.FORWARD);
                            LeftWheel2.setDirection(DcMotorSimple.Direction.FORWARD);


                            //puts up color senor
                            jewelL.setPosition(.85);


                            //stops all drive motors
                            LeftWheel.setPower(0);
                            LeftWheel2.setPower(0);
                            RightWheel.setPower(0);
                            RightWheel2.setPower(0);
                            sleep(500);


                            //moves both sides to their encoder goals at 1/4 power
                            driveDistance(3, .25, .25);
                            

                            //sets encoders back to 0

                            LeftWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                            RightWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


                            //stops all drive motors
                            LeftWheel.setPower(0);
                            LeftWheel2.setPower(0);
                            RightWheel.setPower(0);
                            RightWheel2.setPower(0);
                            sleep(500);
                        }

                        //if it detects blue rotate clockwise

                        else if (colorLeft.red() < colorLeft.blue()) {
                            

                            // sets left motors forward so that positive numbers rotate the robot clockwise

                            LeftWheel.setDirection(DcMotorSimple.Direction.FORWARD);
                            LeftWheel2.setDirection(DcMotorSimple.Direction.FORWARD);


                            //moves both sides to their encoder goals at 1/4 power
                            driveDistance(3, .25, .25);
                            LeftWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                            RightWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                            //stops all drive motors
                            LeftWheel.setPower(0);
                            LeftWheel2.setPower(0);
                            RightWheel.setPower(0);
                            RightWheel2.setPower(0);
                            sleep(500);


                            //puts up color senor
                            jewelL.setPosition(.85);

                            //stops all drive motors
                            LeftWheel.setPower(0);
                            LeftWheel2.setPower(0);
                            RightWheel.setPower(0);
                            RightWheel2.setPower(0);
                            sleep(500);


                            //set both sides reverse to rotate the robot counterclockwise
                            LeftWheel.setDirection(DcMotorSimple.Direction.REVERSE);
                            LeftWheel2.setDirection(DcMotorSimple.Direction.REVERSE);
                            RightWheel.setDirection(DcMotorSimple.Direction.REVERSE);
                            RightWheel2.setDirection(DcMotorSimple.Direction.REVERSE);


                            //moves both sides to their encoder goals at 1/4 power
                            driveDistance(3, .25, .25);
                            

                            //sets encoders back to 0
                            LeftWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                            RightWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


                            //stops all drive motors
                            LeftWheel.setPower(0);
                            LeftWheel2.setPower(0);
                            RightWheel.setPower(0);
                            RightWheel2.setPower(0);
                            sleep(500);
                        }
                    }


                    //if it detects blue rotate clockwise
                    else if (colorLeft.red() < colorLeft.blue())
                    {


                        // sets left motors forward so that positive numbers rotate the robot clockwise

                        LeftWheel.setDirection(DcMotorSimple.Direction.FORWARD);
                        LeftWheel2.setDirection(DcMotorSimple.Direction.FORWARD);


                        //moves both sides to their encoder goals at 1/4 power
                        driveDistance(3,.25,.25);
                        

                        //sets encoders back to 0

                        LeftWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        RightWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


                        //puts up color senor
                        jewelL.setPosition(.85);

                        //stops all drive motors
                        LeftWheel.setPower(0);
                        LeftWheel2.setPower(0);
                        RightWheel.setPower(0);
                        RightWheel2.setPower(0);
                        sleep(500);
                        

                        //set both sides reverse to rotate the robot counterclockwise

                        LeftWheel.setDirection(DcMotorSimple.Direction.REVERSE);
                        LeftWheel2.setDirection(DcMotorSimple.Direction.REVERSE);
                        RightWheel.setDirection(DcMotorSimple.Direction.REVERSE);
                        RightWheel2.setDirection(DcMotorSimple.Direction.REVERSE);


                        //moves both sides to their encoder goals at 1/4 power
                        driveDistance(3,.25,.25);
                        

                        //sets encoders back to 0

                        LeftWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        RightWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


                        //stops all drive motors
                        LeftWheel.setPower(0);
                        LeftWheel2.setPower(0);
                        RightWheel.setPower(0);
                        RightWheel2.setPower(0);
                        sleep(500);
                    }
                    

                    //sets all motors so that positive powers move the robot forward

                    LeftWheel.setDirection(DcMotorSimple.Direction.REVERSE);
                    LeftWheel2.setDirection(DcMotorSimple.Direction.REVERSE);
                    RightWheel.setDirection(DcMotorSimple.Direction.FORWARD);
                    RightWheel2.setDirection(DcMotorSimple.Direction.FORWARD);

                    /* function that is used if the phone reads that the robot needs to put the glyph
                    in the right side of the cryptobox*/
                    if (vuMark == RelicRecoveryVuMark.RIGHT ){

                       //makes robot move the distance created above at half speed
                        driveDistance(r,.5,.5);

                       //sets all drive motors to zero power
                        LeftWheel.setPower(0);
                        LeftWheel2.setPower(0);
                        RightWheel.setPower(0);
                        RightWheel2.setPower(0);
                        sleep(100);

                       //sets encoder values to zero
                        LeftWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        RightWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                       //sets direction of both left wheels to foward so the robot can turn toward the cryptobox
                        LeftWheel.setDirection(DcMotorSimple.Direction.FORWARD);
                        LeftWheel2.setDirection(DcMotorSimple.Direction.FORWARD);

                       //sets robot to half speed and tells it how much distance to move
                       //turns 90
                        driveDistance(13.352, .5,.5);

                       //sets both left wheels to reverse so the robot moves forward
                        LeftWheel.setDirection(DcMotorSimple.Direction.REVERSE);
                        LeftWheel2.setDirection(DcMotorSimple.Direction.REVERSE);

                       //sets all motors to zero power
                        LeftWheel.setPower(0);
                        LeftWheel2.setPower(0);
                        RightWheel.setPower(0);
                        RightWheel2.setPower(0);
                        sleep(100);

                       //sets encoder values to zero
                        LeftWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        RightWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                       //sets robot to half speed and tells it how much distance to move
                        driveDistance(12,.5,.5);

                       //sets all motors to zero power
                        LeftWheel.setPower(0);
                        LeftWheel2.setPower(0);
                        RightWheel.setPower(0);
                        RightWheel2.setPower(0);
                        sleep(100);

                       //opens clamp to drop glyph into the column
                        Clamp.setPower(-.5);
                        Clamp2.setPower(-.5);
                        sleep(500);
                        Clamp.setPower(0);
                        Clamp2.setPower(0);

                        RightWheel.setDirection(DcMotorSimple.Direction.REVERSE);
                        RightWheel2.setDirection(DcMotorSimple.Direction.REVERSE);
                        LeftWheel.setDirection(DcMotorSimple.Direction.FORWARD);
                        LeftWheel2.setDirection(DcMotorSimple.Direction.FORWARD);

                        LeftWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        RightWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                        driveDistance(8, .25, .25);

                        //opens clamp REALLY wide
                        Clamp.setPower(-1);
                        Clamp2.setPower(-1);
                        sleep(500);
                        Clamp.setPower(0);
                        Clamp2.setPower(0);

                        RightWheel.setDirection(DcMotorSimple.Direction.FORWARD);
                        RightWheel2.setDirection(DcMotorSimple.Direction.FORWARD);
                        LeftWheel.setDirection(DcMotorSimple.Direction.REVERSE);
                        LeftWheel2.setDirection(DcMotorSimple.Direction.REVERSE);

                        LeftWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        RightWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                        driveDistance(11, .25, .25);
                        
                        RightWheel.setDirection(DcMotorSimple.Direction.REVERSE);
                        RightWheel2.setDirection(DcMotorSimple.Direction.REVERSE);
                        LeftWheel.setDirection(DcMotorSimple.Direction.FORWARD);
                        LeftWheel2.setDirection(DcMotorSimple.Direction.FORWARD);

                        LeftWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        RightWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                        driveDistance(4, .25, .25);

                        requestOpModeStop();
                        stop();
                    }

                    /* function that is used if the phone reads that the robot needs to put the glyph
                    in the center of the cryptobox*/
                    else if (vuMark == RelicRecoveryVuMark.CENTER){

                        driveDistance(c,.5,.5);

                        //sets all motors to zero power
                        LeftWheel.setPower(0);
                        LeftWheel2.setPower(0);
                        RightWheel.setPower(0);
                        RightWheel2.setPower(0);
                        sleep(100);

                        LeftWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        RightWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                        LeftWheel.setDirection(DcMotorSimple.Direction.FORWARD);
                        LeftWheel2.setDirection(DcMotorSimple.Direction.FORWARD);

                        driveDistance(13.352, .5,.5);

                        LeftWheel.setDirection(DcMotorSimple.Direction.REVERSE);
                        LeftWheel2.setDirection(DcMotorSimple.Direction.REVERSE);

                        //sets all motors to zero power
                        LeftWheel.setPower(0);
                        LeftWheel2.setPower(0);
                        RightWheel.setPower(0);
                        RightWheel2.setPower(0);
                        sleep(100);

                        LeftWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        RightWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                        driveDistance(12,.5,.5);

                        //sets all motors to zero power
                        LeftWheel.setPower(0);
                        LeftWheel2.setPower(0);
                        RightWheel.setPower(0);
                        RightWheel2.setPower(0);
                        sleep(100);

                        Clamp.setPower(-.5);
                        Clamp2.setPower(-.5);
                        sleep(500);
                        Clamp.setPower(0);
                        Clamp2.setPower(0);

                        RightWheel.setDirection(DcMotorSimple.Direction.REVERSE);
                        RightWheel2.setDirection(DcMotorSimple.Direction.REVERSE);
                        LeftWheel.setDirection(DcMotorSimple.Direction.FORWARD);
                        LeftWheel2.setDirection(DcMotorSimple.Direction.FORWARD);

                        LeftWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        RightWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                        driveDistance(8, .25, .25);

                        //opens clamp REALLY wide
                        Clamp.setPower(-1);
                        Clamp2.setPower(-1);
                        sleep(500);
                        Clamp.setPower(0);
                        Clamp2.setPower(0);

                        RightWheel.setDirection(DcMotorSimple.Direction.FORWARD);
                        RightWheel2.setDirection(DcMotorSimple.Direction.FORWARD);
                        LeftWheel.setDirection(DcMotorSimple.Direction.REVERSE);
                        LeftWheel2.setDirection(DcMotorSimple.Direction.REVERSE);

                        LeftWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        RightWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                        driveDistance(11, .25, .25);
                        
                        RightWheel.setDirection(DcMotorSimple.Direction.REVERSE);
                        RightWheel2.setDirection(DcMotorSimple.Direction.REVERSE);
                        LeftWheel.setDirection(DcMotorSimple.Direction.FORWARD);
                        LeftWheel2.setDirection(DcMotorSimple.Direction.FORWARD);

                        LeftWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        RightWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                        driveDistance(4, .25, .25);

                        requestOpModeStop();
                        stop();
                    }
                    else if (vuMark == RelicRecoveryVuMark.LEFT){

                        driveDistance(l,.5,.5);

                        //sets all motors to zero power
                        LeftWheel.setPower(0);
                        LeftWheel2.setPower(0);
                        RightWheel.setPower(0);
                        RightWheel2.setPower(0);
                        sleep(100);

                        LeftWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        RightWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                        LeftWheel.setDirection(DcMotorSimple.Direction.FORWARD);
                        LeftWheel2.setDirection(DcMotorSimple.Direction.FORWARD);

                        driveDistance(13.352, .5,.5);

                        LeftWheel.setDirection(DcMotorSimple.Direction.REVERSE);
                        LeftWheel2.setDirection(DcMotorSimple.Direction.REVERSE);


                        //sets all motors to zero power
                        LeftWheel.setPower(0);
                        LeftWheel2.setPower(0);
                        RightWheel.setPower(0);
                        RightWheel2.setPower(0);
                        sleep(100);

                        LeftWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        RightWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                        driveDistance(12,.5,.5);

                        //sets all motors to zero power
                        LeftWheel.setPower(0);
                        LeftWheel2.setPower(0);
                        RightWheel.setPower(0);
                        RightWheel2.setPower(0);
                        sleep(100);

                        Clamp.setPower(-.5);
                        Clamp2.setPower(-.5);
                        sleep(500);
                        Clamp.setPower(0);
                        Clamp2.setPower(0);

                        RightWheel.setDirection(DcMotorSimple.Direction.REVERSE);
                        RightWheel2.setDirection(DcMotorSimple.Direction.REVERSE);
                        LeftWheel.setDirection(DcMotorSimple.Direction.FORWARD);
                        LeftWheel2.setDirection(DcMotorSimple.Direction.FORWARD);

                        LeftWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        RightWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                        driveDistance(8, .25, .25);

                        //opens clamp REALLY wide
                        Clamp.setPower(-1);
                        Clamp2.setPower(-1);
                        sleep(500);
                        Clamp.setPower(0);
                        Clamp2.setPower(0);

                        RightWheel.setDirection(DcMotorSimple.Direction.FORWARD);
                        RightWheel2.setDirection(DcMotorSimple.Direction.FORWARD);
                        LeftWheel.setDirection(DcMotorSimple.Direction.REVERSE);
                        LeftWheel2.setDirection(DcMotorSimple.Direction.REVERSE);

                        LeftWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        RightWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                        driveDistance(11, .25, .25);
                        
                        RightWheel.setDirection(DcMotorSimple.Direction.REVERSE);
                        RightWheel2.setDirection(DcMotorSimple.Direction.REVERSE);
                        LeftWheel.setDirection(DcMotorSimple.Direction.FORWARD);
                        LeftWheel2.setDirection(DcMotorSimple.Direction.FORWARD);

                        LeftWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        RightWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                        driveDistance(4, .25, .25);

                        requestOpModeStop();
                        stop();
                    }

                        relicTrackables.deactivate();
                    
                }
            }
            else {
                telemetry.addData("VuMark", "not visible");
                RightWheel.setDirection(DcMotorSimple.Direction.REVERSE);
                RightWheel2.setDirection(DcMotorSimple.Direction.REVERSE);
                LeftWheel.setDirection(DcMotorSimple.Direction.FORWARD);
                LeftWheel2.setDirection(DcMotorSimple.Direction.FORWARD);

                LeftWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                RightWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                driveDistance(.5, .25, .25);
			    c+=.5;
			    r+=.5;
			    l+=.5;

                //sets all motors to zero power
			    LeftWheel.setPower(0);
			    LeftWheel2.setPower(0);
                RightWheel.setPower(0);
                RightWheel2.setPower(0);
                sleep(100);
                i ++;
            }

            telemetry.update();
        }

        sleep(1000);

        RightWheel.setDirection(DcMotorSimple.Direction.FORWARD);
        RightWheel2.setDirection(DcMotorSimple.Direction.FORWARD);
        LeftWheel.setDirection(DcMotorSimple.Direction.REVERSE);
        LeftWheel2.setDirection(DcMotorSimple.Direction.REVERSE);

        driveDistance(1.5, .25, .25);


        //sets all motors to zero power
        LeftWheel.setPower(0);
        LeftWheel2.setPower(0);
        RightWheel.setPower(0);
        RightWheel2.setPower(0);
        sleep(100);


        for (double i = .9; i >= 0; i -= .1) {
            jewelL.setPosition(i);
            sleep(10);
        }
        sleep(2000);


        if (colorLeft.red() > colorLeft.blue())
        {

            LeftWheel.setDirection(DcMotorSimple.Direction.REVERSE);
            LeftWheel2.setDirection(DcMotorSimple.Direction.REVERSE);
            RightWheel.setDirection(DcMotorSimple.Direction.FORWARD);
            RightWheel2.setDirection(DcMotorSimple.Direction.FORWARD);

				/* considering light is naturally red, the
				robot moves forward to detect again if it sees red in order to prevent moving the
				incorrect color*/
            driveDistance(1.01,.25,.25);

						/*removes one inch from the measured distance to compensate for the inch that it moved forward*/
            r -= 1;
            l -= 1;
            c -= 1;

            //stops all drive motors
            LeftWheel.setPower(0);
            LeftWheel2.setPower(0);
            RightWheel.setPower(0);
            RightWheel2.setPower(0);
            sleep(1000);


            //if it reads red again, it rotates counterclockwise
            if (colorLeft.red() > colorLeft.blue()) {


                // reverses the right motors so that positive numbers rotate the robot counterclockwise


                RightWheel.setDirection(DcMotorSimple.Direction.REVERSE);
                RightWheel2.setDirection(DcMotorSimple.Direction.REVERSE);
                LeftWheel.setDirection(DcMotorSimple.Direction.REVERSE);
                LeftWheel2.setDirection(DcMotorSimple.Direction.REVERSE);

                //moves both sides to their encoder goals at 1/4 power
                driveDistance(3, .25, .25);


                //stops all drive motors
                LeftWheel.setPower(0);
                LeftWheel2.setPower(0);
                RightWheel.setPower(0);
                RightWheel2.setPower(0);
                sleep(500);

                //sets encoders back to 0

                LeftWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                RightWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);



                //set both sides forwad to rotate the robot clockwise to readjust straight

                RightWheel.setDirection(DcMotorSimple.Direction.FORWARD);
                RightWheel2.setDirection(DcMotorSimple.Direction.FORWARD);
                LeftWheel.setDirection(DcMotorSimple.Direction.FORWARD);
                LeftWheel2.setDirection(DcMotorSimple.Direction.FORWARD);


                //puts up color senor
                jewelL.setPosition(.85);


                //stops all drive motors
                LeftWheel.setPower(0);
                LeftWheel2.setPower(0);
                RightWheel.setPower(0);
                RightWheel2.setPower(0);
                sleep(500);


                //moves both sides to their encoder goals at 1/4 power
                driveDistance(3, .25, .25);


                //sets encoders back to 0

                LeftWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                RightWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


                //stops all drive motors
                LeftWheel.setPower(0);
                LeftWheel2.setPower(0);
                RightWheel.setPower(0);
                RightWheel2.setPower(0);
                sleep(500);
            }

            //if it detects blue rotate clockwise

            else if (colorLeft.red() < colorLeft.blue()) {


                // sets left motors forward so that positive numbers rotate the robot clockwise

                LeftWheel.setDirection(DcMotorSimple.Direction.FORWARD);
                LeftWheel2.setDirection(DcMotorSimple.Direction.FORWARD);


                //moves both sides to their encoder goals at 1/4 power
                driveDistance(3, .25, .25);
                LeftWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                RightWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                //stops all drive motors
                LeftWheel.setPower(0);
                LeftWheel2.setPower(0);
                RightWheel.setPower(0);
                RightWheel2.setPower(0);
                sleep(500);


                //puts up color senor
                jewelL.setPosition(.85);

                //stops all drive motors
                LeftWheel.setPower(0);
                LeftWheel2.setPower(0);
                RightWheel.setPower(0);
                RightWheel2.setPower(0);
                sleep(500);


                //set both sides reverse to rotate the robot counterclockwise
                LeftWheel.setDirection(DcMotorSimple.Direction.REVERSE);
                LeftWheel2.setDirection(DcMotorSimple.Direction.REVERSE);
                RightWheel.setDirection(DcMotorSimple.Direction.REVERSE);
                RightWheel2.setDirection(DcMotorSimple.Direction.REVERSE);


                //moves both sides to their encoder goals at 1/4 power
                driveDistance(3, .25, .25);


                //sets encoders back to 0
                LeftWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                RightWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


                //stops all drive motors
                LeftWheel.setPower(0);
                LeftWheel2.setPower(0);
                RightWheel.setPower(0);
                RightWheel2.setPower(0);
                sleep(500);
            }
        }


        //if it detects blue rotate clockwise
        else if (colorLeft.red() < colorLeft.blue())
        {


            // sets left motors forward so that positive numbers rotate the robot clockwise

            LeftWheel.setDirection(DcMotorSimple.Direction.FORWARD);
            LeftWheel2.setDirection(DcMotorSimple.Direction.FORWARD);


            //moves both sides to their encoder goals at 1/4 power
            driveDistance(3,.25,.25);


            //sets encoders back to 0

            LeftWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            RightWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


            //puts up color senor
            jewelL.setPosition(.85);

            //stops all drive motors
            LeftWheel.setPower(0);
            LeftWheel2.setPower(0);
            RightWheel.setPower(0);
            RightWheel2.setPower(0);
            sleep(500);


            //set both sides reverse to rotate the robot counterclockwise

            LeftWheel.setDirection(DcMotorSimple.Direction.REVERSE);
            LeftWheel2.setDirection(DcMotorSimple.Direction.REVERSE);
            RightWheel.setDirection(DcMotorSimple.Direction.REVERSE);
            RightWheel2.setDirection(DcMotorSimple.Direction.REVERSE);


            //moves both sides to their encoder goals at 1/4 power
            driveDistance(3,.25,.25);


            //sets encoders back to 0

            LeftWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            RightWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


            //stops all drive motors
            LeftWheel.setPower(0);
            LeftWheel2.setPower(0);
            RightWheel.setPower(0);
            RightWheel2.setPower(0);
            sleep(500);
        }
                    LeftWheel.setDirection(DcMotorSimple.Direction.FORWARD);
                    LeftWheel2.setDirection(DcMotorSimple.Direction.FORWARD);
                    RightWheel.setDirection(DcMotorSimple.Direction.REVERSE);
                    RightWheel2.setDirection(DcMotorSimple.Direction.REVERSE);

    }

    String format(OpenGLMatrix transformationMatrix) {
        return (transformationMatrix != null) ? transformationMatrix.formatAsTransform() : "null";
    }
}