package org.firstinspires.ftc.robotcontroller.external.samples;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Hardware;

/**
 * Created by OWNER on 2/21/2018.
 */

public class DonaldHardware {
    public DcMotor rightM = null;
    public DcMotor leftM = null;
    public DcMotor rightC = null;
    public DcMotor leftC = null;
    public DcMotor liftT = null;
    public DcMotor liftB = null;
    public DcMotor rotate = null;
    public DcMotor relicLift = null;
    public Servo rightH = null;
    public ColorSensor rightS = null;

    HardwareMap wallPlan = null;

    public DonaldHardware(){

    }
    public void init (HardwareMap jetMap) {
        //Save stuff to a hardware Map
        wallPlan = jetMap;
        //Define Stuff
        leftM = wallPlan.get(DcMotor.class, "leftM");
        rightM = wallPlan.get(DcMotor.class, "rightM");
        rightC = wallPlan.get(DcMotor.class, "rightC");
        leftC = wallPlan.get(DcMotor.class, "leftC");
        liftT = wallPlan.get(DcMotor.class, "liftT");
        liftB = wallPlan.get(DcMotor.class, "liftB");
        rotate = wallPlan.get(DcMotor.class, "rotate");
        relicLift = wallPlan.get(DcMotor.class, "relicLift");
        rightH = wallPlan.get(Servo.class, "rightH");
        rightS = wallPlan.get(ColorSensor.class, "rightS");

        leftM.setPower(0);
        rightM.setPower(0);
        rightC.setPower(0);
        leftC.setPower(0);
        liftT.setPower(0);
        liftB.setPower(0);
        rotate.setPower(0);
        relicLift.setPower(0);
        leftM.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightM.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightC.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftC.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        liftT.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        liftB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rotate.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        relicLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        rightH.setPosition(.6);
        rightS.enableLed(true);

    }
}
