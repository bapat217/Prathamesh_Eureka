package org.firstinspires.ftc.teamcode.Opmode.Teleop;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
public class FollowerMotorClass {
    public static DcMotorEx m1, m2;
    public static int pos, offset;
    public static double pow;
    public FollowerMotorClass(DcMotorEx m1, DcMotorEx m2) {
        this.m1 = m1;
        this.m2 = m2;
    }

    public void followMotor(int pos, int tolerance, double pow, int offset) {
        this.pos = pos;
        this.offset = offset;
        this.pow = pow;
        m1.setTargetPosition(pos);
        m1.setTargetPositionTolerance(tolerance);
        m1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        m1.setPower(pow);


    }

    public  void updateElevator(){
        if(Math.abs(m1.getCurrentPosition()- pos) <offset){
            m2.setPower(0);
        }
        else{
            if (m1.getCurrentPosition() < m1.getTargetPosition()) {
                m2.setPower(pow);
            }
            else{
                m2.setPower(-pow);
            }
        }
    }

}
