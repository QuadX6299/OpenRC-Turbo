package org.firstinspires.ftc.teamcode.Robot.Sensors

import com.qualcomm.hardware.bosch.BNO055IMU
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import org.firstinspires.ftc.robotcore.external.navigation.*
import org.firstinspires.ftc.teamcode.lib.Extensions.limitAngle

object IMU {
    private lateinit var imu : BNO055IMU

    fun init(op : OpMode) {
        imu = op.hardwareMap.get(BNO055IMU::class.java, "imu")
        val params = BNO055IMU.Parameters()
        params.angleUnit = BNO055IMU.AngleUnit.RADIANS
        params.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC
        imu.initialize(params)
    }

    @JvmStatic
    fun heading() : Double {
        return imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle.toDouble().limitAngle()
    }

    fun imuOrientation() : Orientation {
        return imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.RADIANS)
    }

    fun getTrueDiff(origAngle: Double): Double {
        val currAngle = heading()
        return if (currAngle >= 0 && origAngle >= 0 || currAngle <= 0 && origAngle <= 0)
            -(currAngle - origAngle)
        else if (Math.abs(currAngle - origAngle) <= 180)
            -(currAngle - origAngle)
        else if (currAngle > origAngle)
            360 - (currAngle - origAngle)
        else
            -(360 + (currAngle - origAngle))
    }


    fun startAccel(pollingMs : Int) {
        imu.startAccelerationIntegration(Position(), Velocity(), pollingMs)
    }
}