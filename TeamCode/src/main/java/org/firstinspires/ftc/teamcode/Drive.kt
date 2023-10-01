package org.firstinspires.ftc.teamcode

import com.qualcomm.hardware.lynx.LynxModule
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit
import org.firstinspires.ftc.robotcore.external.navigation.VoltageUnit
import kotlin.concurrent.thread
import kotlin.math.abs

@TeleOp(name = "Drive (FGC 2023)", group = "FGC 2023")
class Drive: LinearOpMode() {
    // Control variables
    private val DRIVE_MODE = DriveMode.TANK
    private val COLLECTOR_STALL_THRESHOLD = 1000
    private val COLLECTOR_STALL_RELEASE_TIME = 500

    private val DRIVE_MLT = 1.0
    private val LIFT_MLT = 0.8
    private val COLLECTOR_MLT = 1.0
    private val HOLD_POWER = 1.0
    private val HOOK_MLT_UP = 1.0
    private val HOOK_MLT_DOWN = 1.0
    private val HOOK_RELEASE_TICKS = 1000

    private val DRIVE_TYPE = MotorType.HD_HEX
    private val LIFT_TYPE = MotorType.HD_HEX
    private val COLLECTOR_TYPE = MotorType.HD_HEX
    private val HOOK_TYPE = MotorType.HD_HEX

    private val FILTER_UPWARDS = arrayOf(0.358, 0.560)
    private val FILTER_ALIGNED_WITH_HOOK = arrayOf(0.435, 0.472)
    private val FILTER_DOWNWARDS = arrayOf(0.062, 0.859)

    private val STORAGE_CLOSED = arrayOf(0.145, 1.0)
    private val STORAGE_OPEN = arrayOf(0.6, 0.62)

    // Hardware Devices
    private lateinit var right: DcMotorEx
    private lateinit var left: DcMotorEx
    private lateinit var rightLift: DcMotorEx
    private lateinit var leftLift: DcMotorEx
    private lateinit var collector: DcMotorEx
    private lateinit var hook: DcMotorEx

    private lateinit var filterServoRight: Servo
    private lateinit var filterServoLeft: Servo
    private lateinit var storageServoRight: Servo
    private lateinit var storageServoLeft: Servo
    private lateinit var clawServoLeft: Servo
    private lateinit var clawServoRight: Servo

    // Collector toggle
    private var collectorOn = true
    private var collectorLock1 = false
    private var collectorLock2 = false

    // Motor constants
    private val driveTicksPerSec = DRIVE_TYPE.ticksAtMotor * (DRIVE_TYPE.rpm / 60)
    private val liftTicksPerSec = LIFT_TYPE.ticksAtMotor * (LIFT_TYPE.rpm / 60)
    private val collectorTicksPerSec = COLLECTOR_TYPE.ticksAtMotor * (COLLECTOR_TYPE.rpm / 60)
    private val hookTicksPerSec = HOOK_TYPE.ticksAtMotor * (HOOK_TYPE.rpm / 60)

    // Slide targets
    private var leftLiftTarget = 0
    private var rightLiftTarget = 0

    // Hook target
    private var hookTarget = 0

    // Motor types
    private enum class MotorType(val ticksAtMotor: Double, val rpm: Int) {
        CORE_HEX(288.0, 125),
        HD_HEX(28.0, 6000);
    }

    // Drive types
    private enum class DriveMode {
        JOYSTICK,
        TANK;
    }

    // Servo State Enums
    private enum class FilterState {
        NONE,
        DOWNWARDS,
        UPWARDS,
        ALIGN_WITH_HOOK;
    }

    private enum class StorageState {
        NONE,
        OPEN,
        CLOSED;
    }

    // Servo State Variables
    private var filterState = FilterState.DOWNWARDS
    private var lastFilterState = FilterState.NONE
    private var storageState = StorageState.CLOSED
    private var lastStorageState = StorageState.NONE

    private val clawLeftMin = 0.0
    private val clawRightMin = 0.0
    private val clawLeftMax = 1.0
    private val clawRightMax = 1.0

    private var clawLeftPos = clawLeftMin
    private var clawRightPos = clawRightMin

    override fun runOpMode() {
        // Hardware Initialization
        right = hardwareMap.get(DcMotorEx::class.java, "right")
        left = hardwareMap.get(DcMotorEx::class.java, "left")
        rightLift = hardwareMap.get(DcMotorEx::class.java, "rightLift")
        leftLift = hardwareMap.get(DcMotorEx::class.java, "leftLift")
        collector = hardwareMap.get(DcMotorEx::class.java, "collector")
        hook = hardwareMap.get(DcMotorEx::class.java, "hook")

        filterServoRight = hardwareMap.get(Servo::class.java, "filterRight")
        filterServoLeft = hardwareMap.get(Servo::class.java, "filterLeft")
        storageServoRight = hardwareMap.get(Servo::class.java, "storageRight")
        storageServoLeft = hardwareMap.get(Servo::class.java, "storageLeft")
        clawServoRight = hardwareMap.get(Servo::class.java, "clawRight")
        clawServoLeft = hardwareMap.get(Servo::class.java, "clawLeft")

        clawServoLeft.direction = Servo.Direction.REVERSE

        // Reverse the motors
        collector.direction = DcMotorSimple.Direction.REVERSE
        right.direction = DcMotorSimple.Direction.REVERSE
        leftLift.direction = DcMotorSimple.Direction.REVERSE

        // Brake drivetrain when control released
        right.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        left.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE

        val initTime = System.currentTimeMillis()
        waitForStart()
        val startTime = System.currentTimeMillis()

        // Driver 1 Thread
        var last = System.nanoTime()
        thread(name = "DRIVER1", start = true) {
            val delta = (System.nanoTime() - last) / 1e6
            last = System.nanoTime()

            var collectorStartTime = 0L
            var collectorStallTime = 0L
            while (opModeIsActive()) {
                // Chassis movement
                if(DRIVE_MODE == DriveMode.JOYSTICK) {
                    left.power = -gamepad1.left_stick_y + gamepad1.right_stick_x * DRIVE_MLT
                    right.power = -gamepad1.left_stick_y - gamepad1.right_stick_x * DRIVE_MLT
                } else if(DRIVE_MODE == DriveMode.TANK) {
                    left.power = -gamepad1.left_stick_y * DRIVE_MLT
                    right.power = -gamepad1.right_stick_y * DRIVE_MLT
                }

                // Collector toggle
                if(gamepad2.b && !collectorLock1) collectorOn = !collectorOn
                collectorLock1 = gamepad2.b

                if(gamepad1.b && !collectorLock2) collectorOn = !collectorOn
                collectorLock2 = gamepad1.b

                val collectorPower = COLLECTOR_MLT

                // Collector stall detection
                if(collector.power == 0.0 && collectorOn) collectorStartTime = System.currentTimeMillis()
                collectorStartTime = maxOf(collectorStallTime + COLLECTOR_STALL_RELEASE_TIME, collectorStartTime)

                if(collector.velocity == 0.0 && System.currentTimeMillis() - collectorStartTime > COLLECTOR_STALL_THRESHOLD) collectorStallTime = System.currentTimeMillis()

                collector.power = if(!collectorOn) 0.0
                else if(System.currentTimeMillis() - collectorStallTime < COLLECTOR_STALL_RELEASE_TIME) -collectorPower
                else collectorPower

                // Claw
                val clawMove = (gamepad1.right_trigger - gamepad1.left_trigger) * delta / 50

                clawLeftPos = maxOf(clawLeftMin, minOf(clawLeftMax, clawLeftPos + clawMove))
                clawServoLeft.position = clawLeftPos

                clawRightPos = maxOf(clawRightMin, minOf(clawRightMax, clawRightPos + clawMove))
                clawServoRight.position = clawRightPos
            }
        }

        // Driver 2 Thread
        thread(name = "DRIVER2", start = true) {
            var savedFilterState = FilterState.NONE;
            while (opModeIsActive()) {
                // Lift (slide) movement
                val liftPower = -gamepad2.left_stick_y * LIFT_MLT
                if(gamepad2.left_stick_y != 0.0f) for (motor in listOf(rightLift, leftLift)) {
                    if(motor == leftLift) leftLiftTarget = 0
                    else rightLiftTarget = 0

                    motor.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
                    motor.power = if(liftPower < 0) 0.0 else liftPower
                } else for (motor in listOf(rightLift, leftLift)) {
                    if(motor == leftLift && leftLiftTarget == 0) leftLiftTarget = motor.currentPosition
                    else if(motor == rightLift && rightLiftTarget == 0) rightLiftTarget = motor.currentPosition

                    motor.targetPosition = if(motor == leftLift) leftLiftTarget
                    else rightLiftTarget

                    motor.mode = DcMotor.RunMode.RUN_TO_POSITION
                    motor.power = HOLD_POWER
                }

                // [Lift -> hook] "master -> slave" relationship
                if(liftPower != 0.0 && abs(leftLift.velocity / liftTicksPerSec) > abs(leftLift.power * 0.1)) {
                    hookTarget = 0
                    hook.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER

                    if(liftPower < 0 && savedFilterState == FilterState.NONE) {
                        savedFilterState = filterState
                        filterState = FilterState.ALIGN_WITH_HOOK
                    } else if(liftPower >= 0.0 && savedFilterState != FilterState.NONE) {
                        filterState = savedFilterState
                        savedFilterState = FilterState.NONE
                    }

                    hook.power = if(liftPower < 0) liftPower * HOOK_MLT_DOWN / LIFT_MLT
                    else liftPower * HOOK_MLT_UP / LIFT_MLT
                } else {
                    if(gamepad2.right_stick_y != 0f) {
                        hookTarget = 0
                        hook.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER

                        if(gamepad2.right_stick_y < 0.0 && savedFilterState == FilterState.NONE) {
                            savedFilterState = filterState
                            filterState = FilterState.ALIGN_WITH_HOOK
                        } else if(gamepad2.right_stick_y >= 0.0 && savedFilterState != FilterState.NONE) {
                            filterState = savedFilterState
                            savedFilterState = FilterState.NONE
                        }

                        hook.power = gamepad2.right_stick_y.toDouble()
                    } else {
                        if (hookTarget == 0) hookTarget = hook.currentPosition + HOOK_RELEASE_TICKS
                        hook.targetPosition = hookTarget
                        hook.mode = DcMotor.RunMode.RUN_TO_POSITION
                        hook.power = HOLD_POWER

                        if(savedFilterState != FilterState.NONE) {
                            filterState = savedFilterState
                            savedFilterState = FilterState.NONE
                        }
                    }
                }

                // Servo control
                if(gamepad2.dpad_down) storageState = StorageState.CLOSED
                else if(gamepad2.dpad_up) storageState = StorageState.OPEN

                if(gamepad2.a) filterState = FilterState.DOWNWARDS
                else if(gamepad2.y) filterState = FilterState.UPWARDS

                if(lastFilterState != filterState) {
                    when(filterState) {
                        FilterState.DOWNWARDS -> {
                            filterServoRight.position = FILTER_DOWNWARDS[0]
                            filterServoLeft.position = FILTER_DOWNWARDS[1]
                        }
                        FilterState.UPWARDS -> {
                            filterServoRight.position = FILTER_UPWARDS[0]
                            filterServoLeft.position = FILTER_UPWARDS[1]
                        }
                        FilterState.ALIGN_WITH_HOOK -> {
                            filterServoLeft.position = FILTER_ALIGNED_WITH_HOOK[0]
                            filterServoRight.position = FILTER_ALIGNED_WITH_HOOK[1]
                        }
                        else -> {}
                    }

                    lastFilterState = filterState
                }

                if(lastStorageState != storageState) {
                    when(storageState) {
                        StorageState.OPEN -> {
                            storageServoRight.position = STORAGE_OPEN[0]
                            storageServoLeft.position = STORAGE_OPEN[1]
                        }
                        StorageState.CLOSED -> {
                            storageServoRight.position = STORAGE_CLOSED[0]
                            storageServoLeft.position = STORAGE_CLOSED[1]
                        }
                        else -> {}
                    }

                    lastStorageState = storageState
                }
            }
        }

        // Telemetry in main thread
        while (opModeIsActive()) {
            telemetry.addData("--- DRIVETRAIN ---", "")
            telemetry.addData("Mode", DRIVE_MODE)
            telemetry.addData("Left Power", right.velocity / driveTicksPerSec)
            telemetry.addData("Right Power", left.velocity / driveTicksPerSec)
            telemetry.addData("", "")

            telemetry.addData("--- HOOK ---", "")
            if(leftLift.power < 0) telemetry.addData("Power", hook.velocity / hookTicksPerSec)
            else telemetry.addData("Target", hookTarget)
            telemetry.addData("", "")

            telemetry.addData("--- Collector ---", "")
            telemetry.addData("On", collectorOn)
            telemetry.addData("Lock1", collectorLock1)
            telemetry.addData("Lock2", collectorLock2)
            telemetry.addData("Power", collector.velocity / collectorTicksPerSec)
            telemetry.addData("Stall check", collector.velocity == 0.0)
            telemetry.addData("", "")

            telemetry.addData("--- SLIDES ---", "")
            telemetry.addData("Mode", rightLift.mode)
            if(rightLift.mode == DcMotor.RunMode.RUN_TO_POSITION) {
                telemetry.addData("Left target", rightLift.targetPosition)
                telemetry.addData("Right target", leftLift.targetPosition)
            } else telemetry.addData("Power", rightLift.velocity / liftTicksPerSec)
            telemetry.addData("", "")

            telemetry.addData("--- SERVOS ---", "")
            telemetry.addData("Storage state", storageState)
            telemetry.addData("Right Storage Servo", storageServoRight.position)
            telemetry.addData("Left Storage Servo", storageServoLeft.position)
            telemetry.addData("Filter state", filterState)
            telemetry.addData("Right Filter Servo", filterServoRight.position)
            telemetry.addData("Left Filter Servo", filterServoLeft.position)
            telemetry.addData("", "")

            telemetry.addData("--- HEALTH ---", "")
            telemetry.addData("Runtime since INIT", String.format("%.2fs", (System.currentTimeMillis() - initTime) / 1000f))
            telemetry.addData("Runtime since START", String.format("%.2fs", (System.currentTimeMillis() - startTime) / 1000f))

            val hubs = hardwareMap.getAll(LynxModule::class.java)
            telemetry.addData("Hubs detected", hubs.size)
            for(i in 0..<hubs.size) {
                telemetry.addData("HUB$i voltage", String.format("%.2fV", hubs[i].getInputVoltage(VoltageUnit.VOLTS)))
                telemetry.addData("HUB$i current", String.format("%.2fA", hubs[i].getCurrent(CurrentUnit.AMPS)))
            }

            telemetry.update()
        }
    }
}