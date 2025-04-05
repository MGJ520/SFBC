
// #include <Arduino.h>
#include <SimpleFOC.h>

// Motor instance
BLDCMotor motor = BLDCMotor(7);
BLDCDriver3PWM driver = BLDCDriver3PWM(35, 34, 33);

LowsideCurrentSense cs = LowsideCurrentSense(0.005f, 50.0f, 2, 3, _NC);
MagneticSensorI2C sensor = MagneticSensorI2C(AS5600_I2C);
TwoWire I2C_A = TwoWire(1);

// commander interface
Commander command = Commander(Serial);
void onMotor(char *cmd)
{
    command.motor(&motor, cmd);
}

void setup()
{

    // initialize encoder sensor hardware
    I2C_A.begin(37, 36, 400000UL);
    sensor.init(&I2C_A);

    // link the motor to the sensor
    motor.linkSensor(&sensor);

    driver.voltage_power_supply = 8.4;
    // driver.pwm_frequency = 15000;
    driver.init();
    // link the motor and the driver
    motor.linkDriver(&driver);
    // link current sense and the driver
    cs.linkDriver(&driver);

    // align voltage

    // control loop type and torque mode
    motor.foc_modulation = FOCModulationType::SpaceVectorPWM;
    motor.torque_controller = TorqueControlType::foc_current;
    motor.controller = MotionControlType::velocity;

    // velocity loop PID
    motor.PID_velocity.P = 1;
    motor.PID_velocity.I = 3.0;
    motor.voltage_sensor_align = 0.5;
    // Low pass filtering time constant
    motor.LPF_velocity.Tf = 0.08;
    // angle loop PID
    motor.P_angle.P = 20.0;
    // Low pass filtering time constant
    motor.LPF_angle.Tf = 0.0;
    // current q loop PID
    motor.PID_current_q.P = 0.2;
    motor.PID_current_q.I = 1;
    // Low pass filtering time constant
    motor.LPF_current_q.Tf = 0.08;
    // current d loop PID
    motor.PID_current_d.P = 0.2;
    motor.PID_current_d.I = 1;
    // Low pass filtering time constant
    motor.LPF_current_d.Tf = 0.08;

    motor.PID_current_d.limit = 2;
    motor.PID_current_q.limit = 2;
    // Limits
    motor.velocity_limit = 1000.0; // 100 rad/s velocity limit
    // motor.voltage_limit = 4;       // 12 Volt limit
    motor.current_limit = 4; // 2 Amp current limit

    // use monitoring with serial for motor init
    // monitoring port
    Serial.begin(115200);
    // comment out if not needed
    motor.useMonitoring(Serial);
    // motor.monitor_variables = _MON_CURR_Q | _MON_CURR_D; // monitor the two currents d and q
    // motor.monitor_downsample = 1000;

    // initialise motor
    motor.init();

    cs.init();
    // driver 8302 has inverted gains on all channels
    cs.gain_a *= -1;
    cs.gain_b *= -1;
    cs.gain_c *= -1;
    motor.linkCurrentSense(&cs);

    // align encoder and start FOC
    motor.initFOC();

    // set the inital target value
    motor.target = 0;

    // define the motor id
    command.add('M', onMotor, "motor");

    _delay(1000);
}

void loop()
{
    // iterative setting FOC phase voltage
    motor.loopFOC();

    // iterative function setting the outter loop target
    motor.move();

    // monitoring the state variables
    motor.monitor();

    // user communication
    command.run();
}