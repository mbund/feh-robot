/// @file main.cpp
/// @author Mark Bundschuh and Eric Zhang
/// @brief Contains the entrypoint for the program

#include <FEHBattery.h>
#include <FEHIO.h>
#include <FEHLCD.h>
#include <FEHMotor.h>
#include <FEHRPS.h>
#include <FEHServo.h>
#include <FEHUtility.h>
#include <LCDColors.h>

// allow us to access private members of FEHSD and initialize it
// ourselves, so that we can clear the screen after it forcefully
// writes to it. This is really janky and I wish that this did not
// have to happen.
#define private public
#include <FEHSD.h>
#undef private

#include <cmath>
#include <cstdint>
#include <memory>
#include <sstream>
#include <unordered_map>
#include <vector>

/// The distance between the center of the robot and the center of one
/// of the wheels, in inches
constexpr auto ROBOT_CENTER_TO_WHEEL_DISTANCE = 4;

/// The mathematical constant tau
constexpr auto TAU = 6.28318530717959;

/// The number of encoder counts per revolution of an IGWAN motor
constexpr auto IGWAN_COUNTS_PER_REVOLUTION = 318;

/// The radius of one of the wheels, in inches
constexpr auto WHEEL_RADIUS = 1.205;

/// The circumference of one of the wheels, in inches
constexpr auto WHEEL_CIRCUMFERENCE = TAU * WHEEL_RADIUS;

/// The number of encoder counts per inch of travel
constexpr auto IGWAN_COUNTS_PER_INCH =
    IGWAN_COUNTS_PER_REVOLUTION / WHEEL_CIRCUMFERENCE;

/// The supposed width of the LCD screen, in pixels
constexpr auto TRUE_LCD_WIDTH = 320;

/// The supposed height of the LCD screen, in pixels
constexpr auto TRUE_LCD_HEIGHT = 240;

/// The actual width of the LCD screen, in pixels
constexpr auto LCD_WIDTH = TRUE_LCD_WIDTH - 1;

/// The actual height of the LCD screen, in pixels
constexpr auto LCD_HEIGHT = TRUE_LCD_HEIGHT - 2;

/// Converts degrees to radians
/// @param deg The degrees to convert
/// @return The radian value of the degrees
double deg_to_rad(double deg) { return deg * TAU / 360.0; }

/// Converts radians to degrees
/// @param rad The radians to convert
/// @return The degree value of the radians
double rad_to_deg(double rad) { return rad * 360.0 / TAU; }

/// Helper function to clamp a value between a lower and upper bound
/// @param n The value to clamp
/// @param lower The lower bound
/// @param upper The upper bound
template <typename T>
T clamp(const T& n, const T& lower, const T& upper) {
    return std::max(lower, std::min(n, upper));
}

enum LOG_LEVEL {
    LOG_LEVEL_NONE,
    LOG_LEVEL_DEBUG,
    LOG_LEVEL_INFO,
    LOG_LEVEL_ERROR,
    LOG_LEVEL_MAX,
};

enum LOG_LEVEL global_log_level = LOG_LEVEL_INFO;

#define LOG_SCOPE(level)                                    \
    for (enum LOG_LEVEL                                     \
             _prev_log_level_##__LINE__ = global_log_level, \
             global_log_level = level;                      \
         false;                                             \
         global_log_level = _prev_log_level_##__LINE__)

/// Log information to the screen and to a file with a built in string
/// stream
#define LOG_MESSAGE(level, message)                             \
    do {                                                        \
        if (level < global_log_level)                           \
            break;                                              \
        std::stringstream ss;                                   \
        ss.precision(4);                                        \
        ss << message;                                          \
        logger->info(                                           \
            ss.str(), __FILE__, __PRETTY_FUNCTION__, __LINE__); \
    } while (0)

#define LOG_DEBUG(message) \
    LOG_MESSAGE(LOG_LEVEL_DEBUG, "DBG: " << message)
#define LOG_INFO(message) \
    LOG_MESSAGE(LOG_LEVEL_INFO, "INF: " << message)
#define LOG_ERROR(message) \
    LOG_MESSAGE(LOG_LEVEL_ERROR, "ERR: " << message)

/// Log information to the screen and to a file
class Log {
   public:
    /// Default constructor for the Log class
    Log();

    /// Delete the copy constructor
    /// @param obj The object to copy
    Log(const Log& obj) = delete;

    /// Log an info message. Generally use the LOG_INFO macro instead
    /// of this.
    /// @param message The message to log
    /// @param file The file the message is being logged from,
    /// typically using the __FILE__ macro
    /// @param pretty_function The function the message is being
    /// logged from, typically using the __PRETTY_FUNCTION__ macro
    /// @param line The line the message is being logged from,
    /// typically using the  __LINE__ macro
    void info(std::string message,
              const char* file,
              const char* pretty_function,
              int line);

    /// Write all logs to the SD card
    void write();

    /// The short messages to display on the screen
    std::vector<std::string> short_messages;

    /// The long messages to write to the log file
    std::vector<std::string> long_messages;
};

/// Global logger
auto logger = std::make_shared<Log>();

Log::Log() {}

void Log::info(std::string message,
               const char* file,
               const char* pretty_function,
               int line) {
    std::stringstream short_message;
    short_message << line << "|" << message;
    short_messages.push_back(short_message.str());

    std::stringstream long_message;
    long_message << "[" << TimeNow() << "|" << file << "|"
                 << pretty_function << "|" << line << "] " << message;
    long_messages.push_back(long_message.str());
}

void Log::write() {
    LOG_INFO("writing log to file");
    auto file = SD.FOpen("log.txt", "a");
    for (const auto& message : long_messages)
        SD.FPrintf(file, "%s\n", message.c_str());
    SD.FClose(file);
    long_messages.clear();
}

/// Wrapper for the FEHMotor class
class Motor {
   public:
    /// Constructor for the Motor class
    /// @param motor_port The port the motor is plugged into
    /// @param encoder_pin The pin the encoder is plugged into
    Motor(FEHMotor::FEHMotorPort motor_port,
          FEHIO::FEHIOPin encoder_pin);

    /// Sets the power of the motor
    /// @param power The power to set the motor to between -1 and 1
    void drive(double power);

    /// Gets the distance the motor has traveled in inches
    /// @return The distance the motor has traveled in inches
    double get_distance();

    /// Resets the encoder and sets motor power to 0
    void flush();

    /// The correction factor for the motor
    double correction_factor = 1;

   private:
    /// The underlying motor
    FEHMotor motor;

    /// The shaft encoder for the motor
    DigitalEncoder encoder;

    /// The last power set for the motor
    double power = 0;
};

Motor::Motor(FEHMotor::FEHMotorPort port, FEHIO::FEHIOPin encoder_pin)
    : motor(port, 9.0), encoder(encoder_pin) {
    flush();
}

void Motor::drive(double power) {
    if (power * correction_factor < -1.0) {
        LOG_ERROR("power oob: " << power);
    } else if (power * correction_factor > 1.0) {
        LOG_ERROR("power oob: " << power);
    }

    this->power = clamp(power * correction_factor, -1.0, 1.0);
    motor.SetPercent(this->power * 100.0);
}

double Motor::get_distance() {
    return encoder.Counts() / IGWAN_COUNTS_PER_INCH;
}

void Motor::flush() {
    motor.SetPercent(0);
    encoder.ResetCounts();

    power = 0;
}

/// Wrapper for the FEHServo class
class Servo {
   public:
    /// Constructor for the Servo class
    /// @param servo_port The port the servo is plugged into
    /// @param min The minimum value the servo can be set to (from
    /// calibration)
    /// @param max The maximum value the servo can be set to (from
    /// calibration)
    Servo(FEHServo::FEHServoPort servo_port, double min, double max);

    /// Sets the angle of the servo
    /// @param theta The angle to set the servo to in radians
    void set_angle(double theta);

   private:
    /// The underlying servo
    FEHServo servo;
};

Servo::Servo(FEHServo::FEHServoPort servo_port,
             double min,
             double max)
    : servo(servo_port) {
    servo.SetMin(min);
    servo.SetMax(max);
}

void Servo::set_angle(double theta) {
    servo.SetDegree(rad_to_deg(theta));
}

/// Global first motor
Motor m1(FEHMotor::Motor0, FEHIO::P3_5);

/// Global second motor
Motor m2(FEHMotor::Motor1, FEHIO::P3_3);

/// Global third motor
Motor m3(FEHMotor::Motor2, FEHIO::P3_1);

/// Global cds cell for light detection
AnalogInputPin cds(FEHIO::P3_7);

/// Global servo which controls the arm
Servo s1(FEHServo::Servo0, 500, 2500);

/// Global luggage servo
Servo s2(FEHServo::Servo7, 500, 2500);

/// Global UI state type to determine what to draw to the screen
enum UI_MENU {
    UI_MENU_DEBUG,
    UI_MENU_BLANK,
    UI_MENU_LOGS,
    UI_MENU_MAX,
};

inline UI_MENU operator++(UI_MENU& self, int) {
    const UI_MENU prev = self;
    const int i = static_cast<int>(self);
    self = static_cast<UI_MENU>((i + 1) % UI_MENU_MAX);
    return prev;
}

/// Global UI state variable to determine what to draw to the screen
UI_MENU ui_select = UI_MENU_DEBUG;

/// Global variable for the current background color
unsigned int current_background_color = BLACK;

/// Global variable for the current font color
unsigned int current_font_color = WHITE;

/// Sets the font color of the screen, with optimizations
/// @param color The color to set the font to
void set_font_color(unsigned int color) {
    if (current_background_color != color)
        LCD.SetFontColor(color);
}

/// Sets the background color of the screen, with optimizations
/// @param color The color to set the background to
void set_background_color(unsigned int color) {
    if (current_font_color != color)
        LCD.SetBackgroundColor(color);
}

double last_touch_switch_time = 0;
double last_write_time = 0;
size_t scroll_index = 0;
size_t num_logs = 0;

/// Library function to draw UI to the screen while the robot is
/// idling
bool idle() {
    constexpr auto FONT_WIDTH = 12;
    constexpr auto FONT_HEIGHT = 17;

    float touch_x;
    float touch_y;
    bool touch_pressed = LCD.Touch(&touch_x, &touch_y);

    set_background_color(BLACK);
    set_font_color(WHITE);

    bool rerender_logs = false;

    if (TimeNow() - last_touch_switch_time > 0.5 && touch_pressed &&
        touch_x > LCD_WIDTH - 50 && touch_y > LCD_HEIGHT - 50) {
        ui_select++;
        if (ui_select == UI_MENU_LOGS)
            rerender_logs = true;

        if (ui_select == UI_MENU_BLANK) {
            LCD.DrawCircle(LCD_WIDTH / 2, LCD_HEIGHT / 2, 20);
        }

        last_touch_switch_time = TimeNow();

        LCD.Clear();
    }

    if (ui_select == UI_MENU_LOGS) {
        constexpr size_t MAX_LINES = LCD_HEIGHT / FONT_HEIGHT;

        size_t prev_scroll_index = scroll_index;
        if (logger->short_messages.size() > MAX_LINES)
            scroll_index = logger->short_messages.size() - MAX_LINES;

        rerender_logs |= prev_scroll_index != scroll_index;
        rerender_logs |= num_logs != logger->short_messages.size();

        num_logs = logger->short_messages.size();

        for (size_t log_index = scroll_index, ui_index = 0;
             rerender_logs && log_index < scroll_index + MAX_LINES &&
             log_index < logger->short_messages.size();
             log_index++, ui_index++) {
            constexpr auto CHARS = 26;
            auto message =
                logger->short_messages[log_index].substr(0, CHARS);
            message += std::string(CHARS - message.size(), ' ');
            LCD.WriteAt(message.c_str(), 0, ui_index * FONT_HEIGHT);
        }
    }

    if (ui_select == UI_MENU_DEBUG) {
        std::stringstream battery_stream;
        battery_stream << "Battery (0-11.7V): " << Battery.Voltage();
        LCD.WriteAt(battery_stream.str().c_str(), 0, FONT_HEIGHT * 0);

        std::stringstream cds_stream;
        cds_stream << "Cds (0-3.3V): " << cds.Value();
        LCD.WriteAt(cds_stream.str().c_str(), 0, FONT_HEIGHT * 1);

        std::stringstream m1_distance_stream;
        m1_distance_stream << "M1 dist (in): " << m1.get_distance();
        LCD.WriteAt(
            m1_distance_stream.str().c_str(), 0, FONT_HEIGHT * 2);

        std::stringstream m2_distance_stream;
        m2_distance_stream << "M2 dist (in): " << m2.get_distance();
        LCD.WriteAt(
            m2_distance_stream.str().c_str(), 0, FONT_HEIGHT * 3);

        std::stringstream m3_distance_stream;
        m3_distance_stream << "M3 dist (in): " << m3.get_distance();
        LCD.WriteAt(
            m3_distance_stream.str().c_str(), 0, FONT_HEIGHT * 4);

        std::stringstream correct_lever_stream;
        correct_lever_stream << "Lever (0,1,2): "
                             << RPS.GetCorrectLever();
        LCD.WriteAt(
            correct_lever_stream.str().c_str(), 0, FONT_HEIGHT * 5);

        char current_region_letter = RPS.CurrentRegionLetter();
        if (current_region_letter < 'A' ||
            current_region_letter > 'L')
            current_region_letter = '?';
        std::stringstream current_region_stream;
        current_region_stream << "Region (A,B,C,D): "
                              << current_region_letter;
        LCD.WriteAt(
            current_region_stream.str().c_str(), 0, FONT_HEIGHT * 6);

        std::stringstream rps_x_stream;
        rps_x_stream << "RPS X: " << RPS.X();
        LCD.WriteAt(rps_x_stream.str().c_str(), 0, FONT_HEIGHT * 7);

        std::stringstream rps_y_stream;
        rps_y_stream << "RPS Y: " << RPS.Y();
        LCD.WriteAt(rps_y_stream.str().c_str(), 0, FONT_HEIGHT * 8);

        LCD.WriteAt("W", 5, LCD_HEIGHT - FONT_HEIGHT - 5);
        if (TimeNow() - last_write_time > 0.5 && touch_pressed &&
            touch_x < 50 && touch_y > LCD_HEIGHT - 50) {
            last_write_time = TimeNow();
            set_background_color(WHITE);
            set_font_color(BLACK);
            logger->write();
        }

        LCD.WriteAt("R", 5 + 50, LCD_HEIGHT - FONT_HEIGHT - 5);
        if (touch_x > 50 && touch_x < 50 + 50 &&
            touch_y > LCD_HEIGHT - 50) {
            set_background_color(WHITE);
            set_font_color(BLACK);
            RPS.InitializeTouchMenu();
            idle();
        }
    }

    return false;
}

#define IDLE(x)       \
    do {              \
        if (idle())   \
            return x; \
    } while (0)

/// Translates the robot a given distance in a given heading at a
/// given power
/// @param distance The distance to translate in inches
/// @param heading The heading to translate in radians
/// @param power The power to translate at
void translate(double distance, double heading, double power) {
    // if (power < 0.0 || power > 1.0) {
    //     LOG_ERROR("power must be between 0 and 1");
    //     return;
    // }

    LOG_DEBUG("mov " << distance << "in " << heading << "rad "
                     << (power * 100.) << "%");

    auto x = std::cos(heading);
    auto y = std::sin(heading);
    auto m1_ratio = (2.0 / 3.0) * x;
    auto m2_ratio = -(1.0 / 3.0) * x - (1.0 / std::sqrt(3.0)) * y;
    auto m3_ratio = -(1.0 / 3.0) * x + (1.0 / std::sqrt(3.0)) * y;

    m1.flush();
    m2.flush();
    m3.flush();

    m1.drive(power * m1_ratio);
    m2.drive(power * m2_ratio);
    m3.drive(power * m3_ratio);

    while (std::abs(m1.get_distance() * m1_ratio) +
               std::abs(m2.get_distance() * m2_ratio) +
               std::abs(m3.get_distance() * m3_ratio) <
           distance)
        IDLE();

    m1.flush();
    m2.flush();
    m3.flush();
}

/// Translates the robot for a given duration.
/// @param duration The duration to translate for.
/// @param heading The heading to translate in.
/// @param power The power to translate at.
void translate_time(double duration, double heading, double power) {
    LOG_DEBUG("mov " << duration << "s " << heading << "rad "
                     << (power * 100.) << "%");

    auto x = std::cos(heading);
    auto y = std::sin(heading);
    auto m1_ratio = (2.0 / 3.0) * x;
    auto m2_ratio = -(1.0 / 3.0) * x - (1.0 / std::sqrt(3.0)) * y;
    auto m3_ratio = -(1.0 / 3.0) * x + (1.0 / std::sqrt(3.0)) * y;

    m1.flush();
    m2.flush();
    m3.flush();

    m1.drive(power * m1_ratio);
    m2.drive(power * m2_ratio);
    m3.drive(power * m3_ratio);

    double t_start = TimeNow();
    while (TimeNow() < t_start + duration)
        IDLE();

    m1.flush();
    m2.flush();
    m3.flush();
}

/// Wait for the CDS to be below a certain value
void cds_wait() {
    LOG_INFO("waiting for cds");

    constexpr auto RED_VALUE = 0.3;
    constexpr auto TIMEOUT_SEC = 30;

    auto timeout = TimeNow();
    auto val = cds.Value();
    while (val >= RED_VALUE) {
        val = cds.Value();
        IDLE();
        if (TimeNow() >= timeout + TIMEOUT_SEC) {
            LOG_ERROR("timeout cds");
            break;
        }
    }

    LOG_INFO("cds end " << val << " < " << RED_VALUE);
}

/// Wait for a touch event
void touch_wait() {
    LOG_INFO("waiting for top touch");

    float touch_x;
    float touch_y;
    while (true) {
        bool touch_pressed = LCD.Touch(&touch_x, &touch_y);
        if (touch_pressed && touch_y < 100)
            break;

        IDLE();
    }

    set_font_color(GREEN);
    LCD.DrawCircle(LCD_WIDTH / 2.0, LCD_HEIGHT / 2.0, 5);

    LOG_INFO("got touch");
}

/// Rotate the robot by a given angle in radians at a given power
/// @param theta Angle in radians
/// @param power Power in the range [0, 1]
void rotate(double theta, double power) {
    LOG_DEBUG("rot " << rad_to_deg(theta) << "rad " << (power * 100.)
                     << "%");

    const auto distance =
        std::abs(ROBOT_CENTER_TO_WHEEL_DISTANCE * theta);

    m1.flush();
    m2.flush();
    m3.flush();

    m1.drive(power);
    m2.drive(power);
    m3.drive(power);

    while (m1.get_distance() + m2.get_distance() + m3.get_distance() <
           distance * 3 * (std::sqrt(3.0) / 2.0))
        IDLE();

    m1.flush();
    m2.flush();
    m3.flush();
}

/// Sleep for a duration
/// @param duration Duration in seconds
void sleep(double duration) {
    LOG_DEBUG("sleep " << duration << "s");

    double t_start = TimeNow();
    while (TimeNow() < t_start + duration)
        IDLE();
}

/// Ticket kiosk task
void ticket_kiosk() {
    constexpr auto RED_VALUE = 0.4;
    constexpr auto BLUE_VALUE = 1.2;
    constexpr auto TIMEOUT_SEC = 5;

    auto timeout = TimeNow();
    while (true) {
        const auto val = cds.Value();

        if (val < RED_VALUE) {
            LOG_INFO("red " << val);

            translate(11, deg_to_rad(180), 0.60);

            translate_time(0.5, deg_to_rad(270), 0.70);
            translate(1.5, deg_to_rad(90), 0.30);

            break;
        } else if ((val > RED_VALUE && val < BLUE_VALUE) ||
                   (TimeNow() >= timeout + TIMEOUT_SEC)) {
            LOG_INFO("blue " << val);

            translate(5, deg_to_rad(180), 0.60);

            translate_time(0.5, deg_to_rad(270), 0.70);
            translate(1.5, deg_to_rad(90), 0.30);

            translate(5, deg_to_rad(180), 0.60);

            break;
        }

        IDLE();
    }
}

/// Fuel lever task
void fuel_lever() {
    constexpr auto LEVER_A = 0;
    constexpr auto LEVER_A1 = 1;
    constexpr auto LEVER_B = 2;

    while (true) {
        const auto lever = RPS.GetCorrectLever();
        LOG_INFO("got rps lever " << lever);

        if (lever == LEVER_A) {
            translate(0.5, deg_to_rad(180), 0.90);
            rotate(deg_to_rad(170), 0.50);
            translate(1, deg_to_rad(90), 0.90);
            s1.set_angle(deg_to_rad(180));
            sleep(0.5);
            translate(1.75, deg_to_rad(270), 0.90);
            sleep(1);
            translate(1.75, deg_to_rad(90), 0.90);
            sleep(5);  // 5 second wait for bonus
            s1.set_angle(deg_to_rad(90));
            sleep(0.5);
            translate(1.75, deg_to_rad(270), 0.90);

            // go to the same ending position as the other levers
            rotate(deg_to_rad(170), -0.50);
            translate(2.8, deg_to_rad(180), 0.90);
            sleep(0.1);

            break;
        }

        if (lever == LEVER_A1) {
            translate(3.4, deg_to_rad(177), 0.90);
            rotate(deg_to_rad(190), 0.50);
            // translate(1, deg_to_rad(90), 0.90);
            s1.set_angle(deg_to_rad(180));
            sleep(0.5);
            translate(1.75, deg_to_rad(270), 0.90);
            sleep(1);
            translate(1.75, deg_to_rad(90), 0.90);
            sleep(5);  // 5 second wait for bonus
            s1.set_angle(deg_to_rad(90));
            sleep(0.5);
            translate(1.75, deg_to_rad(270), 0.90);

            // go to the same ending position as the other levers
            rotate(deg_to_rad(180), -0.50);

            break;
        }

        if (lever == LEVER_B) {
            translate(3.5, deg_to_rad(177), 0.90);
            rotate(deg_to_rad(215), 0.50);
            s1.set_angle(deg_to_rad(180));
            sleep(0.5);
            translate(1.75, deg_to_rad(270), 0.90);
            sleep(1);
            translate(1.75, deg_to_rad(90), 0.90);
            sleep(5);  // 5 second wait for bonus
            s1.set_angle(deg_to_rad(90));
            sleep(0.5);
            translate(2, deg_to_rad(270), 0.90);
            rotate(deg_to_rad(215), -0.50);

            break;
        }

        IDLE();
    }
}

/// Main function which is the entrypoint for the entire program
int main() {
    s1.set_angle(deg_to_rad(90));
    s2.set_angle(deg_to_rad(90));
    SD.Initialize();

    LCD.Clear(BLACK);
    LCD.SetFontColor(WHITE);

    LOG_INFO("bootstrapped");

    while (true) {
        LOG_INFO("starting");
        touch_wait();

        cds_wait();

        // --------- fuel lever ---------
        // navigate from starting point to fuel lever
        translate(8.25, deg_to_rad(90), 0.80);
        sleep(0.1);
        translate(16.5, deg_to_rad(180), 1.00);

        fuel_lever();

        // go up the ramp and square up against the left wall
        translate(22, deg_to_rad(90), 1.20);
        rotate(deg_to_rad(65), 0.60);
        translate_time(1.5, deg_to_rad(270), 0.70);
        translate(10, deg_to_rad(90), 0.70);
        rotate(deg_to_rad(90), -0.60);

        // --------- luggage ---------
        // navigate from fuel lever to luggage

        // square up against luggage wall
        translate_time(0.75, deg_to_rad(270), 0.70);

        // rotate to face luggage
        translate(1, deg_to_rad(90), 0.60);
        rotate(deg_to_rad(50), 0.60);
        translate_time(0.25, deg_to_rad(-60), 0.30);

        // drop luggage
        s2.set_angle(deg_to_rad(0));
        sleep(0.25);
        s2.set_angle(deg_to_rad(90));

        // // square up against luggage wall again
        translate(2, deg_to_rad(120), 0.60);
        rotate(deg_to_rad(60), -0.3);
        // translate_time(1.0, deg_to_rad(270), 0.70);

        // go and square up against the top-left angled wall
        translate(9.0, deg_to_rad(90), 1.60);
        sleep(0.1);
        rotate(deg_to_rad(130), 0.60);
        translate(5, deg_to_rad(270), 0.60);
        translate_time(0.5, deg_to_rad(270), 0.70);

        // stuff robot into corner
        translate_time(1, deg_to_rad(180), 0.60);
        translate(2.7, deg_to_rad(0), 0.6);

        translate(2.8, deg_to_rad(90), 0.60);
        rotate(deg_to_rad(45), 0.30);

        // --------- ticket kiosk ---------
        // bring arm down below the handle in preperation for the
        // passport stamp
        s1.set_angle(deg_to_rad(180));

        ticket_kiosk();

        // --------- passport stamp ---------

        // bring the axis of the motor in line with the axis of the
        // handle
        translate_time(0.5, deg_to_rad(180), 0.60);
        rotate(deg_to_rad(15), -0.60);
        translate(1.0, deg_to_rad(0), 0.50);

        // bring the arm (and lever) up
        s1.set_angle(deg_to_rad(20));
        sleep(0.25);
        s1.set_angle(deg_to_rad(90));
        translate(2, deg_to_rad(90), 0.70);

        // --------- final button ---------
        // move away from passport stamp
        translate(2, deg_to_rad(0), 0.60);

        // navigate down the ramp and hit the button
        translate(12, deg_to_rad(90), 1.40);
        translate(8.5, deg_to_rad(180), 1.40);
        translate(20, deg_to_rad(90), 1.40);
        translate(8, deg_to_rad(115), 1.40);

        // now the robot is down the ramp
        translate_time(3, deg_to_rad(90), 0.60);
        translate_time(5, deg_to_rad(180), 0.60);
    }
}