#pragma once

/// @file util.h
/// @author Mark Bundschuh
/// @brief Utility functions and classes

#include <FEHIO.h>
#include <FEHMotor.h>

#include <functional>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

/// The distance between the center of the robot and the center of one of the
/// wheels, in inches
const auto ROBOT_CENTER_TO_WHEEL_DISTANCE = 4;

/// The mathematical constant pi
constexpr auto PI = 3.141592653589;

/// The mathematical constant tau
constexpr auto TAU = PI * 2;

/// The number of encoder counts per revolution of an IGWAN motor
constexpr auto IGWAN_COUNTS_PER_REVOLUTION = 318;

/// The radius of one of the wheels, in inches
constexpr auto WHEEL_RADIUS = 1.205;

/// The circumference of one of the wheels, in inches
constexpr auto WHEEL_CIRCUMFERENCE = TAU * WHEEL_RADIUS;

/// The number of encoder counts per inch of travel
constexpr auto IGWAN_COUNTS_PER_INCH =
    IGWAN_COUNTS_PER_REVOLUTION / WHEEL_CIRCUMFERENCE;

/// Y offset in pixels for the reported touch location to the actual touch
/// location. Our Proteus innacurately reports touch location, so we must
/// account for it.
const auto TOUCH_OFFSET_Y = -4;

/// The supposed width of the LCD screen, in pixels
const auto TRUE_LCD_WIDTH = 320;

/// The supposed height of the LCD screen, in pixels
const auto TRUE_LCD_HEIGHT = 240;

/// The actual width of the LCD screen, in pixels
const auto LCD_WIDTH = TRUE_LCD_WIDTH - 1;

/// The actual height of the LCD screen, in pixels
const auto LCD_HEIGHT = TRUE_LCD_HEIGHT - 2;

/// The width of a standard character in the font, in pixels
const auto FONT_WIDTH = 12;

/// The height of a standard character in the font, in pixels
const auto FONT_HEIGHT = 17;

/// Global position of the x coordinate of the touch location
inline float touch_x;

/// Global position of the y coordinate of the touch location
inline float touch_y;

/// Global variable for whether the screen is currently being touched
inline bool touch_pressed;

/// Log information to the screen and to a file with a built in string stream
#define LOG_INFO(message)                                                \
    do {                                                                 \
        std::stringstream ss;                                            \
        ss.precision(4);                                                 \
        ss << message;                                                   \
        logger->info(ss.str(), __FILE__, __PRETTY_FUNCTION__, __LINE__); \
    } while (0)

/// Log information to the screen and to a file
class Log {
   public:
    /// Default constructor for the Log class
    Log();

    /// Delete the copy constructor
    /// @param obj The object to copy
    Log(const Log& obj) = delete;

    /// Log an info message. Generally use the LOG_INFO macro instead of this.
    /// @param message The message to log
    /// @param file The file the message is being logged from, typically using
    /// the __FILE__ macro
    /// @param pretty_function The function the message is being logged from,
    /// typically using the __PRETTY_FUNCTION__ macro
    /// @param line The line the message is being logged from, typically using
    /// the  __LINE__ macro
    void info(std::string message,
              const char* file,
              const char* pretty_function,
              int line);

    /// Write all logs to the SD card
    void write();

    /// Allow the LogUI class to access the private members of the Log class
    friend class LogUI;

   private:
    /// The short messages to display on the screen
    std::vector<std::string> short_messages;

    /// The long messages to write to the log file
    std::vector<std::string> long_messages;
};

/// Wrapper for the FEHMotor class
class Motor {
   public:
    /// Constructor for the Motor class
    /// @param motor_port The port the motor is plugged into
    /// @param encoder_pin The pin the encoder is plugged into
    Motor(FEHMotor::FEHMotorPort motor_port, FEHIO::FEHIOPin encoder_pin);

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

/// Base class for all steps
class Step {
   public:
    /// Constructor for the Step class
    /// @param name The name of the step (for debugging)
    Step(std::string name);

    /// Executes the step
    /// @param t The current time
    /// @return Whether the step is done
    virtual bool execute(double t);

    /// Function which is called when the step starts
    virtual void start();

    /// The name of the step
    std::string name;

    /// The time at which the step started
    double t_start;

    /// The time at which the step ended
    double t_end;

    /// Whether the step is ephemeral, meaning it will be deleted after it is
    /// done, or not
    bool ephemeral = false;
};

/// Global logger
inline auto logger = std::make_shared<Log>();

/// Helper template function for making a vector of shared pointer
template <typename T, typename... Ts>
std::shared_ptr<T> make_shared(Ts&&... ts) {
    return std::shared_ptr<T>(new T{std::forward<Ts>(ts)...});
}

/// Helper template function to make a vector of shared pointers, used by the
/// Timeline and UnionStep classes.
template <typename Base, typename... Ts>
std::vector<std::shared_ptr<Base>> make_vector_of_shared(Ts&&... ts) {
    std::shared_ptr<Base> init[] = {make_shared<Ts>(std::forward<Ts>(ts))...};
    return std::vector<std::shared_ptr<Base>>{
        std::make_move_iterator(std::begin(init)),
        std::make_move_iterator(std::end(init))};
}

/// Execute a set of steps in parallel
class UnionStep : public Step {
   public:
    /// Constructor for a union step
    /// @param name The name of the step
    /// @param ts The steps to execute in parallel
    template <typename... Ts>
    UnionStep(std::string name, Ts&&... ts);

    /// Execute the union step
    bool execute(double t) override;

   private:
    /// The steps to execute in parallel
    std::vector<std::shared_ptr<Step>> steps;
};

template <typename... Ts>
UnionStep::UnionStep(std::string name, Ts&&... ts)
    : Step(name), steps(make_vector_of_shared<Step>(std::forward<Ts>(ts)...)) {}

/// Represents a rectangle
struct Rect {
    /// Default constructor for a rectangle
    Rect() : x(0), y(0), width(0), height(0) {}

    /// Constructor for a rectangle
    /// @param x The x coordinate of the top left corner
    /// @param y The y coordinate of the top left corner
    /// @param width The width of the rectangle
    /// @param height The height of the rectangle
    Rect(unsigned int x,
         unsigned int y,
         unsigned int width,
         unsigned int height)
        : x(x), y(y), width(width), height(height) {}

    /// The x coordinate of the top left corner
    unsigned int x;

    /// The y coordinate of the top left corner
    unsigned int y;

    /// The width of the rectangle
    unsigned int width;

    /// The height of the rectangle
    unsigned int height;
};

/// A window on the screen which can be accessed by the navbar
class UIWindow {
   public:
    /// Constructor for the UIWindow class
    UIWindow(Rect bounds);

    /// Initial bulk render of the window (full re-render)
    virtual void render() = 0;

    /// Inject updates to the UI
    virtual void update() = 0;

   protected:
    /// The bounds of the window
    Rect bounds;
};

/// Represents a touchable region on the screen
class TouchableRegion {
   public:
    /// Constructor for a touchable region
    /// @param rect The rectangle that represents the region
    /// @param on_button_enter The function to call when the button is pressed
    /// @param on_button_exit The function to call when the button is released
    TouchableRegion(
        Rect rect,
        std::function<void()> on_button_enter = []() {},
        std::function<void()> on_button_exit = []() {});

    /// Checks the touch sensor and updates the state of the button
    void update();

    /// The rectangle that represents the region
    Rect rect;

   private:
    /// Type to represent the state of the button
    typedef enum {
        InBounds,
        OutOfBounds,
    } ButtonState;

    /// The current state of the button
    ButtonState state;

    /// The function to call when the button is pressed
    std::function<void()> on_button_enter;

    /// The function to call when the button is released
    std::function<void()> on_button_exit;
};

/// Represents a header button on the navbar
class NavbarButton {
   public:
    /// Constructor for a navbar button
    /// @param bounding_box The bounding box of the button
    /// @param text The text to display on the button
    /// @param on_button_down The function to call when the button is pressed
    NavbarButton(Rect bounding_box,
                 std::string text,
                 std::function<void()> on_button_down);

    /// Render te button to the screen
    void render(bool is_pressed);

    /// The touchable region for the button
    std::unique_ptr<TouchableRegion> region;

   private:
    /// The bounding box of the button
    Rect bounding_box;

    /// The text to display on the button
    std::string text;

    /// The function to call when the button is pressed
    std::function<void()> on_button_down;
};

/// Represents the navbar UI at the top of the screen
class Navbar {
   public:
    /// Default constructor for the navbar
    Navbar();

    /// Add a button to the navbar
    /// @param text The text to display on the button
    /// @param on_button_down The function to call when the button is pressed
    void add_button(const std::string& text,
                    std::function<void()> on_button_down);

    /// Update the navbar regions
    void update();

    /// The bounding box of the navbar
    Rect bounding_box;

    /// Allow the log UI to access private members of the navbar
    friend class LogUI;

    /// Allow the timeline UI to access private members of the navbar
    friend class TimelineUI;

    /// Allow the misc UI to access private members of the navbar
    friend class MiscUI;

   private:
    /// The padding between the navbar and the screen
    const unsigned int OUTER_PADDING = 4;

    /// The padding between the buttons
    const unsigned int INNER_PADDING = 2;

    /// The buttons to display on the navbar
    std::vector<NavbarButton> buttons;

    /// The x distance of the next button
    unsigned int x;

    /// The index of the currently selected button
    unsigned int current_selected;
};

/// Sequence of steps to execute
class Timeline {
   public:
    /// Constructor for the Timeline
    /// @param ts The steps to execute in order
    template <typename... Ts>
    Timeline(Ts&&... ts);

    /// Execute the timeline
    /// @return True if the timeline is finished executing
    bool timestep(double dt);

    /// Add one or more steps to the timeline directly after the current step
    /// which will each be removed after they are executed
    /// @param ts The steps to execute in order
    template <typename... Ts>
    void add_ephemeral_steps(Ts&&... ts);

    /// Allow the timeline UI to access private members of the timeline
    friend class TimelineUI;

   private:
    /// The steps to execute
    std::vector<std::shared_ptr<Step>> steps;

    /// If the timeline is currently playing
    bool is_playing = false;

    /// The current time in the timeline
    double t = 0;

    /// The index of the current step
    size_t current_step_index = 0;
};

template <typename... Ts>
Timeline::Timeline(Ts&&... ts)
    : steps(make_vector_of_shared<Step>(std::forward<Ts>(ts)...)) {}

template <typename... Ts>
void Timeline::add_ephemeral_steps(Ts&&... ts) {
    auto xs = make_vector_of_shared<Step>(std::forward<Ts>(ts)...);
    for (auto x : xs)
        x->ephemeral = true;
    steps.insert(steps.begin() + current_step_index + 1, xs.begin(), xs.end());
}

inline std::shared_ptr<Timeline> timeline;

/// Play/pause button UI component
class PlayPauseButton {
   public:
    /// Possible states of the button
    enum State {
        Play,
        Pause,
    };

    /// Constructor for the PlayPauseButton
    /// @param bounding_box The bounding box of the button
    /// @param on_button_down The function to call when the button is pressed
    /// @param default_state The default state of the button
    PlayPauseButton(Rect bounding_box,
                    std::function<State(State)> on_button_down,
                    State default_state);

    /// Update the button state
    void update();

    /// Render the button to the screen
    void render();

    /// The bounding box of the button
    Rect bounding_box;

    /// The current state of the button
    State current_state;

   private:
    /// The touchable region of the button
    std::unique_ptr<TouchableRegion> region;
};

/// UI helper class for displaying the timeline
class TimelineUI : public UIWindow {
   public:
    /// Constructor for the TimelineUI class
    /// @param bounds The bounds of the window
    /// @param timeline The timeline to display
    /// @param navbar The navbar to display
    TimelineUI(Rect bounds, Navbar& navbar);

    /// Initial bulk render of the window (full re-render)
    void render() override;

    /// Inject updates to the UI
    void update() override;

   private:
    /// Rerender the timeline after a pagination
    void paginate();

    /// The previous step index of the timeline
    size_t prev_timeline_step_index = 0;

    size_t prev_timeline_size = 0;

    /// The active navbar
    Navbar& navbar;

    /// The UI region of the page up button
    std::unique_ptr<TouchableRegion> region_page_up;

    /// The UI region of the page down button
    std::unique_ptr<TouchableRegion> region_page_down;

    /// The UI region of the play/pause button
    std::unique_ptr<PlayPauseButton> button_pause_play;

    /// The play/pause buttons for each step
    std::vector<std::unique_ptr<PlayPauseButton>> button_play_steps;

    /// The index of the first step to display because of pagination
    size_t scroll_index = 0;

    /// The standard size of a button
    const size_t BUTTON_MEASURE = 2 * FONT_HEIGHT;
};

/// UI helper class for displaying the log
class LogUI : public UIWindow {
   public:
    /// Constructor for the LogUI class
    /// @param bounds The bounds of the window
    /// @param navbar The navbar to display
    LogUI(Rect bounds, Navbar& navbar);

    /// Initial bulk render of the window (full re-render)
    void render() override;

    /// Inject updates to the UI
    void update() override;

   private:
    /// The UI regions which can be touched
    std::vector<TouchableRegion> regions;

    /// The active navbar
    Navbar& navbar;
};

class Calibrator {
   public:
    bool calibrate_motors();

   private:
    double calibration_start_time = 0;
};

/// UI helper class for displaying miscellaneous UI components
class MiscUI : public UIWindow {
   public:
    /// Constructor for the MiscUI class
    /// @param bounds The bounds of the window
    /// @param navbar The navbar to display
    MiscUI(Rect bounds, Navbar& navbar);

    /// Initial bulk render of the window (full re-render)
    void render() override;

    /// Inject updates to the UI
    void update() override;

   private:
    /// The touchable region of the first motor's calibration button
    std::unique_ptr<TouchableRegion> m1_region;

    /// The time the first motor's calibration button was pressed
    double m1_start_time = 0;

    /// The touchable region of the second motor's calibration button
    std::unique_ptr<TouchableRegion> m2_region;

    /// The time the second motor's calibration button was pressed
    double m2_start_time = 0;

    /// The touchable region of the third motor's calibration button
    std::unique_ptr<TouchableRegion> m3_region;

    /// The time the third motor's calibration button was pressed
    double m3_start_time = 0;

    /// Helper function to update the button UI of a motor during calibration
    void update_motor_button_ui(std::unique_ptr<TouchableRegion>& region,
                                double& start_time);

    // Helper function to count the distance a motor has traveled over some
    // period of time
    void count_single_motor(Motor& motor, double& start_time);

    std::unique_ptr<TouchableRegion> calibrator_region;
    Calibrator calibrator;
    bool is_calibrated = true;

    std::unique_ptr<TouchableRegion> log_write_region;

    /// The active navbar
    Navbar& navbar;
};

/// Global first motor
inline Motor m1(FEHMotor::Motor0, FEHIO::P3_5);

/// Global second motor
inline Motor m2(FEHMotor::Motor1, FEHIO::P3_3);

/// Global third motor
inline Motor m3(FEHMotor::Motor2, FEHIO::P3_1);

/// Global cds cell for light detection
inline AnalogInputPin cds(FEHIO::P3_7);
