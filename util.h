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

const auto ROBOT_CENTER_TO_WHEEL_DISTANCE = 4;  // inches

constexpr auto PI = 3.141592653589;
constexpr auto TAU = PI * 2;
constexpr auto IGWAN_COUNTS_PER_REVOLUTION = 318;
constexpr auto WHEEL_RADIUS = 1.205;
constexpr auto WHEEL_CIRCUMFERENCE = TAU * WHEEL_RADIUS;
constexpr auto IGWAN_COUNTS_PER_INCH =
    IGWAN_COUNTS_PER_REVOLUTION / WHEEL_CIRCUMFERENCE;

const auto TOUCH_OFFSET_Y = -4;  // our Proteus innacurately reports touch
                                 // location, so we must account for it

const auto TRUE_LCD_WIDTH = 320;
const auto TRUE_LCD_HEIGHT = 240;
const auto LCD_WIDTH = TRUE_LCD_WIDTH - 1;
const auto LCD_HEIGHT = TRUE_LCD_HEIGHT - 2;
const auto FONT_WIDTH = 12;
const auto FONT_HEIGHT = 17;
inline float touch_x, touch_y;
inline bool touch_pressed;

#define LOG_INFO(message)                                                \
    do {                                                                 \
        std::stringstream ss;                                            \
        ss.precision(4);                                                 \
        ss << message;                                                   \
        logger->info(ss.str(), __FILE__, __PRETTY_FUNCTION__, __LINE__); \
    } while (0)

class Log {
   public:
    Log();
    Log(const Log& obj) = delete;

    void info(std::string message,
              const char* file,
              const char* pretty_function,
              int line);

    friend class LogUI;

   private:
    std::vector<std::string> short_messages;
    std::vector<std::string> long_messages;
};

/// Wrapper for the FEHMotor class
class Motor {
   public:
    /// Constructor for the Motor class
    Motor(FEHMotor::FEHMotorPort motor_port,
          FEHIO::FEHIOPin encoder_pin,
          double correction_factor);

    /// Sets the power of the motor
    /// @param power The power to set the motor to between -1 and 1
    void drive(double power);

    /// Gets the distance the motor has traveled in inches
    double get_distance();

    void flush();

   private:
    FEHMotor motor;
    DigitalEncoder encoder;
    double correction_factor;
    double power;
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

    virtual void start();

    /// The name of the step
    std::string name;

    /// The time at which the step started
    double t_start;

    /// The time at which the step ended
    double t_end;
};

inline auto logger = std::make_shared<Log>();

template <typename T, typename... Ts>
std::shared_ptr<T> make_shared(Ts&&... ts) {
    return std::shared_ptr<T>(new T{std::forward<Ts>(ts)...});
}

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

class UIWindow {
   public:
    /// Constructor for the UIWindow class
    UIWindow(Rect bounds);

    /// Initial bulk render of the window (full re-render)
    virtual void render() = 0;

    /// Inject updates to the UI
    virtual void update() = 0;

   protected:
    Rect bounds;
};

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

struct NavbarButton {
    NavbarButton(Rect bounding_box,
                 std::string text,
                 std::function<void()> on_button_down);

    Rect bounding_box;
    std::string text;
    std::function<void()> on_button_down;
    std::unique_ptr<TouchableRegion> region;

    void render(bool is_pressed);
};

class Navbar {
   public:
    Navbar();

    void add_button(const std::string& text,
                    std::function<void()> on_button_down);

    void update();

    Rect bounding_box;

    friend class LogUI;
    friend class TimelineUI;
    friend class MiscUI;

   private:
    const unsigned int OUTER_PADDING = 4;
    const unsigned int INNER_PADDING = 2;

    std::vector<NavbarButton> buttons;
    unsigned int x;
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

    friend class TimelineUI;

   private:
    /// The steps to execute
    const std::vector<std::shared_ptr<Step>> steps;

    bool is_playing = false;

    double t = 0;

    /// The index of the current step
    size_t current_step_index = 0;
};

template <typename... Ts>
Timeline::Timeline(Ts&&... ts)
    : steps(make_vector_of_shared<Step>(std::forward<Ts>(ts)...)) {}

class PlayPauseButton {
   public:
    enum State {
        Play,
        Pause,
    };

    PlayPauseButton(Rect bounding_box,
                    std::function<State(State)> on_button_down,
                    State default_state);

    Rect bounding_box;

    State current_state;

    void render();

    void update();

   private:
    std::unique_ptr<TouchableRegion> region;
};

class TimelineUI : public UIWindow {
   public:
    /// Constructor for the TimelineUI class
    /// @param timeline The timeline to display
    TimelineUI(Rect bounds, Timeline& timeline, Navbar& navbar);

    /// Initial bulk render of the window (full re-render)
    void render() override;

    /// Inject updates to the UI
    void update() override;

   private:
    void paginate();

    size_t prev_timeline_step_index = 0;

    /// The timeline to display
    Timeline& timeline;

    Navbar& navbar;

    std::unique_ptr<TouchableRegion> region_page_up;
    std::unique_ptr<TouchableRegion> region_page_down;
    std::unique_ptr<PlayPauseButton> button_pause_play;
    std::vector<std::unique_ptr<PlayPauseButton>> button_play_steps;

    size_t scroll_index = 0;

    const size_t BUTTON_MEASURE = 2 * FONT_HEIGHT;
};

class LogUI : public UIWindow {
   public:
    LogUI(Rect bounds, Navbar& navbar);

    /// Initial bulk render of the window (full re-render)
    void render() override;

    /// Inject updates to the UI
    void update() override;

   private:
    /// The UI regions which can be touched
    std::vector<TouchableRegion> regions;

    Navbar& navbar;
};

class MiscUI : public UIWindow {
   public:
    MiscUI(Rect bounds, Navbar& navbar);

    /// Initial bulk render of the window (full re-render)
    void render() override;

    /// Inject updates to the UI
    void update() override;

   private:
    std::unique_ptr<TouchableRegion> m1_region;
    double m1_start_time = 0;

    std::unique_ptr<TouchableRegion> m2_region;
    double m2_start_time = 0;

    std::unique_ptr<TouchableRegion> m3_region;
    double m3_start_time = 0;

    void update_motor_button_ui(std::unique_ptr<TouchableRegion>& region,
                                double& start_time);

    void calibrate(Motor& motor, double& start_time);

    Navbar& navbar;
};

inline Motor m1(FEHMotor::Motor0, FEHIO::P3_5, 1);
inline Motor m2(FEHMotor::Motor1, FEHIO::P3_3, 0.987543237);
inline Motor m3(FEHMotor::Motor2, FEHIO::P3_1, 0.972031068);

inline AnalogInputPin cds(FEHIO::P3_7);
