/// @file util.cpp
/// @author Mark Bundschuh
/// @brief Utility functions and classes

#include <FEHBattery.h>
#include <FEHLCD.h>
#include <FEHUtility.h>
#include <LCDColors.h>

#include <algorithm>
#include <sstream>

#include "FEHIO.h"
#include "util.h"

/// Helper function to clamp a value between a lower and upper bound
/// @param n The value to clamp
/// @param lower The lower bound
/// @param upper The upper bound
template <typename T>
T clamp(const T& n, const T& lower, const T& upper) {
    return std::max(lower, std::min(n, upper));
}

Log::Log() {}

void Log::info(std::string message,
               const char* file,
               const char* pretty_function,
               int line) {
    std::stringstream short_message;
    short_message << line << "|" << message;
    short_messages.push_back(short_message.str());

    std::stringstream long_message;
    long_message << "[" << TimeNow() << "|" << file << "|" << pretty_function
                 << "|" << line << "] " << message;
    long_messages.push_back(long_message.str());
}

Motor::Motor(FEHMotor::FEHMotorPort port,
             FEHIO::FEHIOPin encoder_pin,
             double correction_factor)
    : motor(port, 9.0),
      correction_factor(correction_factor),
      encoder(encoder_pin) {}

void Motor::drive(double power) {
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

Step::Step(std::string name) : t_start(0), t_end(0), name(name) {}

bool Step::execute(double t) { return true; }

void Step::start() {}

bool UnionStep::execute(double t) {
    for (auto& step : steps)
        step->t_start = t_start;

    return std::all_of(steps.begin(), steps.end(), [t](const auto& step) {
        return step->execute(t);
    });
}

UIWindow::UIWindow(Rect bounds) : bounds(bounds) {}

TouchableRegion::TouchableRegion(Rect rect,
                                 std::function<void()> on_button_enter,
                                 std::function<void()> on_button_exit)
    : rect(rect),
      state(ButtonState::OutOfBounds),
      on_button_enter(on_button_enter),
      on_button_exit(on_button_exit) {}

void TouchableRegion::update() {
    bool in_bounds = touch_x >= rect.x && touch_x <= rect.x + rect.width &&
                     touch_y + TOUCH_OFFSET_Y >= rect.y &&
                     touch_y + TOUCH_OFFSET_Y <= rect.y + rect.height;

    if (in_bounds && touch_pressed && state == ButtonState::OutOfBounds) {
        state = ButtonState::InBounds;
        on_button_enter();
    } else if (!in_bounds && state != ButtonState::OutOfBounds) {
        state = ButtonState::OutOfBounds;
        on_button_exit();
    }
}

NavbarButton::NavbarButton(Rect bounding_box,
                           std::string text,
                           std::function<void()> on_button_down)
    : bounding_box(bounding_box), text(text), on_button_down(on_button_down) {
    region = std::make_unique<TouchableRegion>(
        bounding_box, on_button_down, []() {});
}

void NavbarButton::render(bool is_pressed) {
    LCD.SetFontColor(is_pressed ? GRAY : BLACK);
    LCD.FillRectangle(bounding_box.x + 1,
                      bounding_box.y + 1,
                      bounding_box.width - 1,
                      bounding_box.height - 1);
    LCD.SetFontColor(WHITE);
    LCD.SetBackgroundColor(is_pressed ? GRAY : BLACK);
    LCD.DrawRectangle(bounding_box.x,
                      bounding_box.y,
                      bounding_box.width,
                      bounding_box.height);
    LCD.WriteAt(text.c_str(), bounding_box.x + 1, bounding_box.y + 3);
}

Navbar::Navbar() : x(0), current_selected(0) {
    const auto y = INNER_PADDING + FONT_HEIGHT + INNER_PADDING;
    bounding_box = Rect(0, 0, LCD_WIDTH, y);
    LCD.DrawHorizontalLine(y, 0, LCD_WIDTH);
}

void Navbar::add_button(const std::string& text,
                        std::function<void()> on_button_down) {
    const auto TEXT_WIDTH = text.length() * FONT_WIDTH;
    const Rect button_rect(OUTER_PADDING + x,
                           0,
                           INNER_PADDING + TEXT_WIDTH + INNER_PADDING,
                           INNER_PADDING + FONT_HEIGHT + INNER_PADDING);

    auto index = buttons.size();
    buttons.push_back(NavbarButton(
        button_rect,
        text,
        [this, index, on_button_down]() {  // future segfault, `this` could move
            if (this->current_selected != index) {
                buttons[this->current_selected].render(false);
                buttons[index].render(true);
                this->current_selected = index;

                on_button_down();
            }
        }));

    buttons.back().render(false);

    // the first insertion should be selected and render its own ui
    if (index == 0) {
        buttons[this->current_selected].render(true);
        on_button_down();
    }

    x += button_rect.width + OUTER_PADDING;
}

void Navbar::update() {
    for (auto& button : buttons)
        button.region->update();
}

bool Timeline::timestep(double dt) {
    if (current_step_index >= steps.size())
        return false;

    if (is_playing)
        t += dt;
    else
        return false;

    auto& step = steps[current_step_index];
    step->t_end = t;
    if (step->execute(t)) {
        current_step_index++;

        if (current_step_index < steps.size()) {
            m1.flush();
            m2.flush();
            m3.flush();
            auto& next_step = steps[current_step_index];
            next_step->t_start = t;
            next_step->start();
        }
    }

    // if the timeline is done, then reset it
    if (current_step_index >= steps.size()) {
        current_step_index = 0;
        auto& first_step = steps[current_step_index];
        first_step->t_start = t;
        first_step->t_end = t;
        is_playing = false;

        return true;
    }

    return false;
}

void PlayPauseButton::render() {
    size_t tri_measure = 10;

    LCD.SetFontColor(BLACK);
    LCD.FillRectangle(bounding_box.x,
                      bounding_box.y,
                      bounding_box.width,
                      bounding_box.height);

    LCD.SetFontColor(WHITE);

    LCD.DrawRectangle(bounding_box.x,
                      bounding_box.y,
                      bounding_box.width,
                      bounding_box.height);

    if (current_state == Pause) {
        const auto center_line = bounding_box.x + bounding_box.width / 2;
        for (size_t i = 0; i < 3; i++) {
            LCD.DrawVerticalLine(center_line - i - 4,
                                 bounding_box.y + 8,
                                 bounding_box.y + bounding_box.height - 8);
            LCD.DrawVerticalLine(center_line + i + 4,
                                 bounding_box.y + 8,
                                 bounding_box.y + bounding_box.height - 8);
        }
    } else if (current_state == Play) {
        for (size_t i = 0; i < tri_measure; i++) {
            size_t y = bounding_box.y + bounding_box.height / 2;
            LCD.DrawVerticalLine(
                bounding_box.x + bounding_box.width / 2 + tri_measure / 2 - i,
                y - i,
                y + i);
        }
    }
}

void PlayPauseButton::update() { region->update(); }

PlayPauseButton::PlayPauseButton(Rect bounding_box,
                                 std::function<State(State)> on_button_down,
                                 State default_state)
    : bounding_box(bounding_box), current_state(default_state) {
    region = std::make_unique<TouchableRegion>(
        bounding_box,
        [this, on_button_down]() {
            auto new_state = on_button_down(current_state);
            if (new_state != current_state) {
                current_state = new_state;
                render();
            }
        },
        []() {});
}

TimelineUI::TimelineUI(Rect bounds, Timeline& timeline, Navbar& navbar)
    : UIWindow(bounds), timeline(timeline), navbar(navbar) {
    region_page_up = std::make_unique<TouchableRegion>(
        Rect(bounds.x + bounds.width - BUTTON_MEASURE,
             bounds.y - 1,
             BUTTON_MEASURE,
             BUTTON_MEASURE),
        [&]() {
            if (scroll_index > 0)
                scroll_index--;
            paginate();
        },
        []() {});

    region_page_down = std::make_unique<TouchableRegion>(
        Rect(bounds.x + bounds.width - BUTTON_MEASURE,
             bounds.y + bounds.height - BUTTON_MEASURE - 1,
             BUTTON_MEASURE,
             BUTTON_MEASURE),
        [&]() {
            if (scroll_index < timeline.steps.size() - 6)
                scroll_index++;
            paginate();
        },
        []() {});

    button_pause_play = std::make_unique<PlayPauseButton>(
        Rect(bounds.x + bounds.width - BUTTON_MEASURE,
             bounds.y + bounds.height / 2 - BUTTON_MEASURE / 2,
             BUTTON_MEASURE,
             BUTTON_MEASURE),
        [&](PlayPauseButton::State state) {
            if (state == PlayPauseButton::Play) {
                timeline.is_playing = true;
            } else if (state == PlayPauseButton::Pause) {
                m1.flush();
                m2.flush();
                m3.flush();
                timeline.is_playing = false;
            }

            return state == PlayPauseButton::Play ? PlayPauseButton::Pause
                                                  : PlayPauseButton::Play;
        },
        PlayPauseButton::State::Play);

    Rect block = Rect(1,
                      bounds.y - 1,
                      bounds.width - BUTTON_MEASURE - 1,
                      FONT_HEIGHT * 2 + 3);
    for (size_t i = 0;
         i < std::min(
                 timeline.steps.size(),
                 static_cast<std::vector<std::shared_ptr<Step>>::size_type>(6));
         i++) {
        const auto PLAY_BUTTON_MEASURE = block.height;
        Rect play_rect(block.x + block.width - PLAY_BUTTON_MEASURE,
                       block.y,
                       PLAY_BUTTON_MEASURE,
                       PLAY_BUTTON_MEASURE);
        button_play_steps.push_back(std::make_unique<PlayPauseButton>(
            play_rect,
            [&, i](PlayPauseButton::State state) {
                if (state == PlayPauseButton::Play) {
                    prev_timeline_step_index = timeline.current_step_index;
                    timeline.is_playing = true;
                    timeline.current_step_index = scroll_index + i;
                    auto& step = timeline.steps[timeline.current_step_index];
                    step->t_start = timeline.t;
                    step->t_end = timeline.t;
                    m1.flush();
                    m2.flush();
                    m3.flush();
                    step->start();

                    button_pause_play->current_state =
                        PlayPauseButton::State::Pause;
                    button_pause_play->render();
                    update();
                }

                return PlayPauseButton::State::Play;
            },
            PlayPauseButton::State::Play));

        block.y += block.height - 1;
    }
}

void TimelineUI::render() {
    LCD.SetFontColor(BLACK);
    LCD.FillRectangle(bounds.x, bounds.y, bounds.width + 1, bounds.height);

    LCD.SetFontColor(WHITE);
    LCD.SetBackgroundColor(BLACK);

    LCD.DrawVerticalLine(
        bounds.x + bounds.width, bounds.y, bounds.y + bounds.height);

    size_t tri_measure = 10;

    Rect page_up_rect = region_page_up->rect;
    LCD.DrawRectangle(page_up_rect.x,
                      page_up_rect.y,
                      page_up_rect.width,
                      page_up_rect.height);
    for (size_t i = 0; i < tri_measure; i++) {
        size_t x = page_up_rect.x + page_up_rect.width / 2;
        LCD.DrawHorizontalLine(
            page_up_rect.y + page_up_rect.height / 2 - tri_measure / 2 + i,
            x - i,
            x + i);
    }

    Rect page_down_rect = region_page_down->rect;
    LCD.DrawRectangle(page_down_rect.x,
                      page_down_rect.y,
                      page_down_rect.width,
                      page_down_rect.height);
    for (size_t i = 0; i < tri_measure; i++) {
        size_t x = page_down_rect.x + page_down_rect.width / 2;
        LCD.DrawHorizontalLine(
            page_down_rect.y + page_down_rect.height / 2 + tri_measure / 2 - i,
            x - i,
            x + i);
    }

    Rect block = Rect(1,
                      bounds.y - 1,
                      bounds.width - BUTTON_MEASURE - 1,
                      FONT_HEIGHT * 2 + 3);
    for (size_t i = scroll_index;
         i < std::min(timeline.steps.size(), scroll_index + 6);
         i++) {
        auto& step = timeline.steps[i];
        LCD.DrawRectangle(block.x, block.y, block.width, block.height);
        LCD.WriteAt(step->name.substr(0, 20).c_str(), block.x + 1, block.y + 3);
        LCD.WriteAt("t=", block.x + 1, block.y + 2 + FONT_HEIGHT);
        std::stringstream time_stream;
        time_stream.setf(std::ios::fixed);
        time_stream.precision(3);
        time_stream << (step->t_end - step->t_start) << "s     ";
        LCD.WriteAt(time_stream.str().c_str(),
                    block.x + 1 + (FONT_WIDTH * 2),
                    block.y + 2 + FONT_HEIGHT);

        block.y += block.height - 1;
    }

    button_pause_play->render();
    for (auto& button : button_play_steps)
        button->render();
}

void TimelineUI::update() {
    if (navbar.current_selected != 0)
        return;

    if (timeline.current_step_index >= timeline.steps.size())
        return;

    region_page_up->update();
    region_page_down->update();
    button_pause_play->update();
    for (auto& button : button_play_steps)
        button->update();

    if (timeline.current_step_index != prev_timeline_step_index) {
        if (!timeline.is_playing) {
            button_pause_play->current_state = PlayPauseButton::State::Play;
            button_pause_play->render();
        }

        if (prev_timeline_step_index < scroll_index) {
            // clear top status indicator
            auto step_block = Rect(1,
                                   bounds.y - 1,
                                   bounds.width - BUTTON_MEASURE - 1,
                                   FONT_HEIGHT * 2 + 3);
            LCD.SetFontColor(BLACK);
            LCD.FillRectangle(
                step_block.x + step_block.width - BUTTON_MEASURE - 15 - 5,
                step_block.y + 1,
                10,
                2);
        } else if (prev_timeline_step_index >= scroll_index + 6) {
            // clear bottom status indicator
            auto step_block = Rect(1,
                                   bounds.y - 1 + (FONT_HEIGHT * 2 + 3 - 1) * 5,
                                   bounds.width - BUTTON_MEASURE - 1,
                                   FONT_HEIGHT * 2 + 3);
            LCD.SetFontColor(BLACK);
            LCD.FillRectangle(
                step_block.x + step_block.width - BUTTON_MEASURE - 15 - 5,
                step_block.y + step_block.height - 3,
                10,
                2);
        } else {
            const auto j = prev_timeline_step_index - scroll_index;
            auto status_block = Rect(
                200,
                bounds.y - 1 + (FONT_HEIGHT * 2 + 3 - 1) * j + 2 + FONT_HEIGHT,
                bounds.width - BUTTON_MEASURE - 1 - 199 - BUTTON_MEASURE - 3,
                FONT_HEIGHT);

            LCD.SetFontColor(BLACK);
            LCD.FillRectangle(status_block.x,
                              status_block.y,
                              status_block.width,
                              status_block.height);
        }

        // automatically scroll if needed
        if (timeline.current_step_index > prev_timeline_step_index &&
            timeline.current_step_index >= scroll_index + 6) {
            if (scroll_index < timeline.steps.size() - 6)
                scroll_index++;
            paginate();
        }

        // update the last timestep of the previously updated index
        if (prev_timeline_step_index >= scroll_index &&
            prev_timeline_step_index < scroll_index + 6) {
            const auto i = prev_timeline_step_index - scroll_index;
            auto step_block = Rect(1,
                                   bounds.y - 1 + (FONT_HEIGHT * 2 + 3 - 1) * i,
                                   bounds.width - BUTTON_MEASURE - 1,
                                   FONT_HEIGHT * 2 + 3);
            auto& step = timeline.steps[prev_timeline_step_index];

            LCD.SetFontColor(WHITE);
            std::stringstream time_stream;
            time_stream.setf(std::ios::fixed);
            time_stream.precision(3);
            time_stream << (step->t_end - step->t_start) << "s     ";
            LCD.WriteAt(time_stream.str().c_str(),
                        step_block.x + 1 + (FONT_WIDTH * 2),
                        step_block.y + 2 + FONT_HEIGHT);
        }
    }

    prev_timeline_step_index = timeline.current_step_index;

    LCD.SetFontColor(timeline.is_playing ? GREEN : RED);

    // set top status indicator
    if (timeline.current_step_index < scroll_index) {
        auto step_block = Rect(1,
                               bounds.y - 1,
                               bounds.width - BUTTON_MEASURE - 1,
                               FONT_HEIGHT * 2 + 3);
        LCD.FillRectangle(
            step_block.x + step_block.width - BUTTON_MEASURE - 15 - 5,
            step_block.y + 1,
            10,
            2);
        return;
    }

    // set bottom status indicator
    if (timeline.current_step_index >= scroll_index + 6) {
        auto step_block = Rect(1,
                               bounds.y - 1 + (FONT_HEIGHT * 2 + 3 - 1) * 5,
                               bounds.width - BUTTON_MEASURE - 1,
                               FONT_HEIGHT * 2 + 3);
        LCD.FillRectangle(
            step_block.x + step_block.width - BUTTON_MEASURE - 15 - 5,
            step_block.y + step_block.height - 3,
            10,
            2);
        return;
    }

    const auto i = timeline.current_step_index - scroll_index;
    auto step_block = Rect(1,
                           bounds.y - 1 + (FONT_HEIGHT * 2 + 3 - 1) * i,
                           bounds.width - BUTTON_MEASURE - 1,
                           FONT_HEIGHT * 2 + 3);
    auto& step = timeline.steps[timeline.current_step_index];

    LCD.SetFontColor(WHITE);
    std::stringstream time_stream;
    time_stream.setf(std::ios::fixed);
    time_stream.precision(3);
    time_stream << (step->t_end - step->t_start) << "s     ";
    LCD.WriteAt(time_stream.str().c_str(),
                step_block.x + 1 + (FONT_WIDTH * 2),
                step_block.y + 2 + FONT_HEIGHT);

    LCD.SetFontColor(timeline.is_playing ? GREEN : RED);
    LCD.FillCircle(step_block.x + step_block.width - BUTTON_MEASURE - 15,
                   step_block.y + step_block.height - 12,
                   5);
}

void TimelineUI::paginate() {
    LCD.SetBackgroundColor(BLACK);

    Rect block = Rect(1,
                      bounds.y - 1,
                      bounds.width - BUTTON_MEASURE - 1,
                      FONT_HEIGHT * 2 + 3);
    for (size_t i = scroll_index;
         i < std::min(timeline.steps.size(), scroll_index + 6);
         i++) {
        auto& step = timeline.steps[i];
        LCD.SetFontColor(BLACK);
        const auto& name = step->name.substr(0, 20);
        const auto name_font_length = name.length() * FONT_WIDTH;
        LCD.FillRectangle(block.x + 1 + name_font_length,
                          block.y + 3,
                          block.width - BUTTON_MEASURE - 4 - name_font_length,
                          FONT_HEIGHT);

        const auto status_offset = 198;
        LCD.FillRectangle(block.x + 1 + status_offset,
                          block.y + 2 + FONT_HEIGHT,
                          block.width - BUTTON_MEASURE - 4 - status_offset,
                          FONT_HEIGHT);

        LCD.SetFontColor(WHITE);
        LCD.WriteAt(name.c_str(), block.x + 1, block.y + 3);
        std::stringstream time_stream;
        time_stream.setf(std::ios::fixed);
        time_stream.precision(3);
        time_stream << (step->t_end - step->t_start) << "s     ";
        LCD.WriteAt(time_stream.str().c_str(),
                    block.x + 1 + (FONT_WIDTH * 2),
                    block.y + 2 + FONT_HEIGHT);

        block.y += block.height - 1;
    }

    // clear top status indicator
    LCD.SetFontColor(BLACK);
    auto step_block = Rect(1,
                           bounds.y - 1,
                           bounds.width - BUTTON_MEASURE - 1,
                           FONT_HEIGHT * 2 + 3);
    LCD.FillRectangle(step_block.x + step_block.width - BUTTON_MEASURE - 15 - 5,
                      step_block.y + 1,
                      10,
                      2);
}

LogUI::LogUI(Rect bounds, Navbar& navbar) : UIWindow(bounds), navbar(navbar) {}

void LogUI::render() {
    LCD.SetFontColor(BLACK);
    LCD.FillRectangle(bounds.x, bounds.y, bounds.width + 1, bounds.height);

    update();
}

void LogUI::update() {
    if (navbar.current_selected != 1)
        return;

    LCD.SetBackgroundColor(BLACK);
    LCD.SetFontColor(WHITE);

    const size_t max_lines = bounds.height / FONT_HEIGHT;
    size_t scroll_index = 0;
    if (logger->short_messages.size() > max_lines)
        scroll_index = logger->short_messages.size() - max_lines;
    for (size_t log_index = scroll_index, ui_index = 0;
         log_index < scroll_index + max_lines &&
         log_index < logger->short_messages.size();
         log_index++, ui_index++) {
        LCD.WriteAt(logger->short_messages[log_index].substr(0, 25).c_str(),
                    bounds.x,
                    bounds.y + (ui_index * FONT_HEIGHT) + 2);
    }
}

MiscUI::MiscUI(Rect bounds, Navbar& navbar) : UIWindow(bounds), navbar(navbar) {
    constexpr auto BUTTON_MEASURE = 30;
    const auto m1_rect = Rect(bounds.x + bounds.width - BUTTON_MEASURE,
                              bounds.y + BUTTON_MEASURE,
                              BUTTON_MEASURE,
                              BUTTON_MEASURE);

    m1_region = std::make_unique<TouchableRegion>(m1_rect, [m1_rect, this]() {
        LCD.SetFontColor(GREEN);
        LCD.FillRectangle(m1_rect.x, m1_rect.y, m1_rect.width, m1_rect.height);
        LCD.SetFontColor(WHITE);
        LCD.DrawRectangle(m1_rect.x, m1_rect.y, m1_rect.width, m1_rect.height);
        m1.flush();
        m1_start_time = TimeNow();
    });

    const auto m2_rect = Rect(
        m1_rect.x, m1_rect.y + BUTTON_MEASURE, m1_rect.width, m1_rect.height);
    m2_region = std::make_unique<TouchableRegion>(m2_rect, [m2_rect, this]() {
        LCD.SetFontColor(GREEN);
        LCD.FillRectangle(m2_rect.x, m2_rect.y, m2_rect.width, m2_rect.height);
        LCD.SetFontColor(WHITE);
        LCD.DrawRectangle(m2_rect.x, m2_rect.y, m2_rect.width, m2_rect.height);
        m2.flush();
        m2_start_time = TimeNow();
    });

    const auto m3_rect = Rect(
        m2_rect.x, m2_rect.y + BUTTON_MEASURE, m2_rect.width, m2_rect.height);
    m3_region = std::make_unique<TouchableRegion>(m3_rect, [m3_rect, this]() {
        LCD.SetFontColor(GREEN);
        LCD.FillRectangle(m3_rect.x, m3_rect.y, m3_rect.width, m3_rect.height);
        LCD.SetFontColor(WHITE);
        LCD.DrawRectangle(m3_rect.x, m3_rect.y, m3_rect.width, m3_rect.height);
        m3.flush();
        m3_start_time = TimeNow();
    });
}

void MiscUI::render() {
    LCD.SetFontColor(BLACK);
    LCD.FillRectangle(bounds.x, bounds.y, bounds.width + 1, bounds.height);

    LCD.SetFontColor(m1_start_time == 0 ? RED : GREEN);
    LCD.FillRectangle(m1_region->rect.x,
                      m1_region->rect.y,
                      m1_region->rect.width,
                      m1_region->rect.height);
    LCD.SetFontColor(WHITE);
    LCD.DrawRectangle(m1_region->rect.x,
                      m1_region->rect.y,
                      m1_region->rect.width,
                      m1_region->rect.height);

    LCD.SetFontColor(m2_start_time == 0 ? RED : GREEN);
    LCD.FillRectangle(m2_region->rect.x,
                      m2_region->rect.y,
                      m2_region->rect.width,
                      m2_region->rect.height);
    LCD.SetFontColor(WHITE);
    LCD.DrawRectangle(m2_region->rect.x,
                      m2_region->rect.y,
                      m2_region->rect.width,
                      m2_region->rect.height);

    LCD.SetFontColor(m3_start_time == 0 ? RED : GREEN);
    LCD.FillRectangle(m3_region->rect.x,
                      m3_region->rect.y,
                      m3_region->rect.width,
                      m3_region->rect.height);
    LCD.SetFontColor(WHITE);
    LCD.DrawRectangle(m3_region->rect.x,
                      m3_region->rect.y,
                      m3_region->rect.width,
                      m3_region->rect.height);

    update();
}

void MiscUI::update_motor_button_ui(std::unique_ptr<TouchableRegion>& region,
                                    double& start_time) {
    region->update();
    if (start_time != 0 && TimeNow() - start_time >= 5.0) {
        LCD.SetFontColor(RED);
        LCD.FillRectangle(region->rect.x,
                          region->rect.y,
                          region->rect.width,
                          region->rect.height);
        LCD.SetFontColor(WHITE);
        LCD.DrawRectangle(region->rect.x,
                          region->rect.y,
                          region->rect.width,
                          region->rect.height);
    }
}

void MiscUI::calibrate(Motor& motor, double& start_time) {
    if (start_time != 0) {
        if (TimeNow() - start_time < 5.0) {
            motor.drive(0.2);
        } else {
            start_time = 0;
            LOG_INFO("calib dist " << motor.get_distance());
            motor.flush();
        }
    }
}

void MiscUI::update() {
    calibrate(m1, m1_start_time);
    calibrate(m2, m2_start_time);
    calibrate(m3, m3_start_time);

    if (navbar.current_selected != 2)
        return;

    LCD.SetBackgroundColor(BLACK);
    LCD.SetFontColor(WHITE);

    std::stringstream battery_stream;
    battery_stream << "Battery (0-11.7V): " << Battery.Voltage();
    LCD.WriteAt(
        battery_stream.str().c_str(), bounds.x, bounds.y + 2 + FONT_HEIGHT * 0);

    std::stringstream cds_stream;
    cds_stream << "Cds (0-3.3V): " << cds.Value();
    LCD.WriteAt(
        cds_stream.str().c_str(), bounds.x, bounds.y + 2 + FONT_HEIGHT * 1);

    std::stringstream m1_distance_stream;
    m1_distance_stream << "M1 dist (in): " << m1.get_distance();
    LCD.WriteAt(m1_distance_stream.str().c_str(),
                bounds.x,
                bounds.y + 2 + FONT_HEIGHT * 2);

    std::stringstream m2_distance_stream;
    m2_distance_stream << "M2 dist (in): " << m2.get_distance();
    LCD.WriteAt(m2_distance_stream.str().c_str(),
                bounds.x,
                bounds.y + 2 + FONT_HEIGHT * 3);

    std::stringstream m3_distance_stream;
    m3_distance_stream << "M3 dist (in): " << m3.get_distance();
    LCD.WriteAt(m3_distance_stream.str().c_str(),
                bounds.x,
                bounds.y + 2 + FONT_HEIGHT * 4);

    update_motor_button_ui(m1_region, m1_start_time);
    update_motor_button_ui(m2_region, m2_start_time);
    update_motor_button_ui(m3_region, m3_start_time);
}