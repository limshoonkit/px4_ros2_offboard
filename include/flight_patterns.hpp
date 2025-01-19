#pragma once

#include <px4_msgs/msg/trajectory_setpoint.hpp>

#include <memory>
#include <vector>
#include <cmath>

namespace uosm
{
namespace px4
{
namespace flight_pattern
{

constexpr double TWO_PI = 2.0 * M_PI;

struct PatternParameters
{
    double dt = 0.0f;
    double radius = 0.0f;
    double height = 0.0f;
    double speed = 0.0f;  // linear or angular velocity
    double min_speed = 0.0f;
    double offset_x = 0.0f;
    double offset_y = 0.0f;
    double offset_z = 0.0f;
    double frequency = 0.0f;
    int ngram_vertices = 7;
    int ngram_step = 2;
    int max_iter = 2;
};

/**
 * @brief Abstract class for flight patterns
 */
class Pattern
{
protected:
    PatternParameters params_; // initial params
    double theta_ = 0.0;       // progression of the pattern
    int iteration_ = 0;        // number of iterations

public:
    Pattern() = default;
    explicit Pattern(const PatternParameters &params) : params_(params) {}
    virtual ~Pattern() = default;

    virtual void run(px4_msgs::msg::TrajectorySetpoint &setpoint) = 0;
    void hover(px4_msgs::msg::TrajectorySetpoint &setpoint) const;
    void increase_iteration();
    bool is_done() const { return iteration_ >= params_.max_iter; }
    const PatternParameters &get_params() const { return params_; }
};

class CircularPattern : public Pattern
{
public:
    explicit CircularPattern(const PatternParameters &params) : Pattern(params) {}
    void run(px4_msgs::msg::TrajectorySetpoint &setpoint) override;
};

class SpiralPattern : public Pattern
{
private:
    double min_height_;
    double min_radius_;

public:
    explicit SpiralPattern(const PatternParameters &params)
        : Pattern(params), min_height_(params.height / 2.0), min_radius_(params.radius * 0.1) {}

    void run(px4_msgs::msg::TrajectorySetpoint &setpoint) override;
};

class CloudPattern : public Pattern
{
private:
    double initial_progress_ = 0.0;

public:
    explicit CloudPattern(const PatternParameters &params) : Pattern(params) {}
    void run(px4_msgs::msg::TrajectorySetpoint &setpoint) override;
};

class SinePattern : public Pattern
{
private:
    double time_ = 0.0;
    int period_counter_ = 0;
    double progress_ = 0.0;
    bool first_it_ = true;
    bool forward_ = true;
    double period_length_;

public:
    explicit SinePattern(const PatternParameters &params)
        : Pattern(params), period_length_(TWO_PI / params.frequency) {}

    void run(px4_msgs::msg::TrajectorySetpoint &setpoint) override;
};

class NGramPattern : public Pattern
{
private:
    struct Point
    {
        double x;
        double y;
    };
    std::vector<Point> vertices_;
    std::vector<int> order_;
    int segment_ = -1;
    double progress_ = 0.0;
    double max_distance_ = 0.0;

    std::vector<Point> calculate_vertices(double radius, int n, double offset_x, double offset_y);
    std::vector<int> generate_order(int n, int k);

public:
    explicit NGramPattern(const PatternParameters &params)
        : Pattern(params)
    {
        vertices_ = calculate_vertices(params.radius, params.ngram_vertices,
                                       params.offset_x, params.offset_y);
        order_ = generate_order(params.ngram_vertices, params.ngram_step);
    }

    void run(px4_msgs::msg::TrajectorySetpoint &setpoint) override;
};

/**
 * @brief Factory class for creating flight patterns
 */
class PatternFactory
{
public:
    enum PatternType
    {
        CIRCULAR = 0,
        SPIRAL,
        CLOUD,
        SINE,
        NGRAM
    };

    static std::unique_ptr<Pattern> createPattern(const PatternType type,
                                                  const PatternParameters &params)
    {
        switch (type)
        {
        case CIRCULAR:
            return std::make_unique<CircularPattern>(params);
        case SPIRAL:
            return std::make_unique<SpiralPattern>(params);
        case CLOUD:
            return std::make_unique<CloudPattern>(params);
        case SINE:
            return std::make_unique<SinePattern>(params);
        case NGRAM:
            return std::make_unique<NGramPattern>(params);
        default:
            throw std::invalid_argument("Unknown pattern type");
        }
    }
};

} // namespace flight_pattern
} // namespace px4
} // namespace uosm