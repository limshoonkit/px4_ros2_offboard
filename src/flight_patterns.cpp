#include "flight_patterns.hpp"
#include <cmath>

namespace uosm
{
namespace px4
{
namespace flight_pattern
{

void Pattern::hover(px4_msgs::msg::TrajectorySetpoint &setpoint) const
{
    setpoint.position = {static_cast<float>(params_.offset_x), static_cast<float>(params_.offset_y), static_cast<float>(params_.height)};
    setpoint.yaw = 0.0f;
}

void Pattern::increase_iteration()
{
    if (theta_ >= TWO_PI)
    {
        theta_ = 0.0;
        ++iteration_;
    }
}

void CircularPattern::run(px4_msgs::msg::TrajectorySetpoint &setpoint)
{
    setpoint.position[0] = params_.radius * cos(theta_) + params_.offset_x;
    setpoint.position[1] = params_.radius * sin(theta_) + params_.offset_y;
    setpoint.position[2] = params_.height;

    setpoint.yaw = atan2(params_.offset_y - setpoint.position[1],
                         params_.offset_x - setpoint.position[0]);
    theta_ += params_.speed * params_.dt; // angular velocity
    increase_iteration();
}

void SpiralPattern::run(px4_msgs::msg::TrajectorySetpoint &setpoint)
{
    double completion_rate = (static_cast<double>(iteration_) + theta_ / TWO_PI) / params_.max_iter;
    double current_radius = params_.radius - (params_.radius - min_radius_) * completion_rate;
    double current_height = params_.height - (params_.height - min_height_) * completion_rate;

    setpoint.position[0] = current_radius * cos(theta_) + params_.offset_x;
    setpoint.position[1] = current_radius * sin(theta_) + params_.offset_y;
    setpoint.position[2] = current_height;

    setpoint.yaw = atan2(params_.offset_y - setpoint.position[1],
                         params_.offset_x - setpoint.position[0]);

    theta_ += params_.speed * params_.dt; // angular velocity
    increase_iteration();
}

void CloudPattern::run(px4_msgs::msg::TrajectorySetpoint &setpoint)
{
    double noise_amplitude = params_.radius / 4.0;

    setpoint.position[0] = params_.radius * sin(theta_) +
                           noise_amplitude * sin(params_.frequency * theta_) + params_.offset_x;
    setpoint.position[1] = params_.radius * cos(theta_) +
                           noise_amplitude * cos(params_.frequency * theta_) + params_.offset_y;
    setpoint.position[2] = params_.height;

    setpoint.yaw = atan2(setpoint.position[1] - params_.offset_y, setpoint.position[0] - params_.offset_x);

    double speed_factor = std::abs(sin(2 * params_.frequency * theta_));
    double curr_speed = params_.min_speed +
                        (params_.speed - params_.min_speed) * speed_factor;

    if (initial_progress_ >= (params_.radius + noise_amplitude))
        theta_ += curr_speed * params_.dt;
    else
        initial_progress_ += params_.speed * params_.dt;

    increase_iteration();
}

void SinePattern::run(px4_msgs::msg::TrajectorySetpoint &setpoint)
{
    double progress_step = params_.speed * params_.dt;

    if (first_it_)
    {
        setpoint.position[0] = params_.offset_x;
        first_it_ = false;
    }

    if (forward_)
        setpoint.position[0] += progress_step;
    else
        setpoint.position[0] -= progress_step;

    setpoint.position[1] = params_.offset_y;
    setpoint.position[2] = params_.height +
                           params_.offset_z * sin(params_.frequency * time_);

    setpoint.yaw = 0.0f;

    time_ += params_.dt;
    progress_ += params_.dt;

    if (progress_ >= period_length_)
    {
        period_counter_++;
        progress_ = 0.0;
    }

    if (period_counter_ == 2)
    {
        if (!forward_)
            theta_ = TWO_PI;

        forward_ = !forward_;
        period_counter_ = 0;
    }
    increase_iteration();
}

std::vector<NGramPattern::Point> NGramPattern::calculate_vertices(
    double radius, int n, double offset_x, double offset_y)
{
    std::vector<Point> vertices;
    vertices.reserve(n);
    for (int i = 0; i < n; ++i)
    {
        double angle = TWO_PI * i / n;
        vertices.push_back({radius * cos(angle) + offset_x,
                            radius * sin(angle) + offset_y});
    }
    return vertices;
}

std::vector<int> NGramPattern::generate_order(int n, int k)
{
    std::vector<int> order;
    order.reserve(n);
    int current_vertex = 0;
    std::vector<bool> visited(n, false);

    while (!visited[current_vertex])
    {
        order.push_back(current_vertex);
        visited[current_vertex] = true;
        current_vertex = (current_vertex + k) % n;
    }
    return order;
}

void NGramPattern::run(px4_msgs::msg::TrajectorySetpoint &setpoint)
{
    double start_x = 0.0, start_y = 0.0;
    double end_x = 0.0, end_y = 0.0;

    if (segment_ == -1) // path to the first vertex
    {
        end_x = vertices_[0].x;
        end_y = vertices_[0].y;
    }
    else
    {
        int start_index = order_[segment_ % order_.size()];
        int end_index = order_[(segment_ + 1) % order_.size()];

        start_x = vertices_[start_index].x;
        start_y = vertices_[start_index].y;
        end_x = vertices_[end_index].x;
        end_y = vertices_[end_index].y;
    }

    setpoint.position[0] = start_x + (end_x - start_x) * progress_ + params_.offset_x;
    setpoint.position[1] = start_y + (end_y - start_y) * progress_ + params_.offset_y;
    setpoint.position[2] = params_.height;

    setpoint.yaw = atan2(end_y - setpoint.position[1],
                         end_x - setpoint.position[0]);

    if (segment_ == -1 || segment_ == 0)
        max_distance_ = std::sqrt(std::pow(end_x - start_x, 2) +
                                  std::pow(end_y - start_y, 2));

    double distance_to_next = std::sqrt(std::pow(end_x - setpoint.position[0], 2) +
                                        std::pow(end_y - setpoint.position[1], 2));

    double speed_factor = distance_to_next / max_distance_;
    double current_speed = params_.min_speed +
                           (params_.speed - params_.min_speed) * speed_factor;

    progress_ += current_speed * params_.dt;

    if (progress_ >= 1.0)
    {
        progress_ = 0.0;
        if (segment_ >= 0)
        {
            theta_ += TWO_PI / params_.ngram_vertices;
            increase_iteration();
        }
        segment_++;
    }
}

} // namespace flight_pattern
} // namespace px4
} // namespace uosm