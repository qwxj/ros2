#include "my_hardware/diff_wheel.hpp"

#include <algorithm>
#include <cmath>
#include <string>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace my_hardware
{

    using hardware_interface::CallbackReturn;
    using hardware_interface::return_type;

    void DiffWheel::logTransition(const char* cb, const rclcpp_lifecycle::State & prev)
    {
        RCLCPP_INFO(
            rclcpp::get_logger("DiffWheel"),
            "[LIFECYCLE] %-14s previous_state=%s", cb, prev.label().c_str()
        );
    }


    CallbackReturn DiffWheel::on_init(const hardware_interface::HardwareInfo& info)
    {
        if (hardware_interface::SystemInterface::on_init(info)!=CallbackReturn::SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("DiffWheel"), "基类on_init失败");
            return CallbackReturn::ERROR;
        }

        if(info_.joints.size()!= 2)
        {
            RCLCPP_ERROR(
                rclcpp::get_logger("DiffWheel"),
                "差速底盘需要正好 2 个关节，URDF 里是 %zu 个", info_.joints.size());
                return CallbackReturn::ERROR;
        }
        
        if(info_.hardware_parameters.count("max_wheel_vel"))
        {
            max_wheel_vel_ = std::stod(info_.hardware_parameters.at("max_wheel_vel"));
        }

        if(info_.hardware_parameters.count("tau"))
        {
            tau_ = std::stod(info_.hardware_parameters.at("tau"));
        }

        if(info_.hardware_parameters.count("encoder_cpr"))
        {
            encoder_cpr_ = std::stoi(info_.hardware_parameters.at("encoder_cpr"));
        }

        const std::size_t n = info_.joints.size();
        pos_.assign(n, 0.0);
        vel_.assign(n, 0.0);
        cmd_vel_.assign(n, 0.0);
        target_vel_.assign(n, 0.0);
        true_pos_.assign(n, 0.0);

        for(const auto& j : info_.joints)
        {
            RCLCPP_INFO(rclcpp::get_logger("DiffWheel"), " 关节 %s", j.name.c_str());
        }

        RCLCPP_INFO(
            rclcpp::get_logger("DiffWheel"),
            "  max_wheel_vel=%.1f rad/s  tau=%.3f s  encoder_cpr=%d",
            max_wheel_vel_, tau_, encoder_cpr_);

        return CallbackReturn::SUCCESS;
    }

    CallbackReturn DiffWheel::on_configure(const rclcpp_lifecycle::State & previous_state)
    {
        logTransition("on_configure", previous_state);
        return CallbackReturn::SUCCESS;
    }
    CallbackReturn DiffWheel::on_activate(const rclcpp_lifecycle::State & previous_state)
    {
        logTransition("on_activate", previous_state);
        std::fill(pos_.begin(), pos_.end(), 0.0);
        std::fill(vel_.begin(), vel_.end(), 0.0);
        std::fill(true_pos_.begin(), true_pos_.end(), 0.0);
        read_count_ = 0;
        write_count_ = 0;

        total_dt_ = 0.0;
        t_start_  = std::chrono::steady_clock::now();

        return CallbackReturn::SUCCESS;
    }

    CallbackReturn DiffWheel::on_deactivate(const rclcpp_lifecycle::State & previous_state)
    {
        logTransition("on_deactivate", previous_state);
        RCLCPP_INFO(
            rclcpp::get_logger("DiffWheel"),
            "  累计: read=%lu write=%lu", read_count_, write_count_);
        return CallbackReturn::SUCCESS;
    }

    std::vector<hardware_interface::StateInterface> DiffWheel::export_state_interfaces()
    {
        std::vector<hardware_interface::StateInterface> si;
        si.reserve(info_.joints.size() * 2);
        for(std::size_t i=0;i<info_.joints.size(); i++)
        {
            si.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_POSITION, &pos_[i]);
            si.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &vel_[i]);

            RCLCPP_INFO(
            rclcpp::get_logger("DiffWheel"),
            "[EXPORT] state   : %s/{position,velocity} -> &pos_[%zu], &vel_[%zu]",
            info_.joints[i].name.c_str(), i, i);
        }

        return si;
    }


    std::vector<hardware_interface::CommandInterface> DiffWheel::export_command_interfaces()
    {
        std::vector<hardware_interface::CommandInterface> ci;
        ci.reserve(info_.joints.size());
        for(std::size_t i=0;i<info_.joints.size(); i++)
        {
            ci.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &cmd_vel_[i]);
        }
        return ci;
    }

    return_type DiffWheel::write(const rclcpp::Time& time, const rclcpp::Duration& period)
    {
        ++write_count_;

        for(std::size_t i=0;i<cmd_vel_.size();i++)
        {
            target_vel_[i] = std::clamp(cmd_vel_[i], -max_wheel_vel_, max_wheel_vel_);
        }
        if(write_count_ %100==1)
        {
            RCLCPP_INFO(
                rclcpp::get_logger("DiffWheel"),
                "[PERIOD] write #%lu  cmd=[%.3f, %.3f] -> target=[%.3f, %.3f]",
                write_count_, cmd_vel_[0], cmd_vel_[1], target_vel_[0], target_vel_[1]);
        }

        return return_type::OK;

    }

    return_type DiffWheel::read(const rclcpp::Time& time, const rclcpp::Duration& period)
    {
        ++read_count_;

        const double dt = period.seconds();
        total_dt_ += dt;
        const double alpha = 1.0 - std::exp(-dt / std::max(tau_, 1e-6));
        const double step = (encoder_cpr_ > 0) ? (2.0*M_PI/static_cast<double>(encoder_cpr_)) : 0.0;

        for(std::size_t i=0;i<pos_.size(); i++)
        {
            vel_[i] += (target_vel_[i] - vel_[i]) * alpha;

            // ② 真实位置积分（连续，不量化）
            true_pos_[i] += vel_[i] * dt;

            // ③ 编码器读数 = 量化(真实位置)
            pos_[i] = (step > 0.0)
                ? std::round(true_pos_[i] / step) * step
                : true_pos_[i];
        }

        if (read_count_ % 100 == 1) {
            RCLCPP_INFO(
                rclcpp::get_logger("DiffWheel"),
                "[PERIOD] read #%lu  pos=[%.6f, %.6f]  true=[%.6f, %.6f]  vel=[%.4f, %.4f]",
                read_count_,
                pos_[0], pos_[1],
                true_pos_[0], true_pos_[1],
                vel_[0], vel_[1]);
        }

        // ── 每 10 秒打一条：period 累计 vs 墙钟 ──
        if (read_count_ % 1000 == 1 && read_count_ > 1) {
            const double wall = std::chrono::duration<double>(
                std::chrono::steady_clock::now() - t_start_).count();
            const double diff = wall - total_dt_;
            RCLCPP_INFO(
                rclcpp::get_logger("DiffWheel"),
                "[CLOCK] cycles=%lu  累计dt=%.4f s  墙钟=%.4f s  差=%.4f s (%.2f%%)",
                read_count_, total_dt_, wall, diff,
                (wall > 1e-9) ? (diff / wall * 100.0) : 0.0);
        }
        return return_type::OK;

    }

}

PLUGINLIB_EXPORT_CLASS(my_hardware::DiffWheel, hardware_interface::SystemInterface)