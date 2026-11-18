#ifndef MY_HARDWARE__DIFF_WHEEL_HPP_
#define MY_HARDWARE__DIFF_WHEEL_HPP_

#include <cstddef>
#include <string>
#include <vector>

#include "hardware_interface/system_interface.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"

namespace my_hardware
{

    class DiffWheel : public hardware_interface::SystemInterface
    {
        public:
            hardware_interface::CallbackReturn on_init(
                const hardware_interface::HardwareInfo& infi) override;
            
            hardware_interface::CallbackReturn on_configure(
                const rclcpp_lifecycle::State & previous_state) override;

            hardware_interface::CallbackReturn on_activate(
                const rclcpp_lifecycle::State& previous_state) override;
            
            hardware_interface::CallbackReturn on_deactivate(
                const rclcpp_lifecycle::State & previous_state
            ) override;

            std::vector<hardware_interface::StateInterface>
                export_state_interfaces() override;
            
            std::vector<hardware_interface::CommandInterface>
                export_command_interfaces() override;
            
            hardware_interface::return_type read(
                const rclcpp::Time& time, const rclcpp::Duration& period
            ) override;

            hardware_interface::return_type write(
                const rclcpp::Time & time, const rclcpp::Duration & period
            ) override;

        private:
            
            void logTransition(const char* cb, const rclcpp_lifecycle::State & prev);
            

            /* 全部由vector，关节数由urdf决定，on_init中一次性初始化，之后不再改容量 */

            std::vector<double> true_pos_; 
            std::vector<double> pos_;           // 状态：轮子累计角度
            std::vector<double> vel_;           // 状态：轮子角速度
            std::vector<double> cmd_vel_;       // 命令：控制器写入目标的角速度

            std::vector<double> target_vel_;    // write限幅后的目标

            double max_wheel_vel_{10.0};
            double tau_{0.05};
            int encoder_cpr_{4096};

            unsigned long read_count_{0};
            unsigned long write_count_{0};

            double total_dt_{0.0};
            std::chrono::steady_clock::time_point t_start_{};

    };

}



#endif  // MY_HARDWARE__DIFF_WHEEL_HPP_
