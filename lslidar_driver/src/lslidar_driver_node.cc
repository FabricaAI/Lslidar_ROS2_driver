/*
 * This file is part of lslidar driver.
 *
 * The driver is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * The driver is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with the driver.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "lslidar_driver/lslidar_driver.h"
#include "rclcpp/rclcpp.hpp"

using namespace lslidar_driver;
volatile sig_atomic_t flag = 1;

bool should_shutdown_node { false };
static void sigint_handler(int) {
    printf("Recevied sigint\n");
    should_shutdown_node = true;
}

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<lslidar_driver::LslidarDriver>();
    signal(SIGINT, sigint_handler);

    while (rclcpp::ok() && !should_shutdown_node) {
        rclcpp::spin_some(node);
    }
    node->ShutdownDriver();
    rclcpp::shutdown();
    return 0;
}
