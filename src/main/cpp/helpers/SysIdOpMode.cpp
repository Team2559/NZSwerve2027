#include "helpers/SysIdOpMode.hpp"

wpi::cmd::CommandPtr SysIdOpMode::Quasistatic(wpi::cmd::sysid::Direction direction) {
    return m_routine->Quasistatic(direction).OnlyWhile([this]() {return wpi::RobotState::GetOpModeId() == m_id;});
}

wpi::cmd::CommandPtr SysIdOpMode::Dynamic(wpi::cmd::sysid::Direction direction) {
    return m_routine->Dynamic(direction).OnlyWhile([this]() {return wpi::RobotState::GetOpModeId() == m_id;});
}
