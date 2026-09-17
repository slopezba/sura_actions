#pragma once

#include "sura_actions/usv/planar_action_server.hpp"

namespace sura_actions::usv
{
using GoToPoseLifecycleActionServer = PlanarActionServer<sura_actions::action::GoToPoseUsv>;
}  // namespace sura_actions::usv
