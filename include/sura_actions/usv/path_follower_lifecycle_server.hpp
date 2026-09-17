#pragma once

#include "sura_actions/usv/planar_action_server.hpp"

namespace sura_actions::usv
{
using PathFollowerLifecycleServer = PlanarActionServer<sura_actions::action::FollowPathUsv>;
}  // namespace sura_actions::usv
