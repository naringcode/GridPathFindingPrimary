// Global Module Fragment : Optional
module;

#include "Engine2D/Headers.h"

// Module Preamble : Required
export module PathFinding.Map:Node;

// Module Purview / Module Interface : Optional
export namespace PathFinding
{
    // Grid 계열 Node의 상태
    enum class NodeState
    {
        Invalid,

        Walkable,
        Blocked,

        // 외부에서 받아서 처리하는 것으로 한다.
        // Start,
        // Destination,
    };

    // DynamicMap과 StaticMap에서 사용되는 노드
    struct GridNode
    {
        i32 x = -1;
        i32 y = -1;

        NodeState state = NodeState::Invalid;
    };
}

// Private Module Fragment : Optional
// Private Module Fragment는 주 모듈(Primary Module) 쪽에서만 사용 가능하다.
// module: private;
