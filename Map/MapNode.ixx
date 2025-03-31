// Global Module Fragment : Optional
module;

#include "Engine2D/Headers.h"

// Module Preamble : Required
export module PathFinding.Map:Node;

// Module Purview / Module Interface : Optional
export namespace PathFinding
{
    // Grid �迭 Node�� ����
    enum class NodeState
    {
        Invalid,

        Walkable,
        Blocked,

        // �ܺο��� �޾Ƽ� ó���ϴ� ������ �Ѵ�.
        // Start,
        // Destination,
    };

    // DynamicMap�� StaticMap���� ���Ǵ� ���
    struct GridNode
    {
        i32 x = -1;
        i32 y = -1;

        NodeState state = NodeState::Invalid;
    };
}

// Private Module Fragment : Optional
// Private Module Fragment�� �� ���(Primary Module) �ʿ����� ��� �����ϴ�.
// module: private;
