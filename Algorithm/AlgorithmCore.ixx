// Global Module Fragment : Optional
module;

#include "Engine2D/Headers.h"

// Module Preamble : Required
export module PathFinding.Algorithm:Core;

// Module Purview / Module Interface : Optional
export namespace PathFinding
{
    // sqrt(2)�� ���� �Ź� ����ؼ� ����ϴ� ���� �����ϱ� ���� ��
    constexpr f32 kSqrt2 = 1.414213562373095f;

    // Grid �̵� ���
    constexpr f32 kGridCardinalCost = 1.0f;   // ��������(����)
    constexpr f32 kGridDiagonalCost = kSqrt2; // �ϵ�, �ϼ�, ����, ����(�밢��)
    
    // Ž���� ���Ǵ� ��� ��ã�� �˰���
    enum class AlgorithmType
    {
        DepthFirst,
        BreadthFirst,
        BestFirst, // Heuristic Search
        Dijkstra,
        AStar,
        JumpPoint,
        JumpPointPlus,

        Max,
    };

    struct AlgorithmTypeInfo
    {
        const AlgorithmType type{ };
        std::string_view name{ };
    };

    const std::array<AlgorithmTypeInfo, (i32)AlgorithmType::Max> kAlgorithmTypeInfoList{{
        { AlgorithmType::DepthFirst,    "Depth First Search" },
        { AlgorithmType::BreadthFirst,  "Breadth First Search" },
        { AlgorithmType::BestFirst,     "Best First Search" },
        { AlgorithmType::Dijkstra,      "Dijkstra" },
        { AlgorithmType::AStar,         "A*" },
        { AlgorithmType::JumpPoint,     "Jump Point Search" },
        { AlgorithmType::JumpPointPlus, "JPS+" },
    }};
    
    // Grid �迭 Node���� ���� ��带 ����Ű�� ����
    enum class AdjacentDirection
    {
        /**
         *  d |     0     |   1   |     2     |
         * ------------------------------------
         *  0 | NorthWest | North | NorthEast |
         *  1 |    West   |  None |    East   |
         *  2 | SouthWest | South | SouthEast |
         *
         * dy, dx�� ������� ĳ�������� �� ��ġ�� �ٷ� �ľ��� �� �־�� �Ѵ�.
         * AdjacentDirection = (dx + 1) + ((dy + 1) * 3)
         */
        NorthWest,
        North,
        NorthEast,

        West,
        None,
        East,

        SouthWest,
        South,
        SouthEast,

        NumDirections, // ������ ����

        InvalidDirection,
    };

    // JPS���� OpenNode�� ���ư� ������ ��� ���� �� ����� �뵵�� �÷��� �ڷ���
    // (����) JPS+�� ��� 8���� Direction ���� ���ư� �� �ִ� ���ɼ��� �迭 ������ ���� ���̺��� ����ϱ� ������ AdjacentDirection�� ����.
    enum class JpsDirectionFlags
    {
        /**
         * directionFlags = openNode.directionFlags;
         * 
         * for (i32 i = 0; i < NumFlags; i++)
         * {
         *     i32 dir = (1 << i);
         * 
         *     if (directionFlags & dir)
         *     {
         *         Jump(openNode.x, openNode.y, dir);
         *     }
         * }
         */
        None = 0,

        North = 1 << 0,
        East  = 1 << 1,
        South = 1 << 2,
        West  = 1 << 3,

        NorthWest = 1 << 4,
        NorthEast = 1 << 5,
        SouthEast = 1 << 6,
        SouthWest = 1 << 7,

        AllFlags = (1 << 8) - 1,

        NumFlags = 8,
    };

    // ��θ� ����ϴ� �������� ���Ǵ� ��ã�� ���
    struct PathFindingNode // struct PathNode
    {
        i32 x = -1;
        i32 y = -1;

        const PathFindingNode* parent = nullptr;

        // ���� ������ �ʿ��ϰ� �Ǹ� ����ϵ��� �Ѵ�(����� Ȯ��, JPS �迭, �׸��� �۾� ��).
        // ���� ���� �ƴ� ���� �÷��׷� ����ؾ� �� ���� �ִ�.
        // Dir enteringDirection; // indegree
        // Dir exitingDirection;  // outdegree

        // f(n) = g(n) + h(n)
        f32 f = 0.0f; // ���� ���
        f32 g = 0.0f; // �̵� ��� ���
        f32 h = 0.0f; // �޸���ƽ ���� ���

        // JPS, JPS+
        union
        {
            i32 nextDirectionFlags = 0; // JPS���� ���� ��尡 ������ ������ �÷��׷� ��� ����
            AdjacentDirection indegreeDirection; // JPS+���� ���� ��尡 �����ߴ� ������ ���� ������ ������ ���� ���̺��� ã�� ���� �뵵
        };

        /**
         * ��带 set���� ã�� ������ �����ϱ� ���� ��� �������� set�� �� ���������� ������ �����Ѵ�.
         * Ư�� ��ġ���� Ư�� ��带 ��ȸ�ϸ� �ٷ� � ���տ� ���� �ִ��� �� �� �ְ� ó���ϴ� ���� ����.
         * 
         * 1. ��� �� �߰ߵ� ��Ȳ(isInOpenSet�� isInClosedSet�� false)
         * 2. OpenSet�� ���� �ִ� ��Ȳ(isInOpenSet�� true)
         * 3. ClosedSet�� ���� �ִ� ��Ȳ(isInClosedSet�� true)
         */ 
        bool isInOpenSet   = false;
        bool isInClosedSet = false;

        // ����� ���̵� �ƴ� ��ã�� ���� ��ü�� ���� �ĺ� ��ȣ�̴�.
        // ���⼱ ����� �����ϴ� ����� ��ã�⿡�� ����Ѵ�.
        ui32 pathFindingId = std::numeric_limits<ui32>::max();
    };

    struct PathFindingNodeComp
    {
        std::strong_ordering operator()(PathFindingNode* lhs, PathFindingNode* rhs)
        {
            // if (lhs->f < rhs->f)
            // {
            //     return std::strong_ordering::less;
            // }
            // else if (lhs->f > rhs->f)
            // {
            //     return std::strong_ordering::greater;
            // }
            // 
            // return std::strong_ordering::equal;
            
            // ����� ���� ���� ���� �켱������ �� ����.
            return std::strong_order(lhs->f, rhs->f);
        }
    };

    /**
     * ��ã�� ������ ����Ǿ��� �� �ܰ躰 ���¸� �����ϱ� ���� �ڷ����̴�.
     * Ž�� ������ �����ϱ� ������ Ŀ�ǵ� ���� ��� �����ؼ� ���÷��� ��� ���� �� �����ϸ� �ȴ�.
     * 
     * �⺻ ������ Ž���� ��� ����
     * [ ][ ][ ]    [ ][ ][ ]    [D][ ][ ]
     * [ ][ ][ ] -> [B][ ][ ] -> [X][E][ ]
     * [A][ ][ ]    [X][C][ ]    [X][C][ ]
     *
     * First Record   | Second Record        | Third Record
     * - Visited : A  | - Visited : A, B, C  | - Visited : B, D, E
     * - Open : A     | - Open : B, C        | - Open : D, E
     * - Closed : -   | - Closed : A         | - Closed : B
     * 
     * ������ ������ �� �����ؾ� �ϴ� ��
     * - ���ڵ� ������ Stack�̳� Ŀ�ǵ� ���� ������ �����ϴ� ���� ����(���� ����).
     * - OpenSet�� �ִ� ���� ���� ���� �� ���ŵ� ����.\
     * - Ž�� ������ ������ ���� �ܰ�� �Ѿ�� ���̰� �ݴ�� ������ �����ϸ� ���� �ܰ�� �ǵ��ư��� ���̴�.
     * 
     * ���� ���� �̵�
     * 1. openNodes�� ��Ҹ� RecordInfoStack[y, x]�� Push�Ѵ�(Open ���µ� ÷����).
     * 2. closedNodes�� ��Ҹ� RecordInfoStack[y, x]�� Push�Ѵ�(Closed ���µ� ÷����).
     * 3. ���� PathFindingRecord�� �̵��Ѵ�.
     * 
     * ���� ���� ����
     * 1. ���� PathFindingRecord�� �̵��Ѵ�.
     * 2. openNodes�� ��Ҹ� RecordInfoStack[y, x]���� �����Ѵ�(Pop ����).
     * 3. closedNodes�� ��Ҹ� RecordInfoStack[y, x]���� �����Ѵ�(Pop ����).
     */
    struct PathFindingRecord
    {
        enum class RecordType
        {
            None,

            Processing,  // Ž���� ������ �Ǵ� ����� ���

            Visited,     // Ž�� �������� �� ���̶� ����Ǿ��� ���
            OpenNew,     // ��尡 OpenSet�� �� �߰��� ��Ȳ
            OpenUpdated, // OpenSet�� �ִ� ����� ������ ���ŵ� ��Ȳ
            Closed,      // ��尡 ClosedSet�� �� ��Ȳ
        };

        struct RecordNode
        {
            RecordType recordType = RecordType::None;

            i32 x = -1;
            i32 y = -1;

            i32 parentX = -1;
            i32 parentY = -1;

            // f(n) = g(n) + h(n)
            f32 f = 0.0f; // ���� ���
            f32 g = 0.0f; // �̵� ��� ���
            f32 h = 0.0f; // �޸���ƽ ���� ���
        };

        // Ž���� ������ �Ǵ� ���
        RecordNode processingNode;

        // Ž�� �������κ��� ���������� ���
        std::vector<Vec2D<i32>> pathPoints;

        // Ž���� ��� ���
        std::vector<RecordNode> visitedNodes;

        // OpenSet�� ���� ���Ե� ���
        std::vector<RecordNode> openNewNodes;

        // OpenSet���� ���ŵ� ���
        std::vector<RecordNode> openUpdatedNodes;

        // ClosedSet�� ���Ե� ���
        std::vector<RecordNode> closedNodes;
        
        // PQ ���� Ƚ��
        i32 openSetEnqueuedCnt = 0;
        i32 openSetUpdatedCnt  = 0;
        i32 openSetDequeuedCnt = 0;

        /****************************************
        *      PathFindingRecord Functions      *
        ****************************************/
        void SetProcessingNode(i32 x, i32 y, i32 parentX, i32 parentY, f32 f, f32 g, f32 h)
        {
            processingNode.recordType = RecordType::Processing;

            processingNode.x = x;
            processingNode.y = y;

            processingNode.parentX = parentX;
            processingNode.parentY = parentY;

            processingNode.f = f;
            processingNode.g = g;
            processingNode.h = h;
        }

        template <typename T>
            requires std::same_as<T, std::vector<Vec2D<i32>>>
        void TakePathPoints(T&& pathPoints)
        {
            this->pathPoints = std::forward<T>(pathPoints);
        }

        void AddVisitedNode(i32 x, i32 y, i32 parentX, i32 parentY, f32 f, f32 g, f32 h)
        {
            visitedNodes.push_back({ RecordType::Visited, x, y, parentX, parentY, f, g, h });
        }

        void AddOpenNewNode(i32 x, i32 y, i32 parentX, i32 parentY, f32 f, f32 g, f32 h)
        {
            openNewNodes.push_back({ RecordType::OpenNew, x, y, parentX, parentY, f, g, h });
        }

        void AddOpenUpdatedNode(i32 x, i32 y, i32 parentX, i32 parentY, f32 f, f32 g, f32 h)
        {
            openUpdatedNodes.push_back({ RecordType::OpenUpdated, x, y, parentX, parentY, f, g, h });
        }

        void AddClosedNode(i32 x, i32 y, i32 parentX, i32 parentY, f32 f, f32 g, f32 h)
        {
            closedNodes.push_back({ RecordType::Closed, x, y, parentX, parentY, f, g, h });
        }

        void SetOpenSetInfo(i32 openSetEnqueuedCnt, i32 openSetUpdatedCnt, i32 openSetDequeuedCnt)
        {
            this->openSetEnqueuedCnt = openSetEnqueuedCnt;
            this->openSetUpdatedCnt  = openSetUpdatedCnt;
            this->openSetDequeuedCnt = openSetDequeuedCnt;
        }
    };

    // �θ� ��带 �������Ͽ� ��λ��� �̵� ��ġ�� �迭�� ��ȯ�ϴ� �Լ� 
    std::vector<Vec2D<i32>> RetracePath(const PathFindingNode& node)
    {
        const PathFindingNode* traceNode = &node;

        std::vector<Vec2D<i32>> pathNodes;

        while (traceNode != nullptr)
        {
            pathNodes.push_back({ traceNode->x, traceNode->y });

            traceNode = traceNode->parent;
        }

        std::ranges::reverse(pathNodes);

        return pathNodes;
    }

    // DeltaPos ���� AdjacentDirection���� �ٲ��ִ� �Լ�
    // dx : -1, 0, 1
    // dy : -1, 0, 1
    AdjacentDirection ConvertToNodeDirection(i32 dx, i32 dy)
    {
        i32 dir = (dx + 1) + ((dy + 1) * 3);

        if (dir < 0 || dir >= (i32)AdjacentDirection::NumDirections)
            return AdjacentDirection::InvalidDirection;

        return (AdjacentDirection)dir;
    }

    // AdjacentDirection�� { dx, dy }�� �ٲ��ִ� �Լ�
    Vec2D<i32> ConvertToDeltaPos(AdjacentDirection dir)
    {
        // static const i32 kDx[] = { -1, 0, 1, -1, 0, 1, -1, 0, 1 };
        // static const i32 kDy[] = { -1, -1, -1, 0, 0, 0, 1, 1, 1 };
        // 
        // return { kDx[(i32)dir], kDy[(i32)dir]};

        // ���� 0���� ������� �����ϸ� �����Ϸ��� �˾Ƽ� ���� ���̺��� ���� ����ȭ�Ѵ�.
        switch (dir)
        {
            case AdjacentDirection::NorthWest:
                return { -1, -1 };

            case AdjacentDirection::North:
                return { 0, -1 };

            case AdjacentDirection::NorthEast:
                return { 1, -1 };

            case AdjacentDirection::West:
                return { -1, 0 };

            case AdjacentDirection::None:
                return { 0, 0 };

            case AdjacentDirection::East:
                return { 1, 0 };

            case AdjacentDirection::SouthWest:
                return { -1, 1 };

            case AdjacentDirection::South:
                return { 0, 1 };

            case AdjacentDirection::SouthEast:
                return { 1, 1 };

            default:
                return { 0, 0 };
        }

        return { 0, 0 };
    }

    // JPS+ : AdjacentDirection �������� ������ �Ÿ��� ��ȯ�ϴ� �Լ�
    Vec2D<i32> ConvertToDeltaPos(AdjacentDirection dir, i32 amount)
    {
        Vec2D<i32> deltaPos = ConvertToDeltaPos(dir);
        {
            deltaPos.x *= amount;
            deltaPos.y *= amount;
        }

        return deltaPos;
    }

    // AdjacentDirection�� ũ�⸦ ��ȯ�ϴ� �Լ�
    f32 ConvertToDistance(AdjacentDirection dir)
    {
        switch (dir)
        {
            case AdjacentDirection::NorthWest:
                return kGridDiagonalCost;

            case AdjacentDirection::North:
                return kGridCardinalCost;

            case AdjacentDirection::NorthEast:
                return kGridDiagonalCost;

            case AdjacentDirection::West:
                return kGridCardinalCost;

            case AdjacentDirection::None:
                return 0.0f;

            case AdjacentDirection::East:
                return kGridCardinalCost;

            case AdjacentDirection::SouthWest:
                return kGridDiagonalCost;

            case AdjacentDirection::South:
                return kGridCardinalCost;

            case AdjacentDirection::SouthEast:
                return kGridDiagonalCost;

            default:
                return 0.0f;
        }

        return 0.0f;
    }

    // DeltaPos ���� JpsDirectionFlags�� �ٲ��ִ� �Լ�
    // dx : -1, 0, 1
    // dy : -1, 0, 1
    i32 ConvertToJpsDirectionFlags(i32 dx, i32 dy)
    {
        switch (dy)
        {
            // ��
            case -1:
            {
                switch (dx)
                {
                    case -1: return (i32)JpsDirectionFlags::NorthWest;
                    case  0: return (i32)JpsDirectionFlags::North;
                    case  1: return (i32)JpsDirectionFlags::NorthEast;

                    default:
                        return (i32)JpsDirectionFlags::None;
                }

                break;
            }

            // ����(?)
            case 0: 
            {
                switch (dx)
                {
                    case -1: return (i32)JpsDirectionFlags::West;
                    case  1: return (i32)JpsDirectionFlags::East;

                    default:
                        return (i32)JpsDirectionFlags::None;
                }

                break;
            }

            // ��
            case 1:
            {
                switch (dx)
                {
                    case -1: return (i32)JpsDirectionFlags::SouthWest;
                    case  0: return (i32)JpsDirectionFlags::South;
                    case  1: return (i32)JpsDirectionFlags::SouthEast;

                    default:
                        return (i32)JpsDirectionFlags::None;
                }

                break;
            }

            default:
                return (i32)JpsDirectionFlags::None;
        }

        return 0;
    }

    // JPS+ : AdjacentDirection�� ��������(�����¿�) �������� Ȯ���ϴ� �Լ�
    bool IsDirectionCardinal(AdjacentDirection adjDir)
    {
        switch (adjDir)
        {
            // case AdjacentDirection::North:
            // case AdjacentDirection::East:
            // case AdjacentDirection::South:
            // case AdjacentDirection::West:
            //     return true;

            // ���� ���̺� ����ȭ ����
            case AdjacentDirection::NorthWest:
                return false;

            case AdjacentDirection::North:
                return true;

            case AdjacentDirection::NorthEast:
                return false;

            case AdjacentDirection::West:
                return true;

            case AdjacentDirection::None:
                return false;

            case AdjacentDirection::East:
                return true;

            case AdjacentDirection::SouthWest:
                return false;

            case AdjacentDirection::South:
                return true;

            case AdjacentDirection::SouthEast:
                return false;
        }

        return false;
    }

    // JPS+ : AdjacentDirection�� �밢�� �������� Ȯ���ϴ� �Լ�
    bool IsDirectionDiagonal(AdjacentDirection adjDir)
    {
        switch (adjDir)
        {
            // case AdjacentDirection::kNorthWest:
            // case AdjacentDirection::kNorthEast:
            // case AdjacentDirection::kSouthEast:
            // case AdjacentDirection::kSouthWest:
            //     return true;

           // ���� ���̺� ����ȭ ����
            case AdjacentDirection::NorthWest:
                return true;

            case AdjacentDirection::North:
                return false;

            case AdjacentDirection::NorthEast:
                return true;

            case AdjacentDirection::West:
                return false;

            case AdjacentDirection::None:
                return false;

            case AdjacentDirection::East:
                return false;

            case AdjacentDirection::SouthWest:
                return true;

            case AdjacentDirection::South:
                return false;

            case AdjacentDirection::SouthEast:
                return true;
        }

        return false;
    }
     
    // JPS+ : ��ǥ�� ��Ȯ�� ���⿡ ���� �ִ��� Ȯ��(��������(�����¿�)�� üũ)
    // ���� �밢�� �������� ��Ȯ�� ������ Ȯ���ϰ� �ʹٸ� (abs(diffX) == abs(diffY))�� �� ��!
    bool IsDestinationInExactDirection(Vec2D<i32> here, Vec2D<i32> destPos, AdjacentDirection adjDir)
    {
        i32 diffX = destPos.x - here.x;
        i32 diffY = destPos.y - here.y;

        // !! ���� ������ ����, �Ʒ��� ������ ���� !!
        switch (adjDir)
        {
            // case AdjacentDirection::North:
            //     return (diffX == 0) && (diffY < 0);
            // 
            // case AdjacentDirection::East:
            //     return (diffX > 0) && (diffY == 0);
            // 
            // case AdjacentDirection::South:
            //     return (diffX == 0) && (diffY > 0);
            // 
            // case AdjacentDirection::West:
            //     return (diffX < 0) && (diffY == 0);

            // ���� ���̺� ����ȭ ����
            case AdjacentDirection::NorthWest:
                return false;

            case AdjacentDirection::North:
                return (diffX == 0) && (diffY < 0);

            case AdjacentDirection::NorthEast:
                return false;

            case AdjacentDirection::West:
                return (diffX < 0) && (diffY == 0);

            case AdjacentDirection::None:
                return false;

            case AdjacentDirection::East:
                return (diffX > 0) && (diffY == 0);

            case AdjacentDirection::SouthWest:
                return false;

            case AdjacentDirection::South:
                return (diffX == 0) && (diffY > 0);

            case AdjacentDirection::SouthEast:
                return false;
        }

        return false;
    }

    // JPS+ : ��ǥ�� ��и� ���⿡ ���� �ִ��� Ȯ��(�밢���� üũ)
    // ���� �� ��ġ�� ���� üũ�ϰ� �ʹٸ� IsDestinationInExactDirection()�� �����¿� �ڵ带 �ű⵵�� ����.
    bool IsDestinationInGeneralDirection(Vec2D<i32> here, Vec2D<i32> destPos, AdjacentDirection adjDir)
    {
        i32 diffX = destPos.x - here.x;
        i32 diffY = destPos.y - here.y;
    
        // !! ���� ������ ����, �Ʒ��� ������ ���� !!
        switch (adjDir)
        {
            // case AdjacentDirection::NorthWest:
            //     return (diffX < 0) && (diffY < 0);
            // 
            // case AdjacentDirection::NorthEast:
            //     return (diffX > 0) && (diffY < 0);
            // 
            // case AdjacentDirection::SouthEast:
            //     return (diffX > 0) && (diffY > 0);
            // 
            // case AdjacentDirection::SouthWest:
            //     return (diffX < 0) && (diffY > 0);

            // ���� ���̺� ����ȭ ����
            case AdjacentDirection::NorthWest:
                return (diffX < 0) && (diffY < 0);

            case AdjacentDirection::North:
                return false;

            case AdjacentDirection::NorthEast:
                return (diffX > 0) && (diffY < 0);

            case AdjacentDirection::West:
                return false;

            case AdjacentDirection::None:
                return false;

            case AdjacentDirection::East:
                return false;

            case AdjacentDirection::SouthWest:
                return (diffX < 0) && (diffY > 0);

            case AdjacentDirection::South:
                return false;

            case AdjacentDirection::SouthEast:
                return (diffX > 0) && (diffY > 0);
        }
    
        return false;
    }
}

// Private Module Fragment : Optional
// Private Module Fragment�� �� ���(Primary Module) �ʿ����� ��� �����ϴ�.
// module: private;
